/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-29     RT-Thread    polyphase sinc upsampler for the USB ADC microphone
 * 2026-10-04     RT-Thread    configured by sample rates, one-shot int16 processing
 */

#include <string.h>

#include <rtthread.h>

#include "mic_resample.h"

#define MIC_RESM_H2_TAPS 23U
#define MIC_RESM_H3_TAPS 16U

static const float mic_resm_h2[] =
{
    /* phase 0: h[0..44 step 2] */
    -4.07854162e-04f, 1.29527045e-03f, -2.89261061e-03f, 5.49372833e-03f,
    -9.46974093e-03f, 1.53207000e-02f, -2.38054563e-02f, 3.62793744e-02f,
    -5.56831195e-02f, 9.01942737e-02f, -1.74419387e-01f, 8.99069198e-01f,
    2.96667656e-01f, -1.20828011e-01f, 7.00711725e-02f, -4.48094515e-02f,
    2.94173639e-02f, -1.91696077e-02f, 1.21219084e-02f, -7.28260541e-03f,
    4.04669356e-03f, -1.98832030e-03f, 7.78825595e-04f,
    /* phase 1: h[1..45 step 2] */
    7.78825595e-04f, -1.98832030e-03f, 4.04669356e-03f, -7.28260541e-03f,
    1.21219084e-02f, -1.91696077e-02f, 2.94173639e-02f, -4.48094515e-02f,
    7.00711725e-02f, -1.20828011e-01f, 2.96667656e-01f, 8.99069198e-01f,
    -1.74419387e-01f, 9.01942737e-02f, -5.56831195e-02f, 3.62793744e-02f,
    -2.38054563e-02f, 1.53207000e-02f, -9.46974093e-03f, 5.49372833e-03f,
    -2.89261061e-03f, 1.29527045e-03f, -4.07854162e-04f
};

static const float mic_resm_h3[] =
{
    /* phase 0: h[0..45 step 3] */
    -4.14326616e-04f, 1.91881991e-03f, -5.17467032e-03f, 1.12217381e-02f,
    -2.16681705e-02f, 3.97651211e-02f, -7.53266869e-02f, 1.85553263e-01f,
    9.54030341e-01f, -1.28871717e-01f, 6.01831424e-02f, -3.25295548e-02f,
    1.75535290e-02f, -8.81427490e-03f, 3.83810589e-03f, -1.26465935e-03f,
    /* phase 1: h[1..46 step 3] */
    -1.54488941e-03f, 5.53140612e-03f, -1.36363168e-02f, 2.82175562e-02f,
    -5.31942282e-02f, 9.74996829e-02f, -1.93089510e-01f, 6.30216299e-01f,
    6.30216299e-01f, -1.93089510e-01f, 9.74996829e-02f, -5.31942282e-02f,
    2.82175562e-02f, -1.36363168e-02f, 5.53140612e-03f, -1.54488941e-03f,
    /* phase 2: h[2..47 step 3] */
    -1.26465935e-03f, 3.83810589e-03f, -8.81427490e-03f, 1.75535290e-02f,
    -3.25295548e-02f, 6.01831424e-02f, -1.28871717e-01f, 9.54030341e-01f,
    1.85553263e-01f, -7.53266869e-02f, 3.97651211e-02f, -2.16681705e-02f,
    1.12217381e-02f, -5.17467032e-03f, 1.91881991e-03f, -4.14326616e-04f
};

static void resm_stage_reset(struct mic_resm_stage *st)
{
    (void)memset(st->st, 0, sizeof(st->st));
}

/* Pick the 2x or 3x table and clear the phase history, so the stream does not
 * start with a garbage burst. */
static void resm_stage_select(struct mic_resm_stage *st, uint8_t L)
{
    if (L == 2U)
    {
        st->L = 2U;
        st->taps = MIC_RESM_H2_TAPS;
        st->coeffs = mic_resm_h2;
    }
    else
    {
        st->L = 3U;
        st->taps = MIC_RESM_H3_TAPS;
        st->coeffs = mic_resm_h3;
    }

    resm_stage_reset(st);
}

static int resm_pick_stages(uint32_t factor, uint8_t *nstages, uint8_t *L)
{
    switch (factor)
    {
    case 1U:
        *nstages = 0U;
        break;
    case 2U:
        L[0] = 2U;
        *nstages = 1U;
        break;
    case 3U:
        L[0] = 3U;
        *nstages = 1U;
        break;
    case 4U:
        L[0] = 2U;
        L[1] = 2U;
        *nstages = 2U;
        break;
    case 6U:
        L[0] = 2U;
        L[1] = 3U;
        *nstages = 2U;
        break;
    default:
        return -1;
    }

    return 0;
}

/* Both directions are plain casts with no 1/32768 normalisation: every phase
 * sums to 1.0, so the DC gain is 1.0 and the int16 range maps onto itself. */
static void resm_s16_to_f32(const int16_t *in, float *out, uint32_t frames)
{
    uint32_t i;
    uint32_t n = frames * MIC_RESM_CHANNELS;

    for (i = 0U; i < n; i++)
    {
        out[i] = (float)in[i];
    }
}

static void resm_f32_to_s16(const float *in, int16_t *out, uint32_t frames)
{
    uint32_t i;
    uint32_t n = frames * MIC_RESM_CHANNELS;

    for (i = 0U; i < n; i++)
    {
        float v = in[i];

        if (v > 32767.0f)
        {
            v = 32767.0f;
        }
        if (v < -32768.0f)
        {
            v = -32768.0f;
        }

        out[i] = (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
    }
}

static uint32_t resm_stage_run(struct mic_resm_stage *st, const float *in,
                               uint32_t in_frames, float *out)
{
    const uint8_t L = st->L;
    const uint8_t taps = st->taps;
    const float *h = st->coeffs;
    uint32_t i;
    uint32_t p;
    uint32_t k;
    uint32_t c;

    for (i = 0U; i < in_frames; i++)
    {
        /* Shift each channel's history right, newest sample at k == 0. */
        for (c = 0U; c < MIC_RESM_CHANNELS; c++)
        {
            float *hist = st->st[c];

            for (k = taps; k > 0U; k--)
            {
                hist[k] = hist[k - 1];
            }
            hist[0] = in[i * MIC_RESM_CHANNELS + c];
        }

        for (p = 0U; p < L; p++)
        {
            const float *hp = h + (size_t)p * taps;
            uint32_t oidx = (i * (uint32_t)L + p) * MIC_RESM_CHANNELS;

            for (c = 0U; c < MIC_RESM_CHANNELS; c++)
            {
                float acc = 0.0f;

                for (k = 0U; k < taps; k++)
                {
                    acc += hp[k] * st->st[c][k];
                }
                out[oidx + c] = acc;
            }
        }
    }

    return in_frames * (uint32_t)L;
}

void mic_resampler_deinit(mic_resampler_t *r)
{
    uint32_t n;

    if (r == RT_NULL)
    {
        return;
    }

    if (r->blk_in != RT_NULL)
    {
        rt_free(r->blk_in);
        r->blk_in = RT_NULL;
    }

    /* Walk the whole array, not just nstages: a narrower configuration has to
     * collect whatever a wider one allocated before it. */
    for (n = 0U; n < MIC_RESM_MAX_STAGES; n++)
    {
        if (r->blk_mid[n] != RT_NULL)
        {
            rt_free(r->blk_mid[n]);
            r->blk_mid[n] = RT_NULL;
        }
    }

    r->frames_in = 0U;
    r->factor = 1U;
    r->nstages = 0U;
}

int mic_resampler_init(mic_resampler_t *r, uint32_t src_rate, uint32_t dst_rate)
{
    uint8_t nstages;
    uint8_t L[MIC_RESM_MAX_STAGES];
    uint32_t factor;
    uint32_t step;
    uint32_t n;

    if ((r == RT_NULL) || (src_rate == 0U) || (dst_rate < src_rate))
    {
        return -1;
    }

    factor = dst_rate / src_rate;
    if ((factor * src_rate) != dst_rate)
    {
        /* Not an exact integer multiple; no table covers it. */
        return -1;
    }
    if (resm_pick_stages(factor, &nstages, L) != 0)
    {
        return -1;
    }
    if ((src_rate / 1000U) == 0U)
    {
        /* Less than a frame per ms; the block buffers are sized for a whole ms. */
        return -1;
    }

    /* Validate before releasing, so a rejected configuration cannot destroy a
     * working one. */
    mic_resampler_deinit(r);

    r->frames_in = src_rate / 1000U;
    r->factor = factor;
    r->nstages = nstages;

    if (nstages == 0U)
    {
        return 0;
    }

    for (n = 0U; n < nstages; n++)
    {
        resm_stage_select(&r->stages[n], L[n]);
    }

    /* One ms of interleaved float per block; each stage widens it by its factor. */
    step = r->frames_in * MIC_RESM_CHANNELS;
    r->blk_in = (float *)rt_malloc(step * sizeof(float));
    if (r->blk_in == RT_NULL)
    {
        goto fail;
    }

    for (n = 0U; n < nstages; n++)
    {
        step *= L[n];
        r->blk_mid[n] = (float *)rt_malloc(step * sizeof(float));
        if (r->blk_mid[n] == RT_NULL)
        {
            goto fail;
        }
    }

    return 0;

fail:
    mic_resampler_deinit(r);
    return -1;
}

uint32_t mic_resampler_process(mic_resampler_t *r, const int16_t *in,
                               uint32_t in_frames, int16_t *out)
{
    uint32_t in_done = 0U;
    uint32_t out_done = 0U;
    uint32_t n;

    if ((r == RT_NULL) || (r->frames_in == 0U))
    {
        return 0U;
    }

    if (r->nstages == 0U)
    {
        /* Same rate, nothing to filter: copy the frames straight across. */
        (void)memcpy(out, in, (size_t)in_frames * MIC_RESM_CHANNELS * sizeof(int16_t));
        return in_frames;
    }

    /* Blocks hold one ms of capture samples. */
    while (in_done < in_frames)
    {
        uint32_t consumed;
        uint32_t out_blk;
        const float *src;
        float *dst;

        consumed = in_frames - in_done;
        if (consumed > r->frames_in)
        {
            consumed = r->frames_in;
        }

        resm_s16_to_f32(in + in_done * MIC_RESM_CHANNELS, r->blk_in, consumed);
        src = r->blk_in;
        out_blk = consumed;

        for (n = 0U; n < r->nstages; n++)
        {
            dst = r->blk_mid[n];
            out_blk = resm_stage_run(&r->stages[n], src, out_blk, dst);
            src = dst;
        }

        resm_f32_to_s16(src, out + out_done * MIC_RESM_CHANNELS, out_blk);

        in_done += consumed;
        out_done += out_blk;
    }

    return out_done;
}
