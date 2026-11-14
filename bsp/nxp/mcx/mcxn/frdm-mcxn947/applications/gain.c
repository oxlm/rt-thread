/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-11-11     RT-Thread    reusable PCM digital gain
 * 2026-11-14     RT-Thread    separate mute and target volume controls
 */

#include "gain.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

int gain_init(gain_t *ctx, float max_db, float min_db,
              bool mute, uint32_t slew_ms)
{
    float maximum;
    float minimum;

    if ((ctx == NULL) || !isfinite(max_db) || !isfinite(min_db) ||
        (min_db > max_db))
    {
        return -1;
    }
    maximum = powf(10.0f, max_db / 20.0f);
    minimum = powf(10.0f, min_db / 20.0f);
    if (!isfinite(maximum) || (minimum <= 0.0f))
    {
        return -1;
    }

    ctx->max_db = max_db;
    ctx->min_db = min_db;
    ctx->target_db = max_db;
    ctx->current_db = mute ? min_db : max_db;
    ctx->db_per_ms = (slew_ms == 0U) ? 0.0f : (max_db - min_db) / slew_ms;
    ctx->mute = mute;
    ctx->initialized = true;
    return 0;
}

int gain_set_mute(gain_t *ctx, bool mute)
{
    if ((ctx == NULL) || !ctx->initialized)
    {
        return -1;
    }
    if (ctx->mute != mute)
    {
        ctx->mute = mute;
        ctx->current_db = (mute || (ctx->db_per_ms > 0.0f)) ?
                          ctx->min_db : ctx->target_db;
    }
    return 0;
}

int gain_set_volume(gain_t *ctx, float target_db)
{
    if ((ctx == NULL) || !ctx->initialized || !isfinite(target_db))
    {
        return -1;
    }
    if (target_db > ctx->max_db)
    {
        target_db = ctx->max_db;
    }
    else if (target_db < ctx->min_db)
    {
        target_db = ctx->min_db;
    }
    ctx->target_db = target_db;
    if (!ctx->mute && (ctx->db_per_ms == 0.0f))
    {
        ctx->current_db = target_db;
    }
    return 0;
}

int gain_process(gain_t *ctx, int16_t *pcm, uint32_t frames,
                 uint32_t channels, uint32_t sample_rate)
{
    uint32_t frame;
    uint32_t channel;
    size_t offset = 0U;
    float step_db;
    float coefficient;
    float multiplier = 1.0f;

    if ((ctx == NULL) || !ctx->initialized || (channels == 0U) ||
        (sample_rate == 0U) || ((pcm == NULL) && (frames != 0U)) ||
        ((size_t)frames > SIZE_MAX / sizeof(*pcm) / channels))
    {
        return -1;
    }
    if (frames == 0U)
    {
        return 0;
    }
    if (ctx->mute)
    {
        memset(pcm, 0, (size_t)frames * channels * sizeof(*pcm));
        return 0;
    }

    step_db = ctx->db_per_ms * 1000.0f / sample_rate;
    coefficient = powf(10.0f, ctx->current_db / 20.0f);
    if (fabsf(ctx->target_db - ctx->current_db) > step_db)
    {
        float signed_step = (ctx->target_db > ctx->current_db) ? step_db : -step_db;

        multiplier = powf(10.0f, signed_step / 20.0f);
    }
    for (frame = 0U; frame < frames; frame++)
    {
        if (ctx->current_db != ctx->target_db)
        {
            float distance = ctx->target_db - ctx->current_db;

            if (fabsf(distance) <= step_db)
            {
                ctx->current_db = ctx->target_db;
                coefficient = powf(10.0f, ctx->target_db / 20.0f);
            }
            else
            {
                ctx->current_db += (distance > 0.0f) ? step_db : -step_db;
                coefficient *= multiplier;
                /* Extreme valid ranges may overflow/underflow the per-frame
                 * multiplier. The absolute dB value remains representable. */
                if (!isfinite(coefficient) || (coefficient <= 0.0f))
                {
                    coefficient = powf(10.0f, ctx->current_db / 20.0f);
                }
            }
        }
        for (channel = 0U; channel < channels; channel++)
        {
            float sample = pcm[offset] * coefficient;

            if (sample >= 32767.0f)
            {
                pcm[offset] = 32767;
            }
            else if (sample <= -32768.0f)
            {
                pcm[offset] = -32768;
            }
            else
            {
                pcm[offset] = (int16_t)lrintf(sample);
            }
            offset++;
        }
    }
    return 0;
}

void gain_deinit(gain_t *ctx)
{
    if (ctx != NULL)
    {
        memset(ctx, 0, sizeof(*ctx));
    }
}
