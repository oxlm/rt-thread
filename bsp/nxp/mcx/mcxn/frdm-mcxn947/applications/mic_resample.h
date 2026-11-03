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

#ifndef __MIC_RESMAMPLE_H__
#define __MIC_RESMAMPLE_H__

#include <stdint.h>
#include "drv_mic.h"

/* Channels are interleaved (frame-major), matching the driver's record pipe. */
#define MIC_RESM_CHANNELS  DRV_MIC_CHANNELS
/* Largest taps-per-phase of any shipped filter; sizes the phase history. */
#define MIC_RESM_MAX_TAPS  24U
/* Two cascaded stages reach the widest supported ratio, 6x. */
#define MIC_RESM_MAX_STAGES  2U

/* One polyphase stage. st[c][k] is the k-th most recent input sample of
 * channel c, st[0] = newest, so the filter is a plain dot product. */
struct mic_resm_stage
{
    uint8_t L;                          /* upsample factor, == number of phases */
    uint8_t taps;                       /* taps per phase */
    const float *coeffs;                /* L * taps, index p * taps + k */
    float st[MIC_RESM_CHANNELS][MIC_RESM_MAX_TAPS];
};

/* The rate-dependent float intermediates live on the heap, allocated by init()
 * and sized for one ms each; only the phase history is embedded here. */
typedef struct
{
    uint32_t frames_in;                 /* frames in one ms, src_rate / 1000 */
    uint32_t factor;                    /* dst_rate / src_rate */
    uint8_t nstages;                    /* 0 = passthrough */
    struct mic_resm_stage stages[MIC_RESM_MAX_STAGES];
    float *blk_in;                      /* one ms of interleaved float */
    float *blk_mid[MIC_RESM_MAX_STAGES];
} mic_resampler_t;

/* Configure for a capture rate and a target rate. The target rate must be an
 * exact integer multiple of the capture rate with a ratio built from the shipped
 * 2x / 3x tables, i.e. 1, 2, 3, 4 or 6. Calling it again releases the previous
 * allocation, so it is safe to use for a rate change while streaming.
 *
 * Returns 0 on success, -1 otherwise; on failure the previous configuration is
 * left intact. */
int mic_resampler_init(mic_resampler_t *r, uint32_t src_rate, uint32_t dst_rate);

/* Convert in_frames interleaved int16 frames and write the resampled int16
 * frames to out; returns the number of output frames written. A passthrough
 * configuration copies without filtering. in_frames may exceed one ms.
 * out must hold in_frames * factor frames. */
uint32_t mic_resampler_process(mic_resampler_t *r, const int16_t *in,
                               uint32_t in_frames, int16_t *out);

/* Release the intermediates allocated by mic_resampler_init(). Idempotent, and
 * safe on a never-initialised instance. The instance is unusable afterwards and
 * needs a fresh init. */
void mic_resampler_deinit(mic_resampler_t *r);

#endif /* __MIC_RESMAMPLE_H__ */
