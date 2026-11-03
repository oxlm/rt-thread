/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-11-03     RT-Thread    gravity-referenced three-axis microphone processing
 */

#ifndef __MIC_ALGO_H__
#define __MIC_ALGO_H__

#include <stdint.h>

#define MIC_ALGO_AXES 3U

typedef struct
{
    uint32_t sample_rate;
    uint32_t startup_ms;
    float zero_adc[MIC_ALGO_AXES];
    float counts_per_g[MIC_ALGO_AXES];
    float mount_direction[MIC_ALGO_AXES];
    float gravity_cutoff_hz;
    float gravity_min_g;
    float gravity_max_g;
    float motion_rms_g;
    float min_projection;
    float highpass_hz;
    float lowpass_hz;                  /* 0 bypasses the output low-pass. */
    float fade_ms;
    float pcm_per_g;
    float gain;
} mic_algo_config_t;

/* Caller-owned storage; members below are private algorithm state. No heap,
 * hardware or operating-system resources are owned by this module. A context
 * must only be used by one caller at a time, including init/deinit. */
typedef struct
{
    mic_algo_config_t config;
    float inv_counts_per_g[MIC_ALGO_AXES];
    float gravity[MIC_ALGO_AXES];
    float direction[MIC_ALGO_AXES];
    float window_sum[MIC_ALGO_AXES];
    float startup_sum[MIC_ALGO_AXES];
    float hp_prev_in[MIC_ALGO_AXES];
    float hp_prev_out[MIC_ALGO_AXES];
    float window_energy;
    float gravity_alpha;
    float fade_alpha;
    float hp_alpha;
    float hp_gain;
    float lp_b0;
    float lp_b1;
    float lp_b2;
    float lp_a1;
    float lp_a2;
    float lp_z1;
    float lp_z2;
    float envelope;
    uint32_t window_frames;
    uint32_t window_count;
    uint32_t startup_frames;
    uint32_t stable_frames;
    uint8_t initialized;
    uint8_t hp_seeded;
    uint8_t gravity_ready;
    uint8_t direction_valid;
} mic_algo_t;

/* Returns 0 on success, -1 for invalid arguments. Reinitialization clears all
 * history; an invalid configuration leaves ctx deinitialized. */
int mic_algo_init(mic_algo_t *ctx, const mic_algo_config_t *config);

/* xyz contains frames interleaved unsigned ADC triplets X,Y,Z, with FIFO
 * metadata already removed and DC intact. mono_capacity is in int16 samples.
 * Produces one signed sample per triplet at the unchanged sample rate; returns
 * frames, or -1 without processing on invalid arguments. Startup produces
 * silence, followed by a fade-in. History and the internal 1 ms gravity windows
 * persist across calls of any size. Input and output buffers must not overlap. */
int mic_algo_process(mic_algo_t *ctx, const uint16_t *xyz, uint32_t frames,
                     int16_t *mono, uint32_t mono_capacity);

/* Clears history and invalidates the context. NULL and repeated calls are safe. */
void mic_algo_deinit(mic_algo_t *ctx);

#endif /* __MIC_ALGO_H__ */
