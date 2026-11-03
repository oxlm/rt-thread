/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-11-03     RT-Thread    gravity-referenced three-axis microphone processing
 */

#include "mic_algo.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MIC_ALGO_PI 3.14159265358979323846f

static int mic_algo_valid_config(const mic_algo_config_t *config)
{
    float mount_norm = 0.0f;
    float nyquist = (float)config->sample_rate * 0.5f;
    uint32_t axis;

    if ((config->sample_rate < 1000U) || (config->sample_rate > 1000000U) ||
        (config->startup_ms < 10U) || (config->startup_ms > 10000U))
    {
        return 0;
    }
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        if (!isfinite(config->zero_adc[axis]) ||
            (config->zero_adc[axis] < 0.0f) || (config->zero_adc[axis] > 65535.0f) ||
            !isfinite(config->counts_per_g[axis]) ||
            (config->counts_per_g[axis] < 1.0f) ||
            (config->counts_per_g[axis] > 65535.0f) ||
            !isfinite(config->mount_direction[axis]))
        {
            return 0;
        }
        mount_norm += config->mount_direction[axis] * config->mount_direction[axis];
    }
    return isfinite(mount_norm) && (mount_norm > 1.0e-12f) &&
           isfinite(config->gravity_cutoff_hz) && (config->gravity_cutoff_hz > 0.0f) &&
           (config->gravity_cutoff_hz < 100.0f) &&
           isfinite(config->gravity_min_g) && (config->gravity_min_g > 0.0f) &&
           isfinite(config->gravity_max_g) &&
           (config->gravity_max_g > config->gravity_min_g) &&
           (config->gravity_max_g <= 32.0f) &&
           isfinite(config->motion_rms_g) && (config->motion_rms_g > 0.0f) &&
           (config->motion_rms_g <= 32.0f) &&
           isfinite(config->min_projection) && (config->min_projection > 0.0f) &&
           (config->min_projection <= 1.0f) &&
           isfinite(config->highpass_hz) && (config->highpass_hz > 0.0f) &&
           (config->highpass_hz < nyquist) &&
           isfinite(config->lowpass_hz) && (config->lowpass_hz >= 0.0f) &&
           (config->lowpass_hz < nyquist) &&
           isfinite(config->fade_ms) && (config->fade_ms >= 1.0f) &&
           (config->fade_ms <= 1000.0f) &&
           isfinite(config->pcm_per_g) && (config->pcm_per_g > 0.0f) &&
           (config->pcm_per_g <= 1000000.0f) &&
           isfinite(config->gain) && (config->gain >= 0.0f) &&
           (config->gain <= 1000000.0f);
}

int mic_algo_init(mic_algo_t *ctx, const mic_algo_config_t *config)
{
    mic_algo_config_t parameters;
    float mount_norm = 0.0f;
    float k;
    float norm;
    uint32_t axis;

    if (ctx == NULL)
    {
        return -1;
    }
    if (config == NULL)
    {
        mic_algo_deinit(ctx);
        return -1;
    }
    parameters = *config;
    mic_algo_deinit(ctx);
    if (!mic_algo_valid_config(&parameters))
    {
        return -1;
    }
    ctx->config = parameters;
    ctx->window_frames = parameters.sample_rate / 1000U;
    ctx->startup_frames = (uint32_t)(((uint64_t)parameters.sample_rate *
                                    parameters.startup_ms) / 1000U);
    ctx->gravity_alpha = 1.0f - expf(-2.0f * MIC_ALGO_PI * parameters.gravity_cutoff_hz *
                                    (float)ctx->window_frames / (float)parameters.sample_rate);
    ctx->fade_alpha = 1.0f - expf(-1000.0f /
                                 ((float)parameters.sample_rate * parameters.fade_ms));
    k = tanf(MIC_ALGO_PI * parameters.highpass_hz / (float)parameters.sample_rate);
    ctx->hp_alpha = (1.0f - k) / (1.0f + k);
    ctx->hp_gain = 1.0f / (1.0f + k);
    if (parameters.lowpass_hz > 0.0f)
    {
        /* Second-order Butterworth, transposed direct form II. */
        k = tanf(MIC_ALGO_PI * parameters.lowpass_hz / (float)parameters.sample_rate);
        norm = 1.0f / (1.0f + 1.41421356237f * k + k * k);
        ctx->lp_b0 = k * k * norm;
        ctx->lp_b1 = 2.0f * ctx->lp_b0;
        ctx->lp_b2 = ctx->lp_b0;
        ctx->lp_a1 = 2.0f * (k * k - 1.0f) * norm;
        ctx->lp_a2 = (1.0f - 1.41421356237f * k + k * k) * norm;
    }
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        mount_norm += parameters.mount_direction[axis] * parameters.mount_direction[axis];
        ctx->inv_counts_per_g[axis] = 1.0f / parameters.counts_per_g[axis];
    }
    mount_norm = sqrtf(mount_norm);
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        ctx->config.mount_direction[axis] /= mount_norm;
    }
    ctx->initialized = 1U;
    return 0;
}

static void mic_algo_update_direction(mic_algo_t *ctx)
{
    float normal[MIC_ALGO_AXES];
    float projection[MIC_ALGO_AXES];
    float norm = 0.0f;
    float dot = 0.0f;
    uint32_t axis;

    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        norm += ctx->gravity[axis] * ctx->gravity[axis];
    }
    ctx->direction_valid = 0U;
    if (norm < ctx->config.gravity_min_g * ctx->config.gravity_min_g)
    {
        return;
    }
    norm = sqrtf(norm);
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        normal[axis] = ctx->gravity[axis] / norm;
        dot += normal[axis] * ctx->config.mount_direction[axis];
    }
    norm = 0.0f;
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        projection[axis] = ctx->config.mount_direction[axis] - dot * normal[axis];
        norm += projection[axis] * projection[axis];
    }
    if (norm < ctx->config.min_projection * ctx->config.min_projection)
    {
        return;
    }
    norm = sqrtf(norm);
    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        ctx->direction[axis] = projection[axis] / norm;
    }
    ctx->direction_valid = 1U;
}

static void mic_algo_update_gravity(mic_algo_t *ctx)
{
    float mean[MIC_ALGO_AXES];
    float norm = 0.0f;
    float variance;
    uint32_t axis;

    for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
    {
        mean[axis] = ctx->window_sum[axis] / (float)ctx->window_frames;
        norm += mean[axis] * mean[axis];
        ctx->window_sum[axis] = 0.0f;
    }
    variance = ctx->window_energy / (float)ctx->window_frames - norm;
    ctx->window_energy = 0.0f;
    ctx->window_count = 0U;
    /* This gate rejects some motion/vibration, but cannot distinguish sustained
     * linear acceleration from gravity. After startup, rejected windows freeze
     * the last estimate rather than replacing it with a motion vector. */
    if ((norm < ctx->config.gravity_min_g * ctx->config.gravity_min_g) ||
        (norm > ctx->config.gravity_max_g * ctx->config.gravity_max_g) ||
        (variance > ctx->config.motion_rms_g * ctx->config.motion_rms_g))
    {
        if (!ctx->gravity_ready)
        {
            ctx->stable_frames = 0U;
            memset(ctx->startup_sum, 0, sizeof(ctx->startup_sum));
        }
        return;
    }
    if (!ctx->gravity_ready)
    {
        ctx->stable_frames += ctx->window_frames;
        for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
        {
            ctx->startup_sum[axis] += mean[axis] * (float)ctx->window_frames;
        }
        if (ctx->stable_frames < ctx->startup_frames)
        {
            return;
        }
        for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
        {
            ctx->gravity[axis] = ctx->startup_sum[axis] / (float)ctx->stable_frames;
        }
        ctx->gravity_ready = 1U;
    }
    else
    {
        for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
        {
            ctx->gravity[axis] += ctx->gravity_alpha * (mean[axis] - ctx->gravity[axis]);
        }
    }
    mic_algo_update_direction(ctx);
}

int mic_algo_process(mic_algo_t *ctx, const uint16_t *xyz, uint32_t frames,
                     int16_t *mono, uint32_t mono_capacity)
{
    uint32_t frame;
    uint32_t axis;

    if ((ctx == NULL) || !ctx->initialized || (frames > mono_capacity) ||
        (frames > (uint32_t)INT_MAX / MIC_ALGO_AXES) ||
        ((frames != 0U) && ((xyz == NULL) || (mono == NULL))))
    {
        return -1;
    }
    for (frame = 0U; frame < frames; frame++)
    {
        float vibration[MIC_ALGO_AXES];
        float sample = 0.0f;
        float target;

        for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
        {
            float acceleration = ((float)xyz[frame * MIC_ALGO_AXES + axis] -
                                  ctx->config.zero_adc[axis]) * ctx->inv_counts_per_g[axis];

            ctx->window_sum[axis] += acceleration;
            ctx->window_energy += acceleration * acceleration;
            if (!ctx->hp_seeded)
            {
                ctx->hp_prev_in[axis] = acceleration;
            }
            vibration[axis] = ctx->hp_alpha * ctx->hp_prev_out[axis] +
                              ctx->hp_gain * (acceleration - ctx->hp_prev_in[axis]);
            ctx->hp_prev_in[axis] = acceleration;
            ctx->hp_prev_out[axis] = vibration[axis];
        }
        ctx->hp_seeded = 1U;
        ctx->window_count++;
        if (ctx->window_count == ctx->window_frames)
        {
            mic_algo_update_gravity(ctx);
        }
        for (axis = 0U; axis < MIC_ALGO_AXES; axis++)
        {
            sample += vibration[axis] * ctx->direction[axis];
        }
        if (ctx->config.lowpass_hz > 0.0f)
        {
            float filtered = ctx->lp_b0 * sample + ctx->lp_z1;

            ctx->lp_z1 = ctx->lp_b1 * sample - ctx->lp_a1 * filtered + ctx->lp_z2;
            ctx->lp_z2 = ctx->lp_b2 * sample - ctx->lp_a2 * filtered;
            sample = filtered;
        }
        target = ctx->direction_valid ? 1.0f : 0.0f;
        ctx->envelope += ctx->fade_alpha * (target - ctx->envelope);
        if ((target == 0.0f) && (ctx->envelope < 1.0e-6f))
        {
            ctx->envelope = 0.0f;
        }
        sample *= ctx->envelope * ctx->config.pcm_per_g * ctx->config.gain;
        if (sample >= 32767.0f)
        {
            mono[frame] = 32767;
        }
        else if (sample <= -32768.0f)
        {
            mono[frame] = -32768;
        }
        else
        {
            mono[frame] = (int16_t)lrintf(sample);
        }
    }
    return (int)frames;
}

void mic_algo_deinit(mic_algo_t *ctx)
{
    if (ctx != NULL)
    {
        memset(ctx, 0, sizeof(*ctx));
    }
}
