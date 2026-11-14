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

#ifndef __GAIN_H__
#define __GAIN_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Caller-owned state. Treat members as private and serialize calls on one
 * context. No heap, hardware or OS resources are owned by this module. */
typedef struct
{
    float max_db;
    float min_db;
    float target_db;
    float current_db;
    float db_per_ms;
    bool mute;
    bool initialized;
} gain_t;

/* Set the usable dB range and the time for a full-range volume change.
 * slew_ms == 0 changes volume immediately. The initial target is max_db.
 * min_db is the lowest nonzero level; mute is separate and outputs zeros.
 * Returns 0 on success, -1 for invalid arguments without changing ctx. */
int gain_init(gain_t *ctx, float max_db, float min_db,
              bool mute, uint32_t slew_ms);

/* Mute immediately without losing the target volume. Unmute starts at min_db
 * and slews to the saved target, or restores it immediately if slew_ms is 0.
 * Repeated calls with the same state do not restart a transition. */
int gain_set_mute(gain_t *ctx, bool mute);

/* Clamp the target to [min_db, max_db]. While muted this only stores the target;
 * no output gain is changed until unmute. All controls return 0 or -1. */
int gain_set_volume(gain_t *ctx, float target_db);

/* In-place interleaved signed 16-bit PCM. All channels share one gain per frame.
 * Slew speed is constant in dB per second; partial moves take proportionally
 * less time. The sample rate may change between calls without reinitializing.
 * Processing rounds/saturates to int16. Returns 0 on success, -1 without
 * processing invalid arguments. pcm may be NULL only when frames is zero. */
int gain_process(gain_t *ctx, int16_t *pcm, uint32_t frames,
                 uint32_t channels, uint32_t sample_rate);

/* Clear the context. NULL and repeated calls are safe. */
void gain_deinit(gain_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __GAIN_H__ */
