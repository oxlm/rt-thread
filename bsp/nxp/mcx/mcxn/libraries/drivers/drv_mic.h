/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-22     RT-Thread    three-channel ADC microphone audio device
 */

#ifndef __DRV_MIC_H__
#define __DRV_MIC_H__

#define DRV_MIC_NAME             "mic0"
#define DRV_MIC_SAMPLE_RATE      16000U
#define DRV_MIC_CHANNELS         3U
#define DRV_MIC_SAMPLE_BITS      16U
#define DRV_MIC_BYTES_PER_SAMPLE 2U
#define DRV_MIC_SAMPLES_PER_MS   (DRV_MIC_SAMPLE_RATE / 1000U)
#define DRV_MIC_PACKET_BYTES     (DRV_MIC_SAMPLES_PER_MS * DRV_MIC_CHANNELS * DRV_MIC_BYTES_PER_SAMPLE)

int rt_hw_mic_init(void);

#endif /* __DRV_MIC_H__ */
