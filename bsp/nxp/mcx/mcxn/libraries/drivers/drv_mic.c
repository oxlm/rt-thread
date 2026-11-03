/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-22     RT-Thread    three-channel ADC microphone audio device
 * 2026-11-03     RT-Thread    use mic_algo for three-axis to mono processing
 *
 * ADC0 side A + ADC1 side A + ADC1 side B, hardware-triggered by CTIMER0 and
 * drained by three loop EDMA channels. mic_algo consumes synchronized raw XYZ
 * triplets and produces one signed mono sample per triplet for the record pipe.
 *
 * The stream format is fixed: 8000 Hz, 1 channel, 16 bits. Everything that
 * produces it (CTIMER period, LPADC trigger, DMA block size, packet length) is
 * a constant, so configure() reports that rather than applying a change.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#define DBG_TAG "adc_mic"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_ADC_MIC

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_ctimer.h"
#include "fsl_edma.h"
#include "fsl_inputmux.h"
#include "fsl_inputmux_connections.h"
#include "fsl_lpadc.h"
#include "fsl_opamp.h"
#include "fsl_port.h"

#include "drivers/dev_audio.h"
#include "drv_mic.h"
#include "mic_algo.h"

#if DRV_MIC_INPUT_CHANNELS != MIC_ALGO_AXES
#error ADC input count must match the algorithm axes
#endif

#define DRV_MIC_DMA_BLOCKS 8U

#define DRV_MIC_ADC_CMD_ID     1U
#define DRV_MIC_ADC_TRIGGER_ID 0U

#define DRV_MIC_DMA_CHANNEL_ADC0_A 0U
#define DRV_MIC_DMA_CHANNEL_ADC1_A 3U
#define DRV_MIC_DMA_CHANNEL_ADC1_B 4U

#ifndef DRV_MIC_USE_OPAMP
#define DRV_MIC_USE_OPAMP 0
#endif

#define DRV_MIC_CTIMER_MATCH_RATE DRV_MIC_SAMPLE_RATE

/* Initial ADXL354 settings only: 3.3 V ADC reference, 1.8 V sensor analog
 * supply, +/-2 g range, and DMA inputs ordered X,Y,Z. Replace zero_adc and
 * counts_per_g with six-face board measurements. The default mounting normal
 * is +Z (sensor PCB against the neck); verify the actual wiring and placement.
 * A mounting normal parallel to gravity deliberately fades to silence.
 * These values are not a measured calibration or a tuned neck-motion model. */
#define DRV_MIC_ADC_COUNTS_PER_V (65535.0f / 3.3f)

static const mic_algo_config_t s_mic_algo_config =
{
    .sample_rate = DRV_MIC_SAMPLE_RATE,
    .startup_ms = 200U,
    .zero_adc = {0.9f * DRV_MIC_ADC_COUNTS_PER_V,
                 0.9f * DRV_MIC_ADC_COUNTS_PER_V,
                 0.9f * DRV_MIC_ADC_COUNTS_PER_V},
    .counts_per_g = {0.4f * DRV_MIC_ADC_COUNTS_PER_V,
                     0.4f * DRV_MIC_ADC_COUNTS_PER_V,
                     0.4f * DRV_MIC_ADC_COUNTS_PER_V},
    .mount_direction = {0.0f, 0.0f, 1.0f},
    .gravity_cutoff_hz = 1.0f,
    .gravity_min_g = 0.7f,
    .gravity_max_g = 1.3f,
    .motion_rms_g = 0.1f,
    .min_projection = 0.15f,
    .highpass_hz = 5.0f,
    .lowpass_hz = 0.0f, /* Preserve the existing audio bandwidth until measured. */
    .fade_ms = 20.0f,
    .pcm_per_g = 0.4f * DRV_MIC_ADC_COUNTS_PER_V,
    .gain = 1.0f,
};

/* LPADC channel numbers for ADC0_A2 + ADC1_A0 + ADC1_B0. */
#ifndef DRV_MIC_ADC0_A_CHANNEL
#define DRV_MIC_ADC0_A_CHANNEL 2U
#endif

#ifndef DRV_MIC_ADC1_A_CHANNEL
#define DRV_MIC_ADC1_A_CHANNEL 0U
#endif

#ifndef DRV_MIC_ADC1_B_CHANNEL
#define DRV_MIC_ADC1_B_CHANNEL 0U
#endif

#ifndef DRV_MIC_OPAMP_GAIN
#define DRV_MIC_OPAMP_GAIN kOPAMP_PosGainNonInvert1X
#endif

#define DRV_MIC_HW_AVG        kLPADC_HardwareAverageCount16
#define DRV_MIC_SAMPLE_TIME   kLPADC_SampleTimeADCK35

struct drv_mic_dev
{
    struct rt_audio_device audio;
    struct rt_audio_configure record_config;
};

static struct drv_mic_dev s_mic_dev;

/* Only the eDMA callbacks and the start/stop ops touch this, and start() sets
 * it after the DMA loops are armed while stop() clears it before CTIMER0 is
 * stopped, so a callback never observes a tearing value. */
static volatile bool s_capturing;

/* Access is serialized with the DMA callback; no allocation on USB open/close. */
static mic_algo_t s_mic_algo;

AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc0_a_samples[DRV_MIC_DMA_BLOCKS][DRV_MIC_SAMPLES_PER_MS], 4U);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc1_a_samples[DRV_MIC_DMA_BLOCKS][DRV_MIC_SAMPLES_PER_MS], 4U);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc1_b_samples[DRV_MIC_DMA_BLOCKS][DRV_MIC_SAMPLES_PER_MS], 4U);

/* Not EDMA_ALLOCATE_TCD(): that macro spells the declaration itself as
 * "edma_tcd_t name[number]", so "static" cannot get in. */
AT_NONCACHEABLE_SECTION_ALIGN(static edma_tcd_t s_adc0_a_tcd[DRV_MIC_DMA_BLOCKS], EDMA_TCD_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static edma_tcd_t s_adc1_a_tcd[DRV_MIC_DMA_BLOCKS], EDMA_TCD_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static edma_tcd_t s_adc1_b_tcd[DRV_MIC_DMA_BLOCKS], EDMA_TCD_ALIGN_SIZE);

static edma_handle_t s_dma_handle[DRV_MIC_INPUT_CHANNELS];
static edma_transfer_config_t s_dma_transfer[DRV_MIC_INPUT_CHANNELS][DRV_MIC_DMA_BLOCKS];
static volatile uint32_t s_dma_done_count[DRV_MIC_INPUT_CHANNELS];
static volatile uint32_t s_mixed_count;

static const volatile uint32_t * const s_adc_sample_src[DRV_MIC_INPUT_CHANNELS] = {
    &s_adc0_a_samples[0][0],
    &s_adc1_a_samples[0][0],
    &s_adc1_b_samples[0][0],
};

static void drv_mic_prepare_dma_ring(edma_handle_t *handle,
                                     edma_transfer_config_t *transfer,
                                     edma_tcd_t *tcd,
                                     uint32_t channel,
                                     int32_t request,
                                     const volatile uint32_t *fifo,
                                     volatile uint32_t raw[DRV_MIC_DMA_BLOCKS][DRV_MIC_SAMPLES_PER_MS],
                                     void *callback_user_data);
static bool drv_mic_start_dma_ring(edma_handle_t *handle, edma_transfer_config_t *transfer);

static void drv_mic_mix_block(uint32_t block)
{
    uint16_t xyz[DRV_MIC_SAMPLES_PER_MS * DRV_MIC_INPUT_CHANNELS];
    int16_t mono[DRV_MIC_SAMPLES_PER_MS];
    uint32_t frame;
    uint32_t axis;

    for (frame = 0U; frame < DRV_MIC_SAMPLES_PER_MS; frame++)
    {
        for (axis = 0U; axis < DRV_MIC_INPUT_CHANNELS; axis++)
        {
            const volatile uint32_t *src = s_adc_sample_src[axis] +
                                           block * DRV_MIC_SAMPLES_PER_MS;

            xyz[frame * DRV_MIC_INPUT_CHANNELS + axis] =
                (uint16_t)ADC_RESFIFO_D(src[frame]);
        }
    }
    if (mic_algo_process(&s_mic_algo, xyz, DRV_MIC_SAMPLES_PER_MS, mono,
                         DRV_MIC_SAMPLES_PER_MS) != (int)DRV_MIC_SAMPLES_PER_MS)
    {
        memset(mono, 0, sizeof(mono));
    }
    /* Cortex-M33 PCM is little-endian. The audio record pipe copies this block
     * synchronously and drops the oldest block if its buffer is full. */
    rt_audio_rx_done(&s_mic_dev.audio, (rt_uint8_t *)mono, sizeof(mono));
}

static uint32_t drv_mic_min_done_count(void)
{
    uint32_t min_done;

    min_done = s_dma_done_count[0];
    if (s_dma_done_count[1] < min_done)
    {
        min_done = s_dma_done_count[1];
    }
    if (s_dma_done_count[2] < min_done)
    {
        min_done = s_dma_done_count[2];
    }

    return min_done;
}

static void drv_mic_dma_callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    uint32_t dma_index;
    uint32_t done_blocks;
    rt_base_t level;

    (void)handle;

    dma_index = (uint32_t)(uintptr_t)userData;
    if (dma_index >= DRV_MIC_INPUT_CHANNELS)
    {
        return;
    }
    /* USB open/close can run in an interrupt too. Prevent it from resetting
     * the algorithm while process() is using its context. */
    level = rt_hw_interrupt_disable();
    if (!s_capturing)
    {
        rt_hw_interrupt_enable(level);
        return;
    }

    done_blocks = tcds;
    if (done_blocks == 0U)
    {
        if (!transferDone)
        {
            rt_hw_interrupt_enable(level);
            return;
        }
        done_blocks = 1U;
    }

    s_dma_done_count[dma_index] += done_blocks;
    while (s_mixed_count < drv_mic_min_done_count())
    {
        uint32_t block;

        block = s_mixed_count % DRV_MIC_DMA_BLOCKS;
        drv_mic_mix_block(block);
        s_mixed_count++;
    }
    rt_hw_interrupt_enable(level);
}

/* drv_mic_start() runs with s_capturing false and no DMA loop armed, so the
 * counters can be zeroed under interrupt disable without a
 * real race. */
static void drv_mic_reset_state(void)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    s_dma_done_count[0] = 0U;
    s_dma_done_count[1] = 0U;
    s_dma_done_count[2] = 0U;
    s_mixed_count = 0U;
    rt_hw_interrupt_enable(level);
}

static void drv_mic_prepare_dma_ring(edma_handle_t *handle,
                                     edma_transfer_config_t *transfer,
                                     edma_tcd_t *tcd,
                                     uint32_t channel,
                                     int32_t request,
                                     const volatile uint32_t *fifo,
                                     volatile uint32_t raw[DRV_MIC_DMA_BLOCKS][DRV_MIC_SAMPLES_PER_MS],
                                     void *callback_user_data)
{
    EDMA_CreateHandle(handle, DMA0, channel);
    EDMA_InstallTCDMemory(handle, tcd, DRV_MIC_DMA_BLOCKS);
    EDMA_SetChannelMux(DMA0, channel, request);
    EDMA_SetCallback(handle, drv_mic_dma_callback, callback_user_data);

    for (uint32_t i = 0U; i < DRV_MIC_DMA_BLOCKS; i++)
    {
        EDMA_PrepareTransfer(&transfer[i],
                             (void *)fifo,
                             sizeof(uint32_t),
                             (void *)&raw[i][0],
                             sizeof(uint32_t),
                             sizeof(uint32_t),
                             sizeof(raw[i]),
                             kEDMA_PeripheralToMemory);
    }
}

static bool drv_mic_start_dma_ring(edma_handle_t *handle, edma_transfer_config_t *transfer)
{
    status_t status;

    EDMA_AbortTransfer(handle);
    status = EDMA_SubmitLoopTransfer(handle, transfer, DRV_MIC_DMA_BLOCKS);
    if (status != kStatus_Success)
    {
        LOG_E("adc mic EDMA submit failed: %d\n", (int)status);
        return false;
    }

    EDMA_StartTransfer(handle);

    return true;
}

static void drv_mic_init_opamp(OPAMP_Type *base)
{
    opamp_config_t config;

    OPAMP_GetDefaultConfig(&config);
    config.enable = true;
    config.mode = kOPAMP_LowNoiseMode;
    config.posGain = DRV_MIC_OPAMP_GAIN;
    config.negGain = kOPAMP_NegGainBufferMode;
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_OUTSW) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_OUTSW
    config.enableOutputSwitch = true;
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_ADCSW1) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_ADCSW1
    config.enablePosADCSw1 = true;
#else
    config.enablePosADCSw = true;
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_ADCSW2) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_ADCSW2
    config.enablePosADCSw2 = true;
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_BUFEN) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_BUFEN
    config.enableRefBuffer = ((uint32_t)DRV_MIC_OPAMP_GAIN <= 7U);
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_INPSEL) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_INPSEL
    config.PosInputChannelSelection = kOPAMP_PosInputChannel0;
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_TRIGMD) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_TRIGMD
    config.enableTriggerMode = false;
#endif

    OPAMP_Init(base, &config);
}

static void drv_mic_config_adc_pins(void)
{
    const port_pin_config_t adc0_a2_config = {
        kPORT_PullDisable,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAlt0,
        kPORT_InputBufferDisable,
        kPORT_InputNormal,
        kPORT_UnlockRegister,
    };

    CLOCK_EnableClock(kCLOCK_Port4);
    PORT_SetPinConfig(PORT4, 23U, &adc0_a2_config);
}

static void drv_mic_init_lpadc(ADC_Type *base)
{
    lpadc_config_t config;

    LPADC_GetDefaultConfig(&config);
    config.enableAnalogPreliminary = true;
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS
    config.conversionAverageMode = kLPADC_ConversionAverage1;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CFG_PWRSEL) && FSL_FEATURE_LPADC_HAS_CFG_PWRSEL
    config.powerLevelMode = kLPADC_PowerLevelAlt4;
#endif
    config.powerUpDelay = 0x10U;
    config.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3;
    config.enableInDozeMode = true;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    config.FIFO0Watermark = 0U;
    config.FIFO1Watermark = 0U;
#endif

    LPADC_Init(base, &config);

#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFSMODE) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFSMODE
    LPADC_DoOffsetCalibration(base, kLPADC_OffsetCalibration16bitMode);
#else
    LPADC_DoOffsetCalibration(base);
#endif
    LPADC_DoAutoCalibration(base);
}

static void drv_mic_config_adc0(void)
{
    lpadc_conv_command_config_t command;
    lpadc_conv_trigger_config_t trigger;

    LPADC_GetDefaultConvCommandConfig(&command);
    command.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    command.channelNumber = DRV_MIC_ADC0_A_CHANNEL;
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    command.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN) && FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN
    command.enableChannelB = false;
#endif
    command.hardwareAverageMode = DRV_MIC_HW_AVG;
    command.sampleTimeMode = DRV_MIC_SAMPLE_TIME;
    LPADC_SetConvCommandConfig(ADC0, DRV_MIC_ADC_CMD_ID, &command);

    memset(&trigger, 0, sizeof(trigger));
    trigger.targetCommandId = DRV_MIC_ADC_CMD_ID;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    trigger.channelAFIFOSelect = 0U;
#endif
    trigger.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC0, DRV_MIC_ADC_TRIGGER_ID, &trigger);

    LPADC_EnableFIFO0WatermarkDMA(ADC0, true);
    LPADC_EnableFIFO1WatermarkDMA(ADC0, false);
}

static void drv_mic_config_adc1(void)
{
    lpadc_conv_command_config_t command;
    lpadc_conv_trigger_config_t trigger;

    LPADC_GetDefaultConvCommandConfig(&command);
    command.sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    command.channelNumber = DRV_MIC_ADC1_A_CHANNEL;
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_ALTB_ADCH) && FSL_FEATURE_LPADC_HAS_CMDL_ALTB_ADCH
    command.channelBNumber = DRV_MIC_ADC1_B_CHANNEL;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    command.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN) && FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN
    command.enableChannelB = true;
#endif
    command.hardwareAverageMode = DRV_MIC_HW_AVG;
    command.sampleTimeMode = DRV_MIC_SAMPLE_TIME;
    LPADC_SetConvCommandConfig(ADC1, DRV_MIC_ADC_CMD_ID, &command);

    memset(&trigger, 0, sizeof(trigger));
    trigger.targetCommandId = DRV_MIC_ADC_CMD_ID;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    trigger.channelAFIFOSelect = 0U;
    trigger.channelBFIFOSelect = 1U;
#endif
    trigger.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC1, DRV_MIC_ADC_TRIGGER_ID, &trigger);

    LPADC_EnableFIFO0WatermarkDMA(ADC1, true);
    LPADC_EnableFIFO1WatermarkDMA(ADC1, true);
}

static void drv_mic_config_ctimer(void)
{
    ctimer_config_t timer_config;
    ctimer_match_config_t match_config;
    uint32_t timer_clk;
    uint32_t periodTicks;

    CTIMER_GetDefaultConfig(&timer_config);
    CTIMER_Init(CTIMER0, &timer_config);

    timer_clk = CLOCK_GetCTimerClkFreq(0U);
    if ((timer_clk == 0U) || ((timer_clk % (DRV_MIC_CTIMER_MATCH_RATE * 2U)) != 0U))
    {
        return;
    }

    /* M3 is configured for kCTIMER_Output_Toggle, so the output flips every
     * periodTicks; the LPADC hardware trigger only looks at rising edges, so the
     * effective trigger interval is 2 x periodTicks. Use half a sample period so
     * the rising edges land on 1 / sample rate. */
    periodTicks = timer_clk / (DRV_MIC_CTIMER_MATCH_RATE * 2U);
    if (periodTicks < 2U)
    {
        return;
    }

    match_config.matchValue = periodTicks - 1U;
    match_config.enableCounterReset = true;
    match_config.enableCounterStop = false;
    match_config.outControl = kCTIMER_Output_Toggle;
    match_config.outPinInitState = false;
    match_config.enableInterrupt = false;

    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_3, &match_config);
}

static rt_err_t drv_mic_init(struct rt_audio_device *audio)
{
    RT_ASSERT(audio != RT_NULL);

    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Dma0);
    CLOCK_AttachClk(kFRO12M_to_ADC0);
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO12M_to_ADC1);
    CLOCK_SetClkDiv(kCLOCK_DivAdc1Clk, 1U);
    CLOCK_AttachClk(kFRO12M_to_CTIMER0);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1U);

    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Ctimer0M3ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Ctimer0M3ToAdc1Trigger);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc0FifoARequestToDma0Ch21Ena, true);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc1FifoARequestToDma0Ch23Ena, true);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc1FifoBRequestoDma0Ch24Ena, true);

#if DRV_MIC_USE_OPAMP
    drv_mic_init_opamp(OPAMP0);
    drv_mic_init_opamp(OPAMP1);
    drv_mic_init_opamp(OPAMP2);
#endif

    drv_mic_config_adc_pins();
    drv_mic_init_lpadc(ADC0);
    drv_mic_init_lpadc(ADC1);
    drv_mic_config_adc0();
    drv_mic_config_adc1();
    drv_mic_config_ctimer();

    drv_mic_prepare_dma_ring(&s_dma_handle[0], &s_dma_transfer[0][0], s_adc0_a_tcd,
                             DRV_MIC_DMA_CHANNEL_ADC0_A, kDma0RequestMuxAdc0FifoARequest,
                             &ADC0->RESFIFO[0], s_adc0_a_samples, (void *)(uintptr_t)0U);
    drv_mic_prepare_dma_ring(&s_dma_handle[1], &s_dma_transfer[1][0], s_adc1_a_tcd,
                             DRV_MIC_DMA_CHANNEL_ADC1_A, kDma0RequestMuxAdc1FifoARequest,
                             &ADC1->RESFIFO[0], s_adc1_a_samples, (void *)(uintptr_t)1U);
    drv_mic_prepare_dma_ring(&s_dma_handle[2], &s_dma_transfer[2][0], s_adc1_b_tcd,
                             DRV_MIC_DMA_CHANNEL_ADC1_B, kDma0RequestMuxAdc1FifoBRequest,
                             &ADC1->RESFIFO[1], s_adc1_b_samples, (void *)(uintptr_t)2U);

    return RT_EOK;
}

static rt_err_t drv_mic_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(audio != RT_NULL);
    RT_ASSERT(caps != RT_NULL);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY:
        caps->udata.mask = AUDIO_TYPE_INPUT;
        break;

    case AUDIO_TYPE_INPUT:
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            caps->udata.config.samplerate = s_mic_dev.record_config.samplerate;
            caps->udata.config.channels = s_mic_dev.record_config.channels;
            caps->udata.config.samplebits = s_mic_dev.record_config.samplebits;
            break;

        case AUDIO_DSP_SAMPLERATE:
            caps->udata.config.samplerate = s_mic_dev.record_config.samplerate;
            break;

        case AUDIO_DSP_CHANNELS:
            caps->udata.config.channels = s_mic_dev.record_config.channels;
            break;

        case AUDIO_DSP_SAMPLEBITS:
            caps->udata.config.samplebits = s_mic_dev.record_config.samplebits;
            break;

        default:
            result = -RT_ERROR;
            break;
        }
        break;

    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_err_t drv_mic_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(audio != RT_NULL);
    RT_ASSERT(caps != RT_NULL);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            if ((caps->udata.config.samplerate != DRV_MIC_SAMPLE_RATE) ||
                (caps->udata.config.channels != DRV_MIC_CHANNELS) ||
                (caps->udata.config.samplebits != DRV_MIC_SAMPLE_BITS))
            {
                LOG_E("adc mic only supports %u Hz, %u ch, %u bit\n",
                      (unsigned)DRV_MIC_SAMPLE_RATE,
                      (unsigned)DRV_MIC_CHANNELS,
                      (unsigned)DRV_MIC_SAMPLE_BITS);
                result = -RT_ERROR;
            }
            break;

        case AUDIO_DSP_SAMPLERATE:
            if (caps->udata.config.samplerate != DRV_MIC_SAMPLE_RATE)
            {
                result = -RT_ERROR;
            }
            break;

        case AUDIO_DSP_CHANNELS:
            if (caps->udata.config.channels != DRV_MIC_CHANNELS)
            {
                result = -RT_ERROR;
            }
            break;

        case AUDIO_DSP_SAMPLEBITS:
            if (caps->udata.config.samplebits != DRV_MIC_SAMPLE_BITS)
            {
                result = -RT_ERROR;
            }
            break;

        default:
            result = -RT_ERROR;
            break;
        }
        break;

    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_err_t drv_mic_start(struct rt_audio_device *audio, int stream)
{
    RT_ASSERT(audio != RT_NULL);

    if (stream != AUDIO_STREAM_RECORD)
    {
        return -RT_ERROR;
    }

    s_capturing = false;
    CTIMER_StopTimer(CTIMER0);
    EDMA_AbortTransfer(&s_dma_handle[0]);
    EDMA_AbortTransfer(&s_dma_handle[1]);
    EDMA_AbortTransfer(&s_dma_handle[2]);
    drv_mic_reset_state();
    if (mic_algo_init(&s_mic_algo, &s_mic_algo_config) != 0)
    {
        LOG_E("invalid microphone algorithm configuration");
        return -RT_EINVAL;
    }

    LPADC_DoResetFIFO0(ADC0);
    LPADC_DoResetFIFO0(ADC1);
    LPADC_DoResetFIFO1(ADC1);

    if (!drv_mic_start_dma_ring(&s_dma_handle[0], &s_dma_transfer[0][0]))
    {
        mic_algo_deinit(&s_mic_algo);
        return -RT_ERROR;
    }
    if (!drv_mic_start_dma_ring(&s_dma_handle[1], &s_dma_transfer[1][0]))
    {
        EDMA_AbortTransfer(&s_dma_handle[0]);
        mic_algo_deinit(&s_mic_algo);
        return -RT_ERROR;
    }
    if (!drv_mic_start_dma_ring(&s_dma_handle[2], &s_dma_transfer[2][0]))
    {
        EDMA_AbortTransfer(&s_dma_handle[0]);
        EDMA_AbortTransfer(&s_dma_handle[1]);
        mic_algo_deinit(&s_mic_algo);
        return -RT_ERROR;
    }

    /* CTIMER0 is started last so no conversion is triggered until all three DMA
     * loops are armed and s_capturing allows the callback through. */
    s_capturing = true;

    CTIMER_Reset(CTIMER0);
    CTIMER_StartTimer(CTIMER0);

    LOG_D("start adc mic record");
    return RT_EOK;
}

static rt_err_t drv_mic_stop(struct rt_audio_device *audio, int stream)
{
    RT_ASSERT(audio != RT_NULL);

    if (stream != AUDIO_STREAM_RECORD)
    {
        return -RT_ERROR;
    }

    /* Clear the flag before stopping CTIMER0 so an in-flight callback returns
     * without touching the counters or the filter state. */
    s_capturing = false;
    CTIMER_StopTimer(CTIMER0);
    EDMA_AbortTransfer(&s_dma_handle[0]);
    EDMA_AbortTransfer(&s_dma_handle[1]);
    EDMA_AbortTransfer(&s_dma_handle[2]);
    LPADC_DoResetFIFO0(ADC0);
    LPADC_DoResetFIFO0(ADC1);
    LPADC_DoResetFIFO1(ADC1);

    mic_algo_deinit(&s_mic_algo);
    LOG_D("stop adc mic record");
    return RT_EOK;
}

static struct rt_audio_ops drv_mic_ops = {
    .getcaps = drv_mic_getcaps,
    .configure = drv_mic_configure,
    .init = drv_mic_init,
    .start = drv_mic_start,
    .stop = drv_mic_stop,
    /* Input-only device: rt_audio_register() gives a RT_DEVICE_FLAG_RDONLY device
     * a NULL replay struct, and _audio_dev_init() dereferences
     * audio->replay->buf_info whenever buffer_info is non-NULL. transmit is
     * unused on the record path. */
    .transmit = RT_NULL,
    .buffer_info = RT_NULL,
};

int rt_hw_mic_init(void)
{
    s_mic_dev.record_config.samplerate = DRV_MIC_SAMPLE_RATE;
    s_mic_dev.record_config.channels = DRV_MIC_CHANNELS;
    s_mic_dev.record_config.samplebits = DRV_MIC_SAMPLE_BITS;

    s_mic_dev.audio.ops = &drv_mic_ops;

    LOG_I("audio adc mic registered: %u Hz, %u ch, %u bit\n",
          (unsigned)DRV_MIC_SAMPLE_RATE,
          (unsigned)DRV_MIC_CHANNELS,
          (unsigned)DRV_MIC_SAMPLE_BITS);
    rt_audio_register(&s_mic_dev.audio, DRV_MIC_NAME, RT_DEVICE_FLAG_RDONLY, (void *)&s_mic_dev);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_mic_init);

#endif /* BSP_USING_ADC_MIC */
