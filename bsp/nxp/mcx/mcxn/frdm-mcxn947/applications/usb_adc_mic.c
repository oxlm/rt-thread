/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-05     RT-Thread    three-channel ADC USB microphone
 */

#include <rthw.h>
#include <rtthread.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "fsl_clock.h"
#include "fsl_ctimer.h"
#include "fsl_edma.h"
#include "fsl_inputmux.h"
#include "fsl_inputmux_connections.h"
#include "fsl_lpadc.h"
#include "fsl_opamp.h"
#include "usbd_audio.h"
#include "usbd_core.h"

#if defined(RT_CHERRYUSB_DEVICE_TEMPLATE_NONE) && defined(RT_CHERRYUSB_DEVICE_AUDIO)

#define USBD_VID           0xffff
#define USBD_PID           0xffff
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#ifdef CONFIG_USB_HS
#define EP_INTERVAL 0x04
#else
#define EP_INTERVAL 0x01
#endif

#define USB_ADC_MIC_SAMPLE_RATE      16000U
#define USB_ADC_MIC_CHANNELS         3U
#define USB_ADC_MIC_BYTES_PER_SAMPLE 2U
#define USB_ADC_MIC_BITS_PER_SAMPLE  16U
#define USB_ADC_MIC_SAMPLES_PER_MS   (USB_ADC_MIC_SAMPLE_RATE / 1000U)
#define USB_ADC_MIC_PACKET_BYTES     (USB_ADC_MIC_SAMPLES_PER_MS * USB_ADC_MIC_CHANNELS * USB_ADC_MIC_BYTES_PER_SAMPLE)
#define USB_ADC_MIC_AUDIO_SLOT_BYTES (USB_ADC_MIC_CHANNELS * USB_ADC_MIC_BYTES_PER_SAMPLE)
#define USB_ADC_MIC_EP_MAX_PACKET_BYTES (USB_ADC_MIC_PACKET_BYTES + USB_ADC_MIC_AUDIO_SLOT_BYTES)
#define USB_ADC_MIC_CTIMER_MATCH_RATE (USB_ADC_MIC_SAMPLE_RATE * 2U)
#define USB_ADC_MIC_TX_THREAD_STACK_SIZE 2048U
#define USB_ADC_MIC_TX_THREAD_PRIORITY   8U
#define USB_ADC_MIC_TX_THREAD_TICK       10U

#ifndef USB_ADC_MIC_USE_TEST_TONE
#define USB_ADC_MIC_USE_TEST_TONE 1
#endif

#if USB_ADC_MIC_USE_TEST_TONE
#define USB_ADC_MIC_TEST_TONE_HZ         997U
#define USB_ADC_MIC_TEST_TONE_TABLE_BITS 5U
#define USB_ADC_MIC_TEST_TONE_TABLE_SIZE (1U << USB_ADC_MIC_TEST_TONE_TABLE_BITS)
#define USB_ADC_MIC_TEST_TONE_PHASE_SHIFT (32U - USB_ADC_MIC_TEST_TONE_TABLE_BITS)
#define USB_ADC_MIC_TEST_TONE_PHASE_STEP \
    ((uint32_t)(((uint64_t)USB_ADC_MIC_TEST_TONE_HZ << 32) / USB_ADC_MIC_SAMPLE_RATE))
#endif

#define USB_ADC_MIC_DMA_BLOCKS 8U
#define USB_ADC_MIC_PCM_BLOCKS 8U

#define AUDIO_IN_EP       0x81
#define AUDIO_IN_CLOCK_ID 0x01
#define AUDIO_IN_FU_ID    0x03

#define BMCONTROL       (AUDIO_V2_CONTROL_MUTE | AUDIO_V2_CONTROL_VOLUME)
#define INPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define INPUT_CH_ENABLE 0x00000007

#define USB_ADC_MIC_ADC_CMD_ID     1U
#define USB_ADC_MIC_ADC_TRIGGER_ID 0U

#define USB_ADC_MIC_DMA_ADC0_A_CH 0U
#define USB_ADC_MIC_DMA_ADC0_B_CH 1U
#define USB_ADC_MIC_DMA_ADC1_A_CH 2U

/* Confirm these internal OPAMP-to-LPADC channel numbers against the MCXN947 RM or Config Tools. */
#ifndef USB_ADC_MIC_ADC0_A_CHANNEL
#define USB_ADC_MIC_ADC0_A_CHANNEL 0U
#endif

#ifndef USB_ADC_MIC_ADC0_B_CHANNEL
#define USB_ADC_MIC_ADC0_B_CHANNEL 0U
#endif

#ifndef USB_ADC_MIC_ADC1_A_CHANNEL
#define USB_ADC_MIC_ADC1_A_CHANNEL 0U
#endif

#ifndef USB_ADC_MIC_OPAMP_GAIN
#define USB_ADC_MIC_OPAMP_GAIN kOPAMP_PosGainNonInvert1X
#endif

#define USB_CONFIG_SIZE (9 +                                                                  \
                         AUDIO_V2_AC_DESCRIPTOR_LEN +                                         \
                         AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                               \
                         AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                             \
                         AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(USB_ADC_MIC_CHANNELS) +          \
                         AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                            \
                         AUDIO_V2_AS_DESCRIPTOR_LEN)

#define AUDIO_AC_SIZ (AUDIO_V2_SIZEOF_AC_HEADER_DESC +                                      \
                      AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                                \
                      AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                              \
                      AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(USB_ADC_MIC_CHANNELS) +           \
                      AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

static const uint8_t device_descriptor[] =
{
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0001, 0x01)
};

static const uint8_t config_descriptor[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_V2_AC_DESCRIPTOR_INIT(0x00, 0x02, AUDIO_AC_SIZ, AUDIO_CATEGORY_MICROPHONE, 0x00, 0x00),
    AUDIO_V2_AC_CLOCK_SOURCE_DESCRIPTOR_INIT(AUDIO_IN_CLOCK_ID, 0x01, 0x01),
    AUDIO_V2_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x02, AUDIO_INTERM_MIC, AUDIO_IN_CLOCK_ID,
                                               USB_ADC_MIC_CHANNELS, INPUT_CH_ENABLE, 0x0000),
    AUDIO_V2_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_IN_FU_ID, 0x02, INPUT_CTRL),
    AUDIO_V2_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x04, AUDIO_TERMINAL_STREAMING, AUDIO_IN_FU_ID,
                                                AUDIO_IN_CLOCK_ID, 0x0000),
    AUDIO_V2_AS_DESCRIPTOR_INIT(0x01, 0x04, USB_ADC_MIC_CHANNELS, INPUT_CH_ENABLE,
                                USB_ADC_MIC_BYTES_PER_SAMPLE, USB_ADC_MIC_BITS_PER_SAMPLE,
                                AUDIO_IN_EP, 0x05, USB_ADC_MIC_EP_MAX_PACKET_BYTES, EP_INTERVAL)
};

static const uint8_t device_quality_descriptor[] =
{
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] =
{
    (const char[]){ 0x09, 0x04 },
    "RT-Thread",
    "FRDM-MCXN947 ADC MIC",
    "2026090504",
};

static const uint8_t mic_default_sampling_freq_table[] =
{
    AUDIO_SAMPLE_FREQ_NUM(1),
    AUDIO_SAMPLE_FREQ_4B(USB_ADC_MIC_SAMPLE_RATE),
    AUDIO_SAMPLE_FREQ_4B(USB_ADC_MIC_SAMPLE_RATE),
    AUDIO_SAMPLE_FREQ_4B(0x00),
};

static volatile bool s_hw_inited;
static volatile bool s_streaming;
static volatile bool s_ep_tx_busy;
static volatile bool s_usb_tx_worker_inited;
static uint8_t s_usb_busid;
static volatile uint32_t s_mic_sample_rate = USB_ADC_MIC_SAMPLE_RATE;

static uint8_t s_pcm_ring[USB_ADC_MIC_PCM_BLOCKS][USB_ADC_MIC_PACKET_BYTES];
static volatile uint8_t s_pcm_read;
static volatile uint8_t s_pcm_write;
static volatile uint8_t s_pcm_count;
static volatile uint32_t s_pcm_overruns;
#if !USB_ADC_MIC_USE_TEST_TONE
static volatile uint32_t s_usb_underruns;
#endif
static volatile bool s_usb_mute_state[USB_ADC_MIC_CHANNELS + 1U];
static volatile int s_usb_volume_db[USB_ADC_MIC_CHANNELS + 1U];
#if USB_ADC_MIC_USE_TEST_TONE
static uint32_t s_test_tone_phase;
#endif

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t s_usb_tx_buffer[USB_ADC_MIC_PACKET_BYTES];
static struct rt_semaphore s_usb_tx_sem;
static struct rt_thread s_usb_tx_thread;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t s_usb_tx_thread_stack[USB_ADC_MIC_TX_THREAD_STACK_SIZE];

AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc0_a_samples[USB_ADC_MIC_DMA_BLOCKS][USB_ADC_MIC_SAMPLES_PER_MS], 4U);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc0_b_samples[USB_ADC_MIC_DMA_BLOCKS][USB_ADC_MIC_SAMPLES_PER_MS], 4U);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_adc1_a_samples[USB_ADC_MIC_DMA_BLOCKS][USB_ADC_MIC_SAMPLES_PER_MS], 4U);

EDMA_ALLOCATE_TCD(s_adc0_a_tcd, USB_ADC_MIC_DMA_BLOCKS);
EDMA_ALLOCATE_TCD(s_adc0_b_tcd, USB_ADC_MIC_DMA_BLOCKS);
EDMA_ALLOCATE_TCD(s_adc1_a_tcd, USB_ADC_MIC_DMA_BLOCKS);

static edma_handle_t s_dma_handle[USB_ADC_MIC_CHANNELS];
static edma_transfer_config_t s_dma_transfer[USB_ADC_MIC_CHANNELS][USB_ADC_MIC_DMA_BLOCKS];
static volatile uint32_t s_dma_done_count[USB_ADC_MIC_CHANNELS];
static volatile uint32_t s_mixed_count;

static const volatile uint32_t *const s_adc_sample_src[USB_ADC_MIC_CHANNELS] =
{
    &s_adc0_a_samples[0][0],
    &s_adc0_b_samples[0][0],
    &s_adc1_a_samples[0][0],
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(string_descriptors[0])))
    {
        return RT_NULL;
    }

    return string_descriptors[index];
}

static const struct usb_descriptor audio_v2_descriptor =
{
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};

static void usb_adc_mic_try_send(uint8_t busid);
static void usb_adc_mic_kick_tx(void);
static void usb_adc_mic_stop_stream(void);

static void usb_adc_mic_tx_thread_entry(void *parameter)
{
    (void)parameter;

    while (1)
    {
        if (rt_sem_take(&s_usb_tx_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            usb_adc_mic_try_send(s_usb_busid);
        }
    }
}

static bool usb_adc_mic_tx_worker_init(uint8_t busid)
{
    rt_err_t err;

    s_usb_busid = busid;
    if (s_usb_tx_worker_inited)
    {
        return true;
    }

    err = rt_sem_init(&s_usb_tx_sem, "uacm_tx", 0, RT_IPC_FLAG_FIFO);
    if (err != RT_EOK)
    {
        USB_LOG_ERR("ADC MIC TX sem init failed: %d\r\n", (int)err);
        return false;
    }

    err = rt_thread_init(&s_usb_tx_thread,
                         "uacm_tx",
                         usb_adc_mic_tx_thread_entry,
                         RT_NULL,
                         s_usb_tx_thread_stack,
                         sizeof(s_usb_tx_thread_stack),
                         USB_ADC_MIC_TX_THREAD_PRIORITY,
                         USB_ADC_MIC_TX_THREAD_TICK);
    if (err != RT_EOK)
    {
        USB_LOG_ERR("ADC MIC TX thread init failed: %d\r\n", (int)err);
        return false;
    }

    err = rt_thread_startup(&s_usb_tx_thread);
    if (err != RT_EOK)
    {
        USB_LOG_ERR("ADC MIC TX thread startup failed: %d\r\n", (int)err);
        return false;
    }

    s_usb_tx_worker_inited = true;
    return true;
}

static void usb_adc_mic_kick_tx(void)
{
    if (s_usb_tx_worker_inited)
    {
        (void)rt_sem_release(&s_usb_tx_sem);
    }
}

static void usb_adc_mic_ring_reset(void)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    s_pcm_read = 0U;
    s_pcm_write = 0U;
    s_pcm_count = 0U;
    s_dma_done_count[0] = 0U;
    s_dma_done_count[1] = 0U;
    s_dma_done_count[2] = 0U;
    s_mixed_count = 0U;
    rt_hw_interrupt_enable(level);
}

static void usb_adc_mic_ring_push(const uint8_t *data)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    if (s_pcm_count >= USB_ADC_MIC_PCM_BLOCKS)
    {
        s_pcm_read = (uint8_t)((s_pcm_read + 1U) % USB_ADC_MIC_PCM_BLOCKS);
        s_pcm_count--;
        s_pcm_overruns++;
    }

    memcpy(s_pcm_ring[s_pcm_write], data, USB_ADC_MIC_PACKET_BYTES);
    s_pcm_write = (uint8_t)((s_pcm_write + 1U) % USB_ADC_MIC_PCM_BLOCKS);
    s_pcm_count++;
    rt_hw_interrupt_enable(level);
}

#if !USB_ADC_MIC_USE_TEST_TONE
static bool usb_adc_mic_ring_pop(uint8_t *data)
{
    bool has_data = false;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    if (s_pcm_count > 0U)
    {
        memcpy(data, s_pcm_ring[s_pcm_read], USB_ADC_MIC_PACKET_BYTES);
        s_pcm_read = (uint8_t)((s_pcm_read + 1U) % USB_ADC_MIC_PCM_BLOCKS);
        s_pcm_count--;
        has_data = true;
    }
    rt_hw_interrupt_enable(level);

    return has_data;
}
#endif

static int16_t usb_adc_mic_adc_to_pcm(uint32_t raw)
{
    uint16_t sample;

    sample = (uint16_t)(raw & ADC_RESFIFO_D_MASK);

    return (int16_t)((int32_t)sample - 32768);
}

static void usb_adc_mic_mix_block(uint32_t block)
{
    uint8_t pcm[USB_ADC_MIC_PACKET_BYTES];
    uint8_t *dst;

    dst = pcm;
    for (uint32_t i = 0U; i < USB_ADC_MIC_SAMPLES_PER_MS; i++)
    {
        for (uint32_t ch = 0U; ch < USB_ADC_MIC_CHANNELS; ch++)
        {
            const volatile uint32_t *src;
            int16_t sample;

            src = s_adc_sample_src[ch] + (block * USB_ADC_MIC_SAMPLES_PER_MS);
            sample = usb_adc_mic_adc_to_pcm(src[i]);
            *dst++ = (uint8_t)(sample & 0xff);
            *dst++ = (uint8_t)(((uint16_t)sample >> 8) & 0xff);
        }
    }

    usb_adc_mic_ring_push(pcm);
}

static uint32_t usb_adc_mic_min_done_count(void)
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

static void usb_adc_mic_dma_callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    uint32_t dma_index;
    uint32_t done_blocks;

    (void)handle;

    dma_index = (uint32_t)(uintptr_t)userData;
    if (dma_index >= USB_ADC_MIC_CHANNELS)
    {
        return;
    }
    if (!s_streaming)
    {
        return;
    }

    done_blocks = tcds;
    if (done_blocks == 0U)
    {
        if (!transferDone)
        {
            return;
        }
        done_blocks = 1U;
    }

    s_dma_done_count[dma_index] += done_blocks;
    while (s_mixed_count < usb_adc_mic_min_done_count())
    {
        uint32_t block;

        block = s_mixed_count % USB_ADC_MIC_DMA_BLOCKS;
        usb_adc_mic_mix_block(block);
        s_mixed_count++;
    }
}

static void usb_adc_mic_prepare_dma_ring(edma_handle_t *handle,
                                         edma_transfer_config_t *transfer,
                                         edma_tcd_t *tcd,
                                         uint32_t channel,
                                         int32_t request,
                                         const volatile uint32_t *fifo,
                                         volatile uint32_t raw[USB_ADC_MIC_DMA_BLOCKS][USB_ADC_MIC_SAMPLES_PER_MS],
                                         void *callback_user_data)
{
    EDMA_CreateHandle(handle, DMA0, channel);
    EDMA_InstallTCDMemory(handle, tcd, USB_ADC_MIC_DMA_BLOCKS);
    EDMA_SetChannelMux(DMA0, channel, request);
    EDMA_SetCallback(handle, usb_adc_mic_dma_callback, callback_user_data);

    for (uint32_t i = 0U; i < USB_ADC_MIC_DMA_BLOCKS; i++)
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

static bool usb_adc_mic_start_dma_ring(edma_handle_t *handle, edma_transfer_config_t *transfer)
{
    status_t status;

    EDMA_AbortTransfer(handle);
    status = EDMA_SubmitLoopTransfer(handle, transfer, USB_ADC_MIC_DMA_BLOCKS);
    if (status != kStatus_Success)
    {
        USB_LOG_ERR("ADC mic EDMA submit failed: %d\r\n", (int)status);
        return false;
    }

    EDMA_StartTransfer(handle);

    return true;
}

static void usb_adc_mic_init_opamp(OPAMP_Type *base)
{
    opamp_config_t config;

    OPAMP_GetDefaultConfig(&config);
    config.enable = true;
    config.mode = kOPAMP_LowNoiseMode;
    config.posGain = USB_ADC_MIC_OPAMP_GAIN;
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
    config.enableRefBuffer = ((uint32_t)USB_ADC_MIC_OPAMP_GAIN <= 7U);
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_INPSEL) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_INPSEL
    config.PosInputChannelSelection = kOPAMP_PosInputChannel0;
#endif
#if defined(FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_TRIGMD) && FSL_FEATURE_OPAMP_HAS_OPAMP_CTR_TRIGMD
    config.enableTriggerMode = false;
#endif

    OPAMP_Init(base, &config);
}

static void usb_adc_mic_init_lpadc(ADC_Type *base)
{
    lpadc_config_t config;

    LPADC_GetDefaultConfig(&config);
    config.enableAnalogPreliminary = true;
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS
    config.conversionAverageMode = kLPADC_ConversionAverage128;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CFG_PWRSEL) && FSL_FEATURE_LPADC_HAS_CFG_PWRSEL
    config.powerLevelMode = kLPADC_PowerLevelAlt4;
#endif
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

static void usb_adc_mic_config_adc0(void)
{
    lpadc_conv_command_config_t command;
    lpadc_conv_trigger_config_t trigger;

    LPADC_GetDefaultConvCommandConfig(&command);
    command.sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    command.channelNumber = USB_ADC_MIC_ADC0_A_CHANNEL;
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_ALTB_ADCH) && FSL_FEATURE_LPADC_HAS_CMDL_ALTB_ADCH
    command.channelBNumber = USB_ADC_MIC_ADC0_B_CHANNEL;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    command.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN) && FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN
    command.enableChannelB = true;
#endif
    command.hardwareAverageMode = kLPADC_HardwareAverageCount1;
    command.sampleTimeMode = kLPADC_SampleTimeADCK35;
    LPADC_SetConvCommandConfig(ADC0, USB_ADC_MIC_ADC_CMD_ID, &command);

    memset(&trigger, 0, sizeof(trigger));
    trigger.targetCommandId = USB_ADC_MIC_ADC_CMD_ID;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    trigger.channelAFIFOSelect = 0U;
    trigger.channelBFIFOSelect = 1U;
#endif
    trigger.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC0, USB_ADC_MIC_ADC_TRIGGER_ID, &trigger);

    LPADC_EnableFIFO0WatermarkDMA(ADC0, true);
    LPADC_EnableFIFO1WatermarkDMA(ADC0, true);
}

static void usb_adc_mic_config_adc1(void)
{
    lpadc_conv_command_config_t command;
    lpadc_conv_trigger_config_t trigger;

    LPADC_GetDefaultConvCommandConfig(&command);
    command.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    command.channelNumber = USB_ADC_MIC_ADC1_A_CHANNEL;
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    command.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif
    command.hardwareAverageMode = kLPADC_HardwareAverageCount1;
    command.sampleTimeMode = kLPADC_SampleTimeADCK35;
    LPADC_SetConvCommandConfig(ADC1, USB_ADC_MIC_ADC_CMD_ID, &command);

    memset(&trigger, 0, sizeof(trigger));
    trigger.targetCommandId = USB_ADC_MIC_ADC_CMD_ID;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    trigger.channelAFIFOSelect = 0U;
#endif
    trigger.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC1, USB_ADC_MIC_ADC_TRIGGER_ID, &trigger);

    LPADC_EnableFIFO0WatermarkDMA(ADC1, true);
}

static void usb_adc_mic_config_ctimer(void)
{
    ctimer_config_t timer_config;
    ctimer_match_config_t match_config;
    uint32_t timer_clk;
    uint32_t match_value;

    CTIMER_GetDefaultConfig(&timer_config);
    CTIMER_Init(CTIMER2, &timer_config);

    timer_clk = CLOCK_GetCTimerClkFreq(2U);
    /*
     * LPADC hardware trigger is edge-sensitive. MAT3 toggles every match, so
     * the rising-edge trigger rate is half of the CTIMER match rate.
     */
    match_value = (timer_clk / USB_ADC_MIC_CTIMER_MATCH_RATE) - 1U;

    memset(&match_config, 0, sizeof(match_config));
    match_config.enableCounterReset = true;
    match_config.enableCounterStop = false;
    match_config.matchValue = match_value;
    match_config.outControl = kCTIMER_Output_Toggle;
    match_config.outPinInitState = false;
    match_config.enableInterrupt = false;
    CTIMER_SetupMatch(CTIMER2, kCTIMER_Match_3, &match_config);
}

static void usb_adc_mic_hw_init(void)
{
    if (s_hw_inited)
    {
        return;
    }

    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Dma0);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC1);
    CLOCK_SetClkDiv(kCLOCK_DivAdc1Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER2);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer2Clk, 1U);

    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Ctimer2M3ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Ctimer2M3ToAdc1Trigger);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc0FifoARequestToDma0Ch21Ena, true);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc0FifoBRequestToDma0Ch22Ena, true);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_Adc1FifoARequestToDma0Ch23Ena, true);

    usb_adc_mic_init_opamp(OPAMP0);
    usb_adc_mic_init_opamp(OPAMP1);
    usb_adc_mic_init_opamp(OPAMP2);

    usb_adc_mic_init_lpadc(ADC0);
    usb_adc_mic_init_lpadc(ADC1);
    usb_adc_mic_config_adc0();
    usb_adc_mic_config_adc1();
    usb_adc_mic_config_ctimer();

    usb_adc_mic_prepare_dma_ring(&s_dma_handle[0], &s_dma_transfer[0][0], s_adc0_a_tcd,
                                 USB_ADC_MIC_DMA_ADC0_A_CH, kDma0RequestMuxAdc0FifoARequest,
                                 &ADC0->RESFIFO[0], s_adc0_a_samples, (void *)(uintptr_t)0U);
    usb_adc_mic_prepare_dma_ring(&s_dma_handle[1], &s_dma_transfer[1][0], s_adc0_b_tcd,
                                 USB_ADC_MIC_DMA_ADC0_B_CH, kDma0RequestMuxAdc0FifoBRequest,
                                 &ADC0->RESFIFO[1], s_adc0_b_samples, (void *)(uintptr_t)1U);
    usb_adc_mic_prepare_dma_ring(&s_dma_handle[2], &s_dma_transfer[2][0], s_adc1_a_tcd,
                                 USB_ADC_MIC_DMA_ADC1_A_CH, kDma0RequestMuxAdc1FifoARequest,
                                 &ADC1->RESFIFO[0], s_adc1_a_samples, (void *)(uintptr_t)2U);

    s_hw_inited = true;
}

static bool usb_adc_mic_start_stream(void)
{
    if (!s_hw_inited)
    {
        usb_adc_mic_hw_init();
    }

    usb_adc_mic_stop_stream();
    usb_adc_mic_ring_reset();
    memset(s_usb_tx_buffer, 0, sizeof(s_usb_tx_buffer));
#if USB_ADC_MIC_USE_TEST_TONE
    s_test_tone_phase = 0U;
#endif

    LPADC_DoResetFIFO0(ADC0);
    LPADC_DoResetFIFO1(ADC0);
    LPADC_DoResetFIFO0(ADC1);

    if (!usb_adc_mic_start_dma_ring(&s_dma_handle[0], &s_dma_transfer[0][0]))
    {
        return false;
    }
    if (!usb_adc_mic_start_dma_ring(&s_dma_handle[1], &s_dma_transfer[1][0]))
    {
        EDMA_AbortTransfer(&s_dma_handle[0]);
        return false;
    }
    if (!usb_adc_mic_start_dma_ring(&s_dma_handle[2], &s_dma_transfer[2][0]))
    {
        EDMA_AbortTransfer(&s_dma_handle[0]);
        EDMA_AbortTransfer(&s_dma_handle[1]);
        return false;
    }

    s_streaming = true;
    s_ep_tx_busy = false;
    CTIMER_Reset(CTIMER2);
    CTIMER_StartTimer(CTIMER2);

    return true;
}

static void usb_adc_mic_stop_stream(void)
{
    if (!s_hw_inited)
    {
        return;
    }

    s_streaming = false;
    CTIMER_StopTimer(CTIMER2);
    EDMA_AbortTransfer(&s_dma_handle[0]);
    EDMA_AbortTransfer(&s_dma_handle[1]);
    EDMA_AbortTransfer(&s_dma_handle[2]);
    LPADC_DoResetFIFO0(ADC0);
    LPADC_DoResetFIFO1(ADC0);
    LPADC_DoResetFIFO0(ADC1);

    s_ep_tx_busy = false;
}

#if USB_ADC_MIC_USE_TEST_TONE
static const int16_t sine_32pt[USB_ADC_MIC_TEST_TONE_TABLE_SIZE] =
{
    0, 3902, 7654, 11111,
    14142, 16629, 18478, 19616,
    20000, 19616, 18478, 16629,
    14142, 11111, 7654, 3902,
    0, -3902, -7654, -11111,
    -14142, -16629, -18478, -19616,
    -20000, -19616, -18478, -16629,
    -14142, -11111, -7654, -3902
};

static int16_t usb_adc_mic_next_test_sample(void)
{
    uint32_t index;
    int16_t sample;

    index = s_test_tone_phase >> USB_ADC_MIC_TEST_TONE_PHASE_SHIFT;
    sample = sine_32pt[index & (USB_ADC_MIC_TEST_TONE_TABLE_SIZE - 1U)];
    s_test_tone_phase += USB_ADC_MIC_TEST_TONE_PHASE_STEP;

    return sample;
}

static void usb_adc_mic_fill_test_tone(uint8_t *buffer)
{
    uint8_t *p = buffer;

    for (uint32_t i = 0; i < USB_ADC_MIC_SAMPLES_PER_MS; i++)
    {
        int16_t sample = usb_adc_mic_next_test_sample();

        for (uint32_t ch = 0; ch < USB_ADC_MIC_CHANNELS; ch++)
        {
            *p++ = (uint8_t)(sample & 0xff);
            *p++ = (uint8_t)(((uint16_t)sample >> 8) & 0xff);
        }
    }
}
#endif

static void usb_adc_mic_try_send(uint8_t busid)
{
    int ret;

    if (!s_streaming)
    {
        return;
    }
    if (s_ep_tx_busy)
    {
        return;
    }

#if USB_ADC_MIC_USE_TEST_TONE
    usb_adc_mic_fill_test_tone(s_usb_tx_buffer);
#else
    if (!usb_adc_mic_ring_pop(s_usb_tx_buffer))
    {
        memset(s_usb_tx_buffer, 0, sizeof(s_usb_tx_buffer));
        s_usb_underruns++;
    }
#endif

    s_ep_tx_busy = true;
    ret = usbd_ep_start_write(busid, AUDIO_IN_EP, s_usb_tx_buffer, sizeof(s_usb_tx_buffer));
    if (ret < 0)
    {
        s_ep_tx_busy = false;
    }
}

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    switch (event)
    {
    case USBD_EVENT_RESET:
    case USBD_EVENT_DISCONNECTED:
    case USBD_EVENT_SUSPEND:
        usb_adc_mic_stop_stream();
        break;
    default:
        break;
    }
}

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    if (intf != 1U)
    {
        return;
    }

    if (usb_adc_mic_start_stream())
    {
        USB_LOG_RAW("ADC MIC OPEN\r\n");
        usb_adc_mic_kick_tx();
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;

    if (intf != 1U)
    {
        return;
    }

    USB_LOG_RAW("ADC MIC CLOSE\r\n");
    usb_adc_mic_stop_stream();
}

void usbd_audio_set_sampling_freq(uint8_t busid, uint8_t ep, uint32_t sampling_freq)
{
    (void)busid;

    if (ep == AUDIO_IN_EP)
    {
        if (sampling_freq != USB_ADC_MIC_SAMPLE_RATE)
        {
            USB_LOG_ERR("ADC MIC only supports %u Hz\r\n", USB_ADC_MIC_SAMPLE_RATE);
        }
        s_mic_sample_rate = USB_ADC_MIC_SAMPLE_RATE;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t busid, uint8_t ep)
{
    (void)busid;

    if (ep == AUDIO_IN_EP)
    {
        return s_mic_sample_rate;
    }

    return 0U;
}

void usbd_audio_get_sampling_freq_table(uint8_t busid, uint8_t ep, uint8_t **sampling_freq_table)
{
    (void)busid;

    if (ep == AUDIO_IN_EP)
    {
        *sampling_freq_table = (uint8_t *)mic_default_sampling_freq_table;
    }
}

void usbd_audio_set_volume(uint8_t busid, uint8_t ep, uint8_t ch, int volume_db)
{
    (void)busid;

    if ((ep == AUDIO_IN_EP) && (ch <= USB_ADC_MIC_CHANNELS))
    {
        s_usb_volume_db[ch] = volume_db;
    }
}

int usbd_audio_get_volume(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;

    if ((ep == AUDIO_IN_EP) && (ch <= USB_ADC_MIC_CHANNELS))
    {
        return s_usb_volume_db[ch];
    }

    return 0;
}

void usbd_audio_set_mute(uint8_t busid, uint8_t ep, uint8_t ch, bool mute)
{
    (void)busid;

    if ((ep == AUDIO_IN_EP) && (ch <= USB_ADC_MIC_CHANNELS))
    {
        s_usb_mute_state[ch] = mute;
    }
}

bool usbd_audio_get_mute(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;

    if ((ep == AUDIO_IN_EP) && (ch <= USB_ADC_MIC_CHANNELS))
    {
        return s_usb_mute_state[ch];
    }

    return false;
}

void usbd_audio_iso_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)nbytes;

    if (ep != AUDIO_IN_EP)
    {
        return;
    }

    s_ep_tx_busy = false;
    usb_adc_mic_kick_tx();
}

static struct usbd_endpoint audio_in_ep =
{
    .ep_cb = usbd_audio_iso_in_callback,
    .ep_addr = AUDIO_IN_EP
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

static struct audio_entity_info audio_entity_table[] =
{
    {
        .bEntityId = AUDIO_IN_CLOCK_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_CLOCK_SOURCE,
        .ep = AUDIO_IN_EP
    },
    {
        .bEntityId = AUDIO_IN_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_IN_EP
    },
};

void usb_adc_mic_init(uint8_t busid, uintptr_t reg_base)
{
    s_mic_sample_rate = USB_ADC_MIC_SAMPLE_RATE;

    usb_adc_mic_hw_init();
    (void)usb_adc_mic_tx_worker_init(busid);

    usbd_desc_register(busid, &audio_v2_descriptor);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, 0x0200, audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, 0x0200, audio_entity_table, 2));
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

#endif
