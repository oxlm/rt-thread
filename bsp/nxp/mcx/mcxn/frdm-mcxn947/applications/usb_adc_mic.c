/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-05     RT-Thread    three-channel ADC USB microphone
 */

#include <rtthread.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "drivers/dev_audio.h"
#include "usbd_audio.h"
#include "usbd_core.h"
#include "drv_mic.h"

#if defined(RT_CHERRYUSB_DEVICE_TEMPLATE_NONE) && defined(RT_CHERRYUSB_DEVICE_AUDIO)

#define USBD_VID           0x2200
#define USBD_PID           0x0009
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#ifdef CONFIG_USB_HS
/* HS isochronous: microframes = 2^(bInterval - 1), i.e. bInterval = log2(microframes) + 1.
 * 1 ms per packet is 8 microframes = 2^3, hence 0x04. */
#define EP_INTERVAL 0x04
#else
#define EP_INTERVAL 0x01
#endif

/* The descriptor must describe what the capture path actually produces, so the
 * stream format comes from drv_mic.h instead of being a second copy here. */
#define USB_ADC_MIC_SAMPLE_RATE          DRV_MIC_SAMPLE_RATE
#define USB_ADC_MIC_CHANNELS             DRV_MIC_CHANNELS
#define USB_ADC_MIC_BYTES_PER_SAMPLE     DRV_MIC_BYTES_PER_SAMPLE
#define USB_ADC_MIC_BITS_PER_SAMPLE      DRV_MIC_SAMPLE_BITS
#define USB_ADC_MIC_SAMPLES_PER_MS   (USB_ADC_MIC_SAMPLE_RATE / 1000U)
#define USB_ADC_MIC_PACKET_BYTES     (USB_ADC_MIC_SAMPLES_PER_MS * USB_ADC_MIC_CHANNELS * USB_ADC_MIC_BYTES_PER_SAMPLE)
#define USB_ADC_MIC_AUDIO_SLOT_BYTES (USB_ADC_MIC_CHANNELS * USB_ADC_MIC_BYTES_PER_SAMPLE)
#define USB_ADC_MIC_EP_MAX_PACKET_BYTES (USB_ADC_MIC_PACKET_BYTES + USB_ADC_MIC_AUDIO_SLOT_BYTES)
#define USB_ADC_MIC_TX_THREAD_STACK_SIZE 2048U
#define USB_ADC_MIC_TX_THREAD_PRIORITY   8U
#define USB_ADC_MIC_TX_THREAD_TICK       10U

#define AUDIO_IN_EP       0x81
#define AUDIO_IN_CLOCK_ID 0x01
#define AUDIO_IN_FU_ID    0x03

#define BMCONTROL       (AUDIO_V2_CONTROL_MUTE | AUDIO_V2_CONTROL_VOLUME)
#define INPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define INPUT_CH_ENABLE 0x00000007

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

static rt_device_t s_mic_dev;
static bool s_mic_dev_open;

static volatile bool s_streaming;
static volatile bool s_ep_tx_busy;
static volatile bool s_usb_tx_worker_inited;
static uint8_t s_usb_busid;
static volatile uint32_t s_mic_sample_rate = USB_ADC_MIC_SAMPLE_RATE;

static volatile bool s_usb_mute_state[USB_ADC_MIC_CHANNELS + 1U];
static volatile int s_usb_volume_db[USB_ADC_MIC_CHANNELS + 1U];

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t s_usb_tx_buffer[USB_ADC_MIC_PACKET_BYTES];
static struct rt_semaphore s_usb_tx_sem;
static struct rt_thread s_usb_tx_thread;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t s_usb_tx_thread_stack[USB_ADC_MIC_TX_THREAD_STACK_SIZE];

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

/* Standard record consumer: find -> open -> AUDIO_CTL_CONFIGURE -> read, the
 * sequence wavrecorder.c uses. rt_device_open() here is allocation free:
 * rt_audio_register() already ran _audio_dev_init() at device init level 3 and
 * set RT_DEVICE_FLAG_ACTIVATED, so rt_device_open() only flips flags, bumps the
 * ref_count and calls ops->start(). That matters because this runs from
 * USBD_IRQHandler via usbd_audio_open().
 *
 * The CONFIGURE result is ignored, as wavrecorder.c does: drv_mic.c rejects
 * anything but 16000 Hz / 3 ch / 16 bit and the descriptor advertises exactly
 * that, so the two cannot drift. */
static bool usb_adc_mic_mic_open(void)
{
    struct rt_audio_caps caps;

    if (s_mic_dev == RT_NULL)
    {
        s_mic_dev = rt_device_find(DRV_MIC_NAME);
        if (s_mic_dev == RT_NULL)
        {
            USB_LOG_ERR("ADC MIC audio device %s not found\r\n", DRV_MIC_NAME);
            return false;
        }
    }

    if (s_mic_dev_open)
    {
        return true;
    }

    if (rt_device_open(s_mic_dev, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        USB_LOG_ERR("ADC MIC audio device open failed\r\n");
        return false;
    }
    s_mic_dev_open = true;

    rt_memset(&caps, 0, sizeof(caps));
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = AUDIO_DSP_PARAM;
    caps.udata.config.samplerate = USB_ADC_MIC_SAMPLE_RATE;
    caps.udata.config.channels = USB_ADC_MIC_CHANNELS;
    caps.udata.config.samplebits = USB_ADC_MIC_BITS_PER_SAMPLE;
    (void)rt_device_control(s_mic_dev, AUDIO_CTL_CONFIGURE, &caps);

    return true;
}

static void usb_adc_mic_mic_close(void)
{
    if (!s_mic_dev_open)
    {
        return;
    }

    rt_device_close(s_mic_dev);
    s_mic_dev_open = false;
}

static bool usb_adc_mic_start_stream(void)
{
    if (!usb_adc_mic_mic_open())
    {
        return false;
    }

    rt_memset(s_usb_tx_buffer, 0, sizeof(s_usb_tx_buffer));
    s_ep_tx_busy = false;

    s_streaming = true;
    return true;
}

static void usb_adc_mic_stop_stream(void)
{
    s_streaming = false;
    usb_adc_mic_mic_close();
    s_ep_tx_busy = false;
}

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

    /* rt_device_read() blocks on the record pipe's RT_PIPE_FLAG_BLOCK_RD, which
     * is fine here: this runs on the TX thread, not in an interrupt. */
    rt_memset(s_usb_tx_buffer, 0, sizeof(s_usb_tx_buffer));
    if (s_mic_dev_open)
    {
        (void)rt_device_read(s_mic_dev, 0, s_usb_tx_buffer, sizeof(s_usb_tx_buffer));
    }

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
        USB_LOG_RAW("ADC MIC port speed: %u\r\n", (unsigned)usbd_get_port_speed(busid));
        usb_adc_mic_kick_tx();
    }
    else
    {
        USB_LOG_ERR("ADC MIC OPEN failed: stream start failed\r\n");
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


    (void)usb_adc_mic_tx_worker_init(busid);

    usbd_desc_register(busid, &audio_v2_descriptor);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, 0x0200, audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, 0x0200, audio_entity_table, 2));
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

#endif
