/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-28     oxlm         Initial version.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include "fsl_common.h"
#include "fsl_lptmr.h"

#ifdef RT_USING_RTC

struct rtc_device_object
{
    rt_rtc_dev_t  dev;
    time_t ts;
    rt_bool_t is_ts_set;
    time_t ts_new;
#ifdef BSP_USING_ALARM
    struct rt_rtc_wkalarm alarm;
#endif
};

static struct rtc_device_object rtc_dev;

void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    if(rtc_dev.is_ts_set)
    {
        rtc_dev.is_ts_set = RT_FALSE;
        rtc_dev.ts = rtc_dev.ts_new;
    }
    rtc_dev.ts++;

#ifdef BSP_USING_ALARM
    if(rtc_dev.alarm.enable)
    {
        rt_alarm_update(&rtc_dev.dev.parent, 0);
    }
#endif

    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}

static rt_err_t mcx_rtc_init(void)
{
    lptmr_config_t lptmrConfig;

    LPTMR_GetDefaultConfig(&lptmrConfig);
    LPTMR_Init(LPTMR0, &lptmrConfig);
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(1000000, 16000U));
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
    EnableIRQ(LPTMR0_IRQn);
    LPTMR_StartTimer(LPTMR0);

    return RT_EOK;
}

static rt_err_t mcx_rtc_get_time(time_t *ts)
{
    if(rtc_dev.is_ts_set)
    {
        *ts = rtc_dev.ts_new;
    }
    else
    {
        *ts = rtc_dev.ts;
    }

    return RT_EOK;
}

static rt_err_t mcx_rtc_set_time(time_t *ts)
{
    rtc_dev.ts_new = *ts;
    rtc_dev.is_ts_set = RT_TRUE;

    return RT_EOK;
}

#ifdef BSP_USING_ALARM
rt_err_t mcx_rtc_get_alarm(struct rt_rtc_wkalarm *alarm)
{
    rt_memcpy(alarm, &rtc_dev.alarm, sizeof(struct rt_rtc_wkalarm));
    return RT_EOK;
}

rt_err_t mcx_rtc_set_alarm(struct rt_rtc_wkalarm *alarm)
{
    rt_memcpy(&rtc_dev.alarm, alarm, sizeof(struct rt_rtc_wkalarm));
    return RT_EOK;
}
#endif

static const struct rt_rtc_ops ops =
{
    .init = mcx_rtc_init,
    .get_secs = mcx_rtc_get_time,
    .set_secs = mcx_rtc_set_time,
#ifdef BSP_USING_ALARM
    .get_alarm = mcx_rtc_get_alarm,
    .set_alarm = mcx_rtc_set_alarm,
#else
    .get_alarm = RT_NULL,
    .set_alarm = RT_NULL,
#endif
    .get_timeval = RT_NULL,
    .set_timeval = RT_NULL,
};

int rt_hw_rtc_init(void)
{
    rtc_dev.is_ts_set = RT_FALSE;
    rtc_dev.ts = 0;

    rtc_dev.dev.ops = &ops;
    if (rt_hw_rtc_register(&rtc_dev.dev, "rtc", RT_DEVICE_FLAG_RDWR, RT_NULL) != RT_EOK)
    {
        rt_kprintf("rtc init failed");
        return -RT_ERROR;
    }

    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_rtc_init);

#endif /*RT_USING_RTC */