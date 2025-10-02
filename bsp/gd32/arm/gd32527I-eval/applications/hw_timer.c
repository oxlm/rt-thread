#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define TIMER_NAME "timer"

int hw_timer_test(int argc, char **argv)
{
    rt_device_t d_timer;
    rt_hwtimer_mode_t new_mode = HWTIMER_MODE_PERIOD;
    rt_int32_t new_freq        = 1000;
    rt_hwtimerval_t timeout_s;

    if (argc != 2) {
        rt_kprintf("Usage: hw_timer_test timerx\n");
        return -1;
    } else if (rt_strncmp(argv[1], TIMER_NAME, rt_strlen(TIMER_NAME)) != 0) {
        rt_kprintf("Error: timer name should be %s[0 ~ 13]\n", TIMER_NAME);
        return 0;
    }

    d_timer = rt_device_find(argv[1]);
    if (d_timer == RT_NULL) {
        rt_kprintf("find %s failed!\n", TIMER_NAME);
        return -1;
    }

    if (rt_device_open(d_timer, RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
        rt_kprintf("open %s failed!\n", TIMER_NAME);
        return -1;
    }

    rt_device_control(d_timer, HWTIMER_CTRL_MODE_SET, &new_mode);

    for (new_freq = 1000; new_freq <= 1000000; new_freq *= 10) {
        if (rt_device_control(d_timer, HWTIMER_CTRL_FREQ_SET, &new_freq) == RT_EOK) {
            rt_kprintf("set freq %d Hz success\n", new_freq);
            timeout_s.sec  = 5; /* 秒 */
            timeout_s.usec = 0; /* 微秒 */
            rt_device_write(d_timer, 0, &timeout_s, sizeof(timeout_s));

            rt_thread_mdelay(2000);

            /* 读取定时器当前值 */
            rt_device_read(d_timer, 0, &timeout_s, sizeof(timeout_s));
            rt_kprintf("Read: Sec = %d, Usec = %d\n", timeout_s.sec, timeout_s.usec);
        }
    }

    rt_device_close(d_timer);
    return 0;
}

MSH_CMD_EXPORT_ALIAS(hw_timer_test, hw_timer_test, hardware timer callback);