/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2024-07-11     oxlm          first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define LED_PIN    "PH.2" /* Green LED pins */

int main(int argc, char **argv)
{
    rt_kprintf("\nHello RT-Thread!\n");

    rt_pin_mode(rt_pin_get(LED_PIN), PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(rt_pin_get(LED_PIN), PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(rt_pin_get(LED_PIN), PIN_LOW);
        rt_thread_mdelay(500);
    }
}
