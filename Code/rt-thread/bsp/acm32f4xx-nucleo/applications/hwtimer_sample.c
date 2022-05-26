/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-06     AisinoChip   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>

#ifdef RT_USING_HWTIMER

/* 默认的名称 */
#define     TIM_NAME   "hwtim1"
/* 默认周期 */
#define     TIM_FREQ    3000
/* 默认模式 */
#define     TIM_MODE    HWTIMER_MODE_PERIOD

static rt_err_t timer_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("%s isr, tick %d\n", dev->parent.name, rt_tick_get());

    return 0;
}

static void hwtimer(int argc, char** argv)
{
    rt_err_t err;
    rt_hwtimerval_t val;
    rt_device_t dev = RT_NULL;
    rt_tick_t tick;
    rt_hwtimer_mode_t mode = TIM_MODE;
    int freq = TIM_FREQ;
    int t = 5;
    char name[32] = {0};

    if (argc > 4)
    {
        rt_kprintf("Input invalid\n");
        return;
    }

    if(argc == 4)
    {
        rt_memcpy(name, argv[1], rt_strlen(argv[1]));
        freq = atoi(argv[2]);
        mode = atoi(argv[3]);
    }
    else if(argc == 3)
    {
        rt_memcpy(name, argv[1], rt_strlen(argv[1]));
        freq = atoi(argv[2]);
    }
    else if(argc == 2)
    {
        rt_memcpy(name, argv[1], rt_strlen(argv[1]));
    }
    else
    {
        rt_memcpy(name, TIM_NAME, rt_strlen(TIM_NAME));
    }

    if ((dev = rt_device_find(name)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", name);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", name);
        return;
    }

    do
    {
        /* 时间测量 */
        /* 计数时钟设置(默认1Mhz或支持的最小计数频率) */
        err = rt_device_control(dev, HWTIMER_CTRL_FREQ_SET, &freq);
        if (err != RT_EOK)
        {
            rt_kprintf("Set Freq=%dhz Fail\n", freq);
            break;
        }
        /* 周期模式 */
        err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);

        tick = rt_tick_get();
        rt_kprintf("Start Timer> Tick: %d\n", tick);
        /* 设置定时器超时值并启动定时器 */
        val.sec = t;
        val.usec = 0;
        rt_kprintf("SetTime: Sec %d, Usec %d\n", val.sec, val.usec);
        if (rt_device_write(dev, 0, &val, sizeof(val)) != sizeof(val))
        {
            rt_kprintf("SetTime Fail\n");
            break;
        }
        rt_kprintf("Sleep %d sec\n", t);
        rt_thread_delay(t*RT_TICK_PER_SECOND);

        /* 停止定时器 */
        err = rt_device_control(dev, HWTIMER_CTRL_STOP, RT_NULL);
        rt_kprintf("Timer Stoped\n");
        /* 读取计数 */
        rt_device_read(dev, 0, &val, sizeof(val));
        rt_kprintf("Read: Sec = %d, Usec = %d\n", val.sec, val.usec);

        /* 定时执行回调函数 -- 单次模式 */
        /* 设置超时回调函数 */
        rt_device_set_rx_indicate(dev, timer_timeout_cb);

        /* 单次模式 */
        err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);

        /* 设置定时器超时值并启动定时器 */
        val.sec = t;
        val.usec = 0;
        rt_kprintf("SetTime: Sec %d, Usec %d\n", val.sec, val.usec);
        if (rt_device_write(dev, 0, &val, sizeof(val)) != sizeof(val))
        {
            rt_kprintf("SetTime Fail\n");
            break;
        }

        /* 等待回调函数执行 */
        rt_thread_delay((t + 1)*RT_TICK_PER_SECOND*2);
    } while(0);

    err = rt_device_close(dev);
    rt_kprintf("Close %s\n", name);

    return;
}
#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(hwtimer, hardware timer sample:hwtimer name freq);
#endif
#endif /* RT_USING_HWTIMER */
