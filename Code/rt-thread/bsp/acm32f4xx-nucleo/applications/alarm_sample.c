/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-07     AisinoChip   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#ifdef RT_USING_ALARM

void user_alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    rt_kprintf("%04X alarm %d.\n", alarm->flag, rt_tick_get());
}

static int alarm_sample(int argc, char *argv[])
{
    struct rt_alarm_setup setup;
    struct rt_alarm * alarm = RT_NULL;
    static time_t now;
    struct tm p_tm;
    int mode = RT_ALARM_SECOND;
    int alarm_time = 1;

    if (argc == 3)
    {
        alarm_time = atoi(argv[1]);
        mode = atoi(argv[2]);
    }
    else if (argc == 2)
    {
        alarm_time = atoi(argv[1]);
    }

    /* 获取当前时间戳，并把下一秒时间设置为闹钟时间 */
    now = time(NULL) + alarm_time;
    gmtime_r(&now,&p_tm);

    setup.flag = mode;
    setup.wktime.tm_year = p_tm.tm_year;
    setup.wktime.tm_mon = p_tm.tm_mon;
    setup.wktime.tm_mday = p_tm.tm_mday;
    setup.wktime.tm_wday = p_tm.tm_wday;
    setup.wktime.tm_hour = p_tm.tm_hour;
    setup.wktime.tm_min = p_tm.tm_min;
    setup.wktime.tm_sec = p_tm.tm_sec;    

    alarm = rt_alarm_create(user_alarm_callback, &setup);
    if(RT_NULL != alarm)
    {
        rt_alarm_start(alarm);
    }
    return RT_EOK;
}
/* export msh cmd */
MSH_CMD_EXPORT(alarm_sample, alarm sample);
#endif
