/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author        Notes
 * 2022-04-07   AisinoChip    first version
 */

#include "board.h"
#include <sys/time.h>
#include <rtdevice.h>

#ifdef BSP_USING_RTC

static RTC_ConfigTypeDef RTC_Handle;

rt_inline rt_uint8_t dec2hex(rt_uint8_t dec)
{
    return ((dec / 10) << 4) + (dec % 10);
}

rt_inline rt_uint8_t hex2dec(rt_uint8_t hex)
{
    return ((hex / 16) * 10) + (hex % 16);
}

static time_t get_rtc_timestamp(void)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm tm_new = {0};

    HAL_RTC_GetTime(&RTC_TimeStruct);
    HAL_RTC_GetDate(&RTC_DateStruct);

    tm_new.tm_sec  = hex2dec(RTC_TimeStruct.u8_Seconds);
    tm_new.tm_min  = hex2dec(RTC_TimeStruct.u8_Minutes);
    tm_new.tm_hour = hex2dec(RTC_TimeStruct.u8_Hours);
    tm_new.tm_mday = hex2dec(RTC_DateStruct.u8_Date);
    tm_new.tm_mon  = hex2dec(RTC_DateStruct.u8_Month) - 1;
    tm_new.tm_year = hex2dec(RTC_DateStruct.u8_Year) + 100;

    return timegm(&tm_new);
}

static rt_err_t set_rtc_time_stamp(time_t time_stamp)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm *p_tm;

    p_tm = gmtime(&time_stamp);
    if (p_tm->tm_year < 100)
    {
        return -RT_ERROR;
    }

    RTC_TimeStruct.u8_Seconds = dec2hex(p_tm->tm_sec);
    RTC_TimeStruct.u8_Minutes = dec2hex(p_tm->tm_min);
    RTC_TimeStruct.u8_Hours   = dec2hex(p_tm->tm_hour);
    RTC_DateStruct.u8_Date    = dec2hex(p_tm->tm_mday);
    RTC_DateStruct.u8_Month   = dec2hex(p_tm->tm_mon + 1);
    RTC_DateStruct.u8_Year    = dec2hex(p_tm->tm_year - 100);
    RTC_DateStruct.u8_WeekDay = dec2hex(p_tm->tm_wday) + 1;

    HAL_RTC_SetTime(&RTC_TimeStruct);
    HAL_RTC_SetDate(&RTC_DateStruct);

    return RT_EOK;
}

static rt_err_t _rtc_init(void)
{
    RTC_Handle.u32_ClockSource       = RTC_CLOCK_XTL;
    RTC_Handle.u32_Compensation      = COMPENSATION_INCREASE;
    RTC_Handle.u32_CompensationValue = 0x05;

    HAL_RTC_Config(&RTC_Handle);

#ifdef RT_USING_ALARM
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
#endif

    return RT_EOK;
}

static rt_err_t _rtc_get_secs(void *args)
{
    *(rt_uint32_t *)args = get_rtc_timestamp();
    return RT_EOK;
}

static rt_err_t _rtc_set_secs(void *args)
{
    rt_err_t result = RT_EOK;
    if (set_rtc_time_stamp(*(rt_uint32_t *)args))
    {
        result = -RT_ERROR;
    }

    return result;
}

static rt_err_t _rtc_get_timeval(void *args)
{
    ((struct timeval *)args)->tv_sec = get_rtc_timestamp();

    return RT_EOK;
}

static rt_err_t _rtc_set_timeval(void *args)
{
    struct timeval *tv = (struct timeval *)args;

    return set_rtc_time_stamp(((struct timeval *)args)->tv_sec);
}

static rt_rtc_dev_t acm32_rtc_dev;

#ifdef RT_USING_ALARM
RTC_AlarmTypeDef gstr_Alarm;
static rt_err_t _set_alarm(void* args)
{
    struct rt_rtc_wkalarm *wkalarm = (struct rt_rtc_wkalarm *)args;
    RTC_DateTypeDef RTC_DateStruct = {0};
    RTC_TimeTypeDef RTC_TimeStruct = {0};

    HAL_RTC_GetDate(&RTC_DateStruct);
    HAL_RTC_GetTime(&RTC_TimeStruct);

    RT_ASSERT(args != RT_NULL);

    gstr_Alarm.u32_AlarmMode      = RTC_ALARM_DAY_MODE;
    gstr_Alarm.u32_AlarmInterrupt = RTC_ALARM_INT_ENABLE;
    gstr_Alarm.u32_DayMask        = RTC_ALARM_DAY_MASK_DISABLE;
    gstr_Alarm.u32_HourMask       = RTC_ALARM_HOUR_MASK_DISABLE;
    gstr_Alarm.u32_MinMask        = RTC_ALARM_MIN_MASK_DISABLE;

    gstr_Alarm.u32_AlarmDay       = RTC_DateStruct.u8_Date;
    gstr_Alarm.u32_Hours          = dec2hex(wkalarm->tm_hour);
    gstr_Alarm.u32_Minutes        = dec2hex(wkalarm->tm_min);
    gstr_Alarm.u32_Seconds        = dec2hex(wkalarm->tm_sec);

    HAL_RTC_AlarmConfig(&gstr_Alarm);

    if(wkalarm->enable == RT_TRUE)
    {
        HAL_RTC_AlarmEnable();
    }
    else
    {
        HAL_RTC_AlarmDisable();
    }

    return RT_EOK;
}

static rt_err_t _get_alarm(void* args)
{
    struct rt_rtc_wkalarm *wkalarm = (struct rt_rtc_wkalarm *)args;
    RT_ASSERT(args != RT_NULL);

    wkalarm->tm_hour = hex2dec(gstr_Alarm.u32_Hours);
    wkalarm->tm_min = hex2dec(gstr_Alarm.u32_Minutes);
    wkalarm->tm_sec = hex2dec(gstr_Alarm.u32_Seconds);
    wkalarm->enable = (RTC->CR & BIT4) == 0 ? RT_FALSE : RT_TRUE;

    return RT_EOK;
}

static void alarm_isr()
{
    if (RTC->SR & RTC_SR_ALM)
    {
        RTC->SR = RTC_SR_ALM;

        rt_alarm_update((rt_device_t)&acm32_rtc_dev, (rt_uint32_t)RT_NULL);
    }
}

void RTC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    alarm_isr();

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

static const struct rt_rtc_ops acm32_rtc_ops =
{
    _rtc_init,
    _rtc_get_secs,
    _rtc_set_secs,
#ifdef RT_USING_ALARM
    _get_alarm,
    _set_alarm,
#else
    RT_NULL,
    RT_NULL,
#endif
    _rtc_get_timeval,
    _rtc_set_timeval,
};


static int rt_hw_rtc_init(void)
{
    acm32_rtc_dev.ops = &acm32_rtc_ops;

    return rt_hw_rtc_register(&acm32_rtc_dev, "rtc", RT_DEVICE_FLAG_RDWR, RT_NULL);
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);
#endif /* BSP_USING_RTC */

