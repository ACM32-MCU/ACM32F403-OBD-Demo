/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-15     Aisinochip   First version
 */

#include <board.h>

#if defined(RT_USING_PWM)

#include "pwm_config.h"
#include <drivers/rt_drv_pwm.h>
#include "drv_pin_table.h"

#define MAX_PERIOD  65535
#define MIN_PERIOD  3
#define MIN_PULSE   2

enum
{
#if defined(BSP_USING_TIM1) && defined(BSP_USING_PWM1)
    PWM1_INDEX,
#endif

#if defined(BSP_USING_TIM2) && defined(BSP_USING_PWM2)
    PWM2_INDEX,
#endif

#if defined(BSP_USING_TIM3) && defined(BSP_USING_PWM3)
    PWM3_INDEX,
#endif

#if defined(BSP_USING_TIM4) && defined(BSP_USING_PWM4)
    PWM4_INDEX,
#endif

#if defined(BSP_USING_TIM14) && defined(BSP_USING_PWM14)
    PWM14_INDEX,
#endif

#if defined(BSP_USING_TIM15) && defined(BSP_USING_PWM15)
    PWM15_INDEX,
#endif

#if defined(BSP_USING_TIM16) && defined(BSP_USING_PWM16)
    PWM16_INDEX,
#endif

#if defined(BSP_USING_TIM17) && defined(BSP_USING_PWM17)
    PWM17_INDEX,
#endif
};

struct acm32_pwm
{
    struct rt_device_pwm    pwm_device;
    TIM_HandleTypeDef       tim_handle;
    rt_uint8_t              channel;
    char                    *name;
};

static struct acm32_pwm acm32_pwm_obj[] =
{
#ifdef BSP_USING_PWM1
    PWM1_CONFIG,
#endif

#ifdef BSP_USING_PWM2
    PWM2_CONFIG,
#endif

#ifdef BSP_USING_PWM3
    PWM3_CONFIG,
#endif

#ifdef BSP_USING_PWM4
    PWM4_CONFIG,
#endif

#ifdef BSP_USING_PWM5
    PWM5_CONFIG,
#endif

#ifdef BSP_USING_PWM6
    PWM6_CONFIG,
#endif

#ifdef BSP_USING_PWM7
    PWM7_CONFIG,
#endif

#ifdef BSP_USING_PWM8
    PWM8_CONFIG,
#endif

#ifdef BSP_USING_PWM9
    PWM9_CONFIG,
#endif

#ifdef BSP_USING_PWM10
    PWM10_CONFIG,
#endif

#ifdef BSP_USING_PWM11
    PWM11_CONFIG,
#endif

#ifdef BSP_USING_PWM12
    PWM12_CONFIG,
#endif

#ifdef BSP_USING_PWM13
    PWM13_CONFIG,
#endif

#ifdef BSP_USING_PWM14
    PWM14_CONFIG,
#endif

#ifdef BSP_USING_PWM15
    PWM15_CONFIG,
#endif

#ifdef BSP_USING_PWM16
    PWM16_CONFIG,
#endif

#ifdef BSP_USING_PWM17
    PWM17_CONFIG,
#endif
};

static rt_uint32_t get_timer_clock(void)
{
    rt_uint32_t timer_clock;

    /* get timer clock */
    timer_clock = System_Get_APBClock();
    if (System_Get_SystemClock() != System_Get_APBClock())
    {
        timer_clock =  System_Get_APBClock() << 1;
    }

    return timer_clock ;
}

static rt_err_t drv_pwm_enable(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = configuration->channel - 1;

    if (!configuration->complementary)
    {
        if (!enable)
        {
            HAL_TIM_PWM_Output_Stop(htim->Instance, channel);
        }
        else
        {
            HAL_TIM_PWM_Output_Start(htim->Instance, channel);
        }
    }
    else if (configuration->complementary)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t drv_pwm_get(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t tim_clock;

    tim_clock = get_timer_clock();

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    configuration->period = (htim->Instance->ARR + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
    switch(configuration->channel)
    {
    case 1:
        configuration->pulse = (htim->Instance->CCR1 + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
        break;
    case 2:
        configuration->pulse = (htim->Instance->CCR2 + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
        break;
    case 3:
        configuration->pulse = (htim->Instance->CCR3 + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
        break;
    case 4:
        configuration->pulse = (htim->Instance->CCR4 + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
        break;
    default:
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t drv_pwm_set(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    rt_uint32_t period, pulse;
    rt_uint32_t tim_clock, psc;

    tim_clock = get_timer_clock();

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    period = (unsigned long long)configuration->period * tim_clock / 1000ULL ;
    psc = period / MAX_PERIOD + 1;
    period = period / psc;
    htim->Instance->PSC = psc - 1;

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    htim->Instance->PSC = period - 1;

    pulse = (unsigned long long)configuration->pulse * tim_clock / psc / 1000ULL;
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }

    switch(configuration->channel)
    {
    case 1:
        htim->Instance->CCR1 = pulse - 1;
        break;
    case 2:
        htim->Instance->CCR2 = pulse - 1;
        break;
    case 3:
        htim->Instance->CCR3 = pulse - 1;
        break;
    case 4:
        htim->Instance->CCR4 = pulse - 1;
        break;
    default:
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)device->parent.user_data;

    switch (cmd)
    {
    case PWMN_CMD_ENABLE:
        configuration->complementary = RT_TRUE;
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(htim, configuration, RT_TRUE);
    case PWMN_CMD_DISABLE:
        configuration->complementary = RT_FALSE;
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(htim, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(htim, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(htim, configuration);
    default:
        return RT_EINVAL;
    }
}

static rt_err_t hw_pwm_io_init(char* pwm_name, rt_uint32_t channel)
{
#define AF_CONN(no, ch)    AF_TIM##no##_CH##ch

    pin_func_t  func = AF_INVALID;
    int pwm_no = atoi(&pwm_name[3]);

    switch(pwm_no)
    {
    case 1:
        switch(channel)
        {
        case 1:
            func = AF_CONN(1, 1);
            break;
        case 2:
            func = AF_CONN(1, 2);
            break;
        case 3:
            func = AF_CONN(1, 3);
            break;
        case 4:
            func = AF_CONN(1, 4);
            break;
        default:
            return -RT_ERROR;
        }
        break;
    case 2:
        switch(channel)
        {
        case 1:
            func = AF_TIM2_CH1_ETR;
            break;
        case 2:
            func = AF_CONN(2, 2);
            break;
        case 3:
            func = AF_CONN(2, 3);
            break;
        case 4:
            func = AF_CONN(2, 4);
            break;
        default:
            return -RT_ERROR;
        }
        break;
    case 3:
        switch(channel)
        {
        case 1:
            func = AF_CONN(3, 1);
            break;
        case 2:
            func = AF_CONN(3, 2);
            break;
        case 3:
            func = AF_CONN(3, 3);
            break;
        case 4:
            func = AF_CONN(3, 4);
            break;
        default:
            return -RT_ERROR;
        }
        break;
    case 4:
        switch(channel)
        {
        case 1:
            func = AF_CONN(4, 1);
            break;
        case 2:
            func = AF_CONN(4, 2);
            break;
        case 3:
            func = AF_CONN(4, 3);
            break;
        case 4:
            func = AF_CONN(4, 4);
            break;
        default:
            return -RT_ERROR;
        }
        break;
    case 14:
        if(channel != 1)
        {
            func = AF_TIM14_CH1;
        }
        else
        {
            return -RT_ERROR;
        }
        break;
    case 15:
        if(channel == 1)
        {
            func = AF_TIM15_CH1;
        }
        else if(channel == 2)
        {
            func = AF_TIM15_CH2;
        }
        else
        {
            return -RT_ERROR;
        }
        break;
    case 16:
        if(channel != 1)
        {
            func = AF_TIM16_CH1;
        }
        else
        {
            return -RT_ERROR;
        }
        break;
    case 17:
        if(channel != 1)
        {
            func = AF_TIM17_CH1;
        }
        else
        {
            return -RT_ERROR;
        }
        break;
    default:
        return -RT_ERROR;
    }

    if(!drv_pin_init_for_function(func, GPIO_MODE_AF_PP, GPIO_NOPULL))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t acm32_hw_pwm_init(struct acm32_pwm *device)
{
#define TIM_CLOCK_FREQ            (8000000U)
    TIM_OC_InitTypeDef Tim_OC_Init_Para;
    rt_err_t result = RT_EOK;
    TIM_HandleTypeDef *tim = RT_NULL;
    TIM_OC_InitTypeDef oc_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_ClockConfigTypeDef clock_config = {0};
    rt_uint32_t timer_clock;

    RT_ASSERT(device != RT_NULL);

    tim = (TIM_HandleTypeDef *)&device->tim_handle;

    /* get timer clock */
    timer_clock = get_timer_clock();

    /* configure the timer to pwm mode */
    tim->Init.ARRPreLoadEn = TIM_ARR_PRELOAD_ENABLE;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.RepetitionCounter = 0;
    tim->Init.Prescaler = (timer_clock/TIM_CLOCK_FREQ) - 1;
    if (timer_clock%TIM_CLOCK_FREQ > TIM_CLOCK_FREQ/2)
    {
        tim->Init.Prescaler = tim->Init.Prescaler + 1;
    }
    tim->Init.Period = (TIM_CLOCK_FREQ/1000000)*10 - 1;

    HAL_TIMER_MSP_Init(tim);
    HAL_TIMER_Base_Init(tim);

    Tim_OC_Init_Para.OCMode = OUTPUT_MODE_PWM1;
    Tim_OC_Init_Para.OCIdleState = OUTPUT_IDLE_STATE_0;
    Tim_OC_Init_Para.OCNIdleState = OUTPUT_IDLE_STATE_0;
    Tim_OC_Init_Para.OCPolarity = OUTPUT_POL_ACTIVE_HIGH;
    Tim_OC_Init_Para.OCNPolarity = OUTPUT_POL_ACTIVE_HIGH;
    Tim_OC_Init_Para.OCFastMode =  OUTPUT_FAST_MODE_DISABLE;
    Tim_OC_Init_Para.Pulse = (tim->Init.Period + 1)/2;

    /* config pwm channel */
    do
    {
        if (device->channel & 0x01)
        {
            if (HAL_TIMER_Output_Config(tim->Instance, &Tim_OC_Init_Para, TIM_CHANNEL_1) != HAL_OK)
            {
                result = -RT_ERROR;
                break;
            }
            result = hw_pwm_io_init(device->name, 1);
            if(result != RT_EOK)
            {
                break;
            }
        }

        if (device->channel & 0x02)
        {
            if (HAL_TIMER_Output_Config(tim->Instance, &Tim_OC_Init_Para, TIM_CHANNEL_2) != HAL_OK)
            {
                result = -RT_ERROR;
                break;
            }
            result = hw_pwm_io_init(device->name, 2);
            if(result != RT_EOK)
            {
                break;
            }
        }

        if (device->channel & 0x04)
        {
            if (HAL_TIMER_Output_Config(tim->Instance, &Tim_OC_Init_Para, TIM_CHANNEL_3) != HAL_OK)
            {
                result = -RT_ERROR;
                break;
            }
            result = hw_pwm_io_init(device->name, 3);
            if(result != RT_EOK)
            {
                break;
            }
        }

        if (device->channel & 0x08)
        {
            if (HAL_TIMER_Output_Config(tim->Instance, &Tim_OC_Init_Para, TIM_CHANNEL_4) != HAL_OK)
            {
                result = -RT_ERROR;
                break;
            }
            result = hw_pwm_io_init(device->name, 4);
            if(result != RT_EOK)
            {
                break;
            }
        }

    } while(0);

    return result;
}

static void pwm_get_channel(void)
{
#ifdef BSP_USING_PWM1_CH1
    acm32_pwm_obj[PWM1_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM1_CH2
    acm32_pwm_obj[PWM1_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM1_CH3
    acm32_pwm_obj[PWM1_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM1_CH4
    acm32_pwm_obj[PWM1_INDEX].channel |= 1 << 3;
#endif

#ifdef BSP_USING_PWM2_CH1
    acm32_pwm_obj[PWM2_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM2_CH2
    acm32_pwm_obj[PWM2_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM2_CH3
    acm32_pwm_obj[PWM2_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM2_CH4
    acm32_pwm_obj[PWM2_INDEX].channel |= 1 << 3;
#endif

#ifdef BSP_USING_PWM3_CH1
    acm32_pwm_obj[PWM3_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM3_CH2
    acm32_pwm_obj[PWM3_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM3_CH3
    acm32_pwm_obj[PWM3_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM3_CH4
    acm32_pwm_obj[PWM3_INDEX].channel |= 1 << 3;
#endif

#ifdef BSP_USING_PWM4_CH1
    acm32_pwm_obj[PWM4_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM4_CH2
    acm32_pwm_obj[PWM4_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM4_CH3
    acm32_pwm_obj[PWM4_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM4_CH4
    acm32_pwm_obj[PWM4_INDEX].channel |= 1 << 3;
#endif

#ifdef BSP_USING_PWM14_CH1
    acm32_pwm_obj[PWM14_INDEX].channel |= 1 << 0;
#endif

#ifdef BSP_USING_PWM15_CH1
    acm32_pwm_obj[PWM15_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM15_CH2
    acm32_pwm_obj[PWM15_INDEX].channel |= 1 << 1;
#endif

#ifdef BSP_USING_PWM16_CH1
    acm32_pwm_obj[PWM16_INDEX].channel |= 1 << 0;
#endif

#ifdef BSP_USING_PWM17_CH1
    acm32_pwm_obj[PWM17_INDEX].channel |= 1 << 0;
#endif
}

static struct rt_pwm_ops drv_ops =
{
    drv_pwm_control
};

static int acm32_pwm_init(void)
{
    int i = 0;
    int result = RT_EOK;

    pwm_get_channel();

    for (i = 0; i < sizeof(acm32_pwm_obj) / sizeof(acm32_pwm_obj[0]); i++)
    {
        /* pwm init */
        if (acm32_hw_pwm_init(&acm32_pwm_obj[i]) != RT_EOK)
        {
            result = -RT_ERROR;
            break;
        }
        else
        {
            /* register pwm device */
            if (rt_device_pwm_register(&acm32_pwm_obj[i].pwm_device, acm32_pwm_obj[i].name, &drv_ops, &acm32_pwm_obj[i].tim_handle) != RT_EOK)
            {
                result = -RT_ERROR;
                break;
            }
        }
    }

    return result;
}
INIT_DEVICE_EXPORT(acm32_pwm_init);
#endif /* RT_USING_PWM */
