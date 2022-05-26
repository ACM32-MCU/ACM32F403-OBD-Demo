/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-06     AisinoChip   first version
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_HWTIMER
#include "tim_config.h"

enum
{
#ifdef BSP_USING_TIM1
    TIM1_INDEX,
#endif
#ifdef BSP_USING_TIM2
    TIM2_INDEX,
#endif
#ifdef BSP_USING_TIM3
    TIM3_INDEX,
#endif
#ifdef BSP_USING_TIM4
    TIM4_INDEX,
#endif
#ifdef BSP_USING_TIM6
    TIM6_INDEX,
#endif
#ifdef BSP_USING_TIM7
    TIM7_INDEX,
#endif
#ifdef BSP_USING_TIM14
    TIM14_INDEX,
#endif
#ifdef BSP_USING_TIM15
    TIM15_INDEX,
#endif
#ifdef BSP_USING_TIM16
    TIM16_INDEX,
#endif
#ifdef BSP_USING_TIM17
    TIM17_INDEX,
#endif
};

struct acm32_hwtimer
{
    rt_hwtimer_t            time_device;
    TIM_HandleTypeDef       tim_handle;
    IRQn_Type               tim_irqn;
    char                    *name;
};

static struct acm32_hwtimer acm32_hwtimer_obj[] =
{
#ifdef BSP_USING_TIM1
    TIM1_CONFIG,
#endif

#ifdef BSP_USING_TIM2
    TIM2_CONFIG,
#endif

#ifdef BSP_USING_TIM3
    TIM3_CONFIG,
#endif

#ifdef BSP_USING_TIM4
    TIM4_CONFIG,
#endif

#ifdef BSP_USING_TIM6
    TIM6_CONFIG,
#endif

#ifdef BSP_USING_TIM7
    TIM7_CONFIG,
#endif

#ifdef BSP_USING_TIM14
    TIM14_CONFIG,
#endif

#ifdef BSP_USING_TIM15
    TIM15_CONFIG,
#endif

#ifdef BSP_USING_TIM16
    TIM16_CONFIG,
#endif

#ifdef BSP_USING_TIM17
    TIM17_CONFIG,
#endif
};

static rt_uint32_t get_tim_clock(void)
{
    rt_uint32_t tim_clock;

    tim_clock = System_Get_APBClock();
    if (System_Get_SystemClock() != tim_clock)  /* if hclk/pclk != 1, then timer clk = pclk * 2 */
    {
        tim_clock <<= 1;
    }
    return tim_clock;
}

static void timer_init(struct rt_hwtimer_device *timer, rt_uint32_t state)
{
    rt_uint32_t timer_clock = 0;
    TIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);
    if (state)
    {
        tim = (TIM_HandleTypeDef *)timer->parent.user_data;

        /* calculate timer clock */
        timer_clock = get_tim_clock();
        /* calculate the ARR value */
        tim->Init.Period            = timer->reload*timer->cycles - 1;
        /* calculate the prescaler value used to divide the TIM clock */
        tim->Init.Prescaler         = (timer_clock / timer->freq) - 1 ;

        tim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

        /* set timer count mode */
        if (timer->info->cntmode == HWTIMER_CNTMODE_UP)
        {
            tim->Init.CounterMode   = TIM_COUNTERMODE_UP;
        }
        else
        {
            tim->Init.CounterMode   = TIM_COUNTERMODE_DOWN;
        }

        tim->Init.RepetitionCounter = 0;
        tim->Init.ARRPreLoadEn = TIM_ARR_PRELOAD_ENABLE;

        HAL_TIMER_MSP_Init(tim);
        HAL_TIMER_Base_Init(tim);
    }
}

static rt_err_t timer_start(rt_hwtimer_t *timer, rt_uint32_t t, rt_hwtimer_mode_t opmode)
{
    TIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (TIM_HandleTypeDef *)timer->parent.user_data;

    /* set tim cnt */
    tim->Instance->CNT = 0;
    /* set tim arr */
    tim->Instance->ARR = t - 1;

    if (opmode == HWTIMER_MODE_ONESHOT)
    {
        /* set timer to single mode */
        SET_BIT(tim->Instance->CR1, BIT3);
    }
    else
    {
        /* set timer to period mode */
        CLEAR_BIT(tim->Instance->CR1, BIT3);
    }

    /* enable IRQ */
    HAL_TIM_ENABLE_IT(tim, TIMER_INT_EN_UPD);

    /* start timer */
    HAL_TIMER_Base_Start(tim->Instance);

    return RT_EOK;
}

static void timer_stop(rt_hwtimer_t *timer)
{
    TIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (TIM_HandleTypeDef *)timer->parent.user_data;

    /* stop timer */
    HAL_TIMER_Base_Stop(tim->Instance);
}

static rt_err_t timer_ctrl(rt_hwtimer_t *timer, rt_uint32_t cmd, void *arg)
{
    TIM_HandleTypeDef *tim = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    tim = (TIM_HandleTypeDef *)timer->parent.user_data;

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
    {
        rt_uint32_t freq;
        rt_uint32_t timer_clock;
        rt_uint16_t val;

        /* set timer frequence */
        freq = *((rt_uint32_t *)arg);

        /* get timer clock */
        timer_clock = get_tim_clock();

        val = timer_clock / freq;
        tim->Instance->PSC = val - 1;

        /* update timer's frequency */
        timer->freq = freq;

        /* Update frequency value */
        tim->Instance->CR1 = BIT2;  /* CEN=0, URS=1, OPM = 0 */
        tim->Instance->EGR |= TIM_EVENTSOURCE_UPDATE;
    }
    break;
    default:
    {
        result = -RT_ENOSYS;
    }
    break;
    }

    return result;
}

static rt_uint32_t timer_counter_get(rt_hwtimer_t *timer)
{
    RT_ASSERT(timer != RT_NULL);

    return ((TIM_HandleTypeDef *)timer->parent.user_data)->Instance->CNT;
}

static const struct rt_hwtimer_info _info = TIM_DEV_INFO_CONFIG;

static const struct rt_hwtimer_ops _ops =
{
    .init = timer_init,
    .start = timer_start,
    .stop = timer_stop,
    .count_get = timer_counter_get,
    .control = timer_ctrl,
};

static void timer_isr(int idx)
{
    if (acm32_hwtimer_obj[idx].tim_handle.Instance->SR & TIMER_SR_UIF)
    {
        rt_device_hwtimer_isr(&acm32_hwtimer_obj[idx].time_device);
    }
    acm32_hwtimer_obj[idx].tim_handle.Instance->SR = 0;   /* write 0 to clear hardware flag */
}

#ifdef BSP_USING_TIM1
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM1_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM2
void TIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM2_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM3
void TIM3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM3_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM4
void TIM4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM4_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM6
void TIM6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM6_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM7
void TIM7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM7_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_TIM14
void TIM14_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM14_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef BSP_USING_TIM15
void TIM15_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM15_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef BSP_USING_TIM16
void TIM16_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM16_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef BSP_USING_TIM17
void TIM17_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* interrupt service routine */
    timer_isr(TIM17_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

static int acm32_hwtimer_init(void)
{
    int i = 0;
    int result = RT_EOK;

    for (i = 0; i < sizeof(acm32_hwtimer_obj) / sizeof(acm32_hwtimer_obj[0]); i++)
    {
        acm32_hwtimer_obj[i].time_device.info = &_info;
        acm32_hwtimer_obj[i].time_device.ops  = &_ops;
        result = rt_device_hwtimer_register(&acm32_hwtimer_obj[i].time_device,
                                            acm32_hwtimer_obj[i].name,
                                            &acm32_hwtimer_obj[i].tim_handle);
        if (result != RT_EOK)
        {
            result = -RT_ERROR;
            break;
        }
    }

    return result;
}
INIT_BOARD_EXPORT(acm32_hwtimer_init);

#endif /* RT_USING_HWTIMER */

