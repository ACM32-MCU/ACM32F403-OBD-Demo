/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     AisinoChip   first version
 */

#ifndef __DRV_PIN_TABLE_H__
#define __DRV_PIN_TABLE_H__

#include <stdlib.h>

#define     PIN_NAME_MAX_SIZE   5

typedef enum {
    AF_INVALID = 0,
    AF_TIM1_CH1,
    AF_CAN1_RX,
    AF_I2C1_SCL,
    AF_SPI3_CS,
    AF_SPI3_MOSI,
    AF_SPI4_IO3,
    AF_TIM1_CH2,
    AF_SPI2_IO3,
    AF_TIM2_CH1_ETR,
    AF_SPI3_MISO,
    AF_SWDIO,
    AF_UART2_CTS,
    AF_LPUART1_TX,
    AF_SPI2_MISO,
    AF_TIM1_CH4,
    AF_UART2_TX,
    AF_UART3_TX,
    AF_TIM3_CH3,
    AF_SPI3_IO2,
    AF_TIM16_BKIN,
    AF_TIM4_CH3,
    AF_UART4_RX,
    AF_SPI3_IO3,
    AF_TIM3_CH1,
    AF_TIM15_BKIN,
    AF_TIM16_CH1,
    AF_LPUART1_RX,
    AF_SPI3_SCK,
    AF_TIM15_CH1N,
    AF_TIM17_BKIN,
    AF_SPI4_IO2,
    AF_I2S1_MCK,
    AF_UART1_RTS,
    AF_CAN2_TX,
    AF_SPI1_SCK,
    AF_SPI1_MISO,
    AF_TIM17_CH1,
    AF_COMP2_OUT,
    AF_IR_OUT,
    AF_SWCLK,
    AF_TIM4_ETR,
    AF_SPI1_MOSI,
    AF_TIM1_ETR,
    AF_TIM1_CH3,
    AF_CAN1_TX,
    AF_TIM4_CH4,
    AF_TIM3_ETR,
    AF_UART1_CTS,
    AF_TIM1_CH1N,
    AF_TIM1_CH2N,
    AF_I2S1_WS,
    AF_UART2_RTS,
    AF_SPI1_CS,
    AF_SPI1_IO3,
    AF_UART4_CTS,
    AF_I2C2_SCL,
    AF_I2C2_SDA,
    AF_RSTO,
    AF_TIM4_CH2,
    AF_UART4_TX,
    AF_TIM3_CH2,
    AF_TIM2_CH2,
    AF_UART3_CTS,
    AF_I2S1_CK,
    AF_MCO,
    AF_SPI1_IO2,
    AF_SPI4_SCK,
    AF_SPI2_SCK,
    AF_UART3_RTS,
    AF_TIM15_CH1,
    AF_TRST,
    AF_REMAP,
    AF_TIM2_CH3,
    AF_I2S1_SD,
    AF_TIM17_CH1N,
    AF_SPI4_CS,
    AF_GPIO,
    AF_TIM15_CH2,
    AF_SPI4_MOSI,
    AF_MCO2,
    AF_UART3_RX,
    AF_UART4_RTS,
    AF_CAN2_RX,
    AF_TIM3_CH4,
    AF_TIM1_BKIN,
    AF_SPI4_MISO,
    AF_TIM2_CH4,
    AF_COMP1_OUT,
    AF_SPI2_IO2,
    AF_TIM14_CH1,
    AF_BOOT,
    AF_UART2_RX,
    AF_UART1_RX,
    AF_SPI2_MOSI,
    AF_TDO,
    AF_UART1_TX,
    AF_TIM16_CH1N,
    AF_I2C1_SDA,
    AF_TIM1_CH3N,
    AF_TIM4_CH1,
    AF_TDI,
    AF_SPI2_CS,
} pin_func_t;

bool drv_pin_init(char* name, pin_func_t func, uint32_t mode, uint32_t pull);
bool drv_pin_init_for_function(pin_func_t func, uint32_t mode, uint32_t pull);

#endif

