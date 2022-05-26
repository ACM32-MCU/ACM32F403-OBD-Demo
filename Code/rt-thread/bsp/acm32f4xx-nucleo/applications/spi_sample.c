/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     AisinoChip   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "board.h"

#ifdef RT_USING_SPI
extern rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, enum_GPIOx_t cs_gpiox, uint16_t cs_gpio_pin);
static int spi_sample(int argc, char *argv[])
{
    struct rt_spi_device *dev = RT_NULL;
    char bus_name[RT_NAME_MAX] = {0};
    char dev_name[RT_NAME_MAX] = {0};
    char data_send[128] = {0};
    char data_recv[128] = {0};
    int data_size = 0, recv_size;
    rt_err_t rslt;
    struct rt_spi_configuration cfg;


    if (argc == 4)
    {
        rt_strncpy(bus_name, argv[1], RT_NAME_MAX);
        rt_strncpy(dev_name, argv[2], RT_NAME_MAX);
        rt_strcpy(data_send, argv[3]);
        data_size = rt_strlen(data_send);
    }
    else
    {
        rt_kprintf("parameter invalid, spi_sample bus dev data");
        return -1;
    }

    if(RT_EOK != rt_hw_spi_device_attach(bus_name, dev_name, GPIOA, GPIO_PIN_4))
    {
        rt_kprintf("can't amount %s device on %s!\n", dev_name, bus_name);
    }

    dev = (struct rt_spi_device *)rt_device_find(dev_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", dev_name);
        return -1;
    }

    do
    {
        cfg.data_width = 8;
        cfg.max_hz = 1*1000*1000;
        cfg.mode = RT_SPI_MSB | RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_NO_CS;
        rslt = rt_spi_configure(dev, &cfg);
        if (RT_EOK != rslt)
        {
            rt_kprintf("rt_spi_configure error %x\n", rslt);
            break;
        }

        recv_size = rt_spi_send(dev, data_send, data_size);
        if(recv_size != data_size)
        {
            rt_kprintf("rt_spi_send error\n");
            rslt = -1;
        }

        recv_size = rt_spi_recv(dev, data_recv, data_size);
        if(recv_size != data_size || 0 != rt_memcmp(data_recv, data_send, data_size))
        {
            rt_kprintf("rt_spi_recv error\n");
            for(int i = 0; i< recv_size;i++)
            {
                rt_kprintf("%02X ", data_recv[i]);
            }
            rt_kprintf("\n");
            rslt = -1;
        }
#if 0
        recv_size = rt_spi_transfer(dev, data_send, data_recv, data_size);
        if(recv_size != data_size || 0 != rt_memcmp(data_recv, data_send, data_size))
        {
            rt_kprintf("rt_spi_transfer error\n");
            for(int i = 0; i< recv_size;i++)
            {
                rt_kprintf("%02X ", data_recv[i]);
            }
            rt_kprintf("\n");
            rslt = -1;
        }

        rslt = rt_spi_send_then_recv(dev, data_send, data_size, data_recv, data_size);
        if (RT_EOK != rslt)
        {
            rt_kprintf("rt_spi_send_then_recv error %x\n", rslt);
        }
        else
        {
            if(0 != rt_memcmp(data_recv, data_send, data_size))
            {
                rt_kprintf("rt_spi_send_then_recv data error\n");
                for(int i = 0; i< data_size;i++)
                {
                    rt_kprintf("%02X ", data_recv[i]);
                }
                rt_kprintf("\n");
                rslt = -1;
            }
        }

        rt_kprintf("success\n");
#endif

    } while(0);

    rt_spi_release_bus(dev);

    return rslt;
}
MSH_CMD_EXPORT(spi_sample, spi sample:spi_sample bus dev data);
#endif

