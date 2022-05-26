/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-08     AisinoChip   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#ifdef RT_USING_I2C

static int i2c_sample(int argc, char *argv[])
{
    struct rt_i2c_bus_device *i2c_bus = RT_NULL;
    char name[RT_NAME_MAX] = {0};
    char data_send[128] = {0};  
    char data_recv[128] = {0};
    int data_size = 0;
    rt_uint16_t i2c_addr;
    struct rt_i2c_msg msgs;
    rt_uint16_t flags;


    if (argc == 4)
    {
        rt_strncpy(name, argv[1], RT_NAME_MAX);
        i2c_addr = atoi(argv[2]);
        rt_strcpy(data_send, argv[3]);
        data_size = rt_strlen(data_send);
    }
    else
    {
        rt_kprintf("parameter invalid, i2c_sample name addr data");
        return -1;
    }

    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(name);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", name);
        return -1;
    }

    flags = RT_I2C_WR;
    if(data_size != rt_i2c_master_send(i2c_bus, i2c_addr, flags, (const rt_uint8_t*)data_send, data_size))
    {
        rt_kprintf("can't find write data to device!\n");
        return -1;
    }

    flags = RT_I2C_RD;
    if(data_size != rt_i2c_master_recv(i2c_bus, i2c_addr, flags, (rt_uint8_t*)data_recv, data_size))
    {
        rt_kprintf("can't find write data to device!\n");
        return -1;
    }

    if(0 != rt_memcmp(data_recv, data_send, data_size))
    {
        rt_kprintf("i2c master recieve data error\n");
        for(int i = 0; i< data_size;i++)
        {
            rt_kprintf("%c ", data_recv[i]);
        }     
        rt_kprintf("\n");
        return -1;
    }
    rt_kprintf("success\n"); 

    return 0;
}
MSH_CMD_EXPORT(i2c_sample, i2c sample:i2c_sample name addr data);
#endif

