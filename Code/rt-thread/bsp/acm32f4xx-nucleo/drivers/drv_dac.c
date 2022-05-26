/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-15     Aisinochip   First version
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#if defined(BSP_USING_DAC)

#define     DAC_NAME        "dac"

struct acm32_dac
{
    DAC_HandleTypeDef       handle;
    struct rt_dac_device    acm32_dac_device;
    rt_uint8_t              enable;
};

static struct acm32_dac acm32_dac_obj = {0};

static bool dac_channel_validate(rt_uint32_t channel)
{
    if(channel == DAC_CHANNEL_1 || channel == DAC_CHANNEL_2)
    {
        return true;
    }
    return false;
}

static rt_err_t acm32_dac_enabled(struct rt_dac_device *device, rt_uint32_t channel)
{
    struct acm32_dac *dacObj = RT_NULL;
    DAC_ChannelConfTypeDef sConfig;

    RT_ASSERT(device != RT_NULL);

    if(!dac_channel_validate(channel))
    {
        return -RT_ERROR;
    }

    dacObj = rt_container_of(device, struct acm32_dac, acm32_dac_device);

    if(dacObj->enable &(1<<channel))
    {
        return RT_EOK;
    }

    dacObj->handle.Instance = DAC;

    HAL_DAC_Init(&dacObj->handle);

    sConfig.DAC_Trigger=DAC_TRIGGER_SOFTWARE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

    dacObj->enable |= (1<<channel);

    HAL_DAC_ConfigChannel(&dacObj->handle, &sConfig, channel);

    HAL_DACEx_SelfCalibrate(&dacObj->handle, &sConfig, channel);

    return RT_EOK;
}

static rt_err_t acm32_dac_disabled(struct rt_dac_device *device, rt_uint32_t channel)
{
    struct acm32_dac *dacObj = RT_NULL;

    RT_ASSERT(device != RT_NULL);

    if(!dac_channel_validate(channel))
    {
        return -RT_ERROR;
    }

    dacObj = rt_container_of(device, struct acm32_dac, acm32_dac_device);

    dacObj->enable &= ~(1<<channel);

    HAL_DAC_Stop(&dacObj->handle, channel);

    return RT_EOK;
}

static rt_err_t acm32_dac_convert(struct rt_dac_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    struct acm32_dac *dacObj = RT_NULL;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);

    if(!dac_channel_validate(channel))
    {
        return -RT_ERROR;
    }

    *value = -1;
    dacObj = rt_container_of(device, struct acm32_dac, acm32_dac_device);

    /* channel disabled */
    if(0 == (dacObj->enable & (1<<channel) ))
    {
        return -RT_ERROR;
    }

    HAL_DAC_SetValue(&dacObj->handle, channel, DAC_ALIGN_12B_R, *value);
    
    HAL_DAC_Start(&dacObj->handle, channel);

    return RT_EOK;
}

static const struct rt_dac_ops acm_dac_ops =
{
    .enabled = acm32_dac_enabled,
    .disabled = acm32_dac_disabled,
    .convert = acm32_dac_convert,
};

static int acm32_dac_init(void)
{
    return rt_hw_dac_register(&acm32_dac_obj.acm32_dac_device,
                              DAC_NAME,
                              &acm_dac_ops,
                              RT_NULL);
}
INIT_BOARD_EXPORT(acm32_dac_init);

#endif /* BSP_USING_DAC */

