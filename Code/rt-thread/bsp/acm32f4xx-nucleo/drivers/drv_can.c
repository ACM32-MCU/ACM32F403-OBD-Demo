/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-05     AisinoChip   first implementation.
 */

#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

#if defined(RT_USING_CAN)
#if defined(BSP_USING_CAN1) || defined(BSP_USING_CAN2)

enum
{
#ifdef BSP_USING_CAN1
    CAN1_INDEX,
#endif
#ifdef BSP_USING_CAN2
    CAN2_INDEX,
#endif
};

/* acm32 can config */
struct acm32_can_config
{
    CAN_TypeDef         *instance;
    char                *name;

    enum_Enable_ID_t    enable_id;

    IRQn_Type           irq_type;
    rt_uint32_t         irq_priority;

    enum_GPIOx_t        tx_port;
    rt_uint32_t         tx_pin;
    rt_uint32_t         tx_alternate;

    enum_GPIOx_t        rx_port;
    rt_uint32_t         rx_pin;
    rt_uint32_t         rx_alternate;
};

/* acm32 can device */
struct acm32_can_device
{
    struct acm32_can_config config;
    CAN_HandleTypeDef       handle;
    CAN_FilterInitTypeDef   filter;
    struct rt_can_device    device;     /* inherit from can device */
};

static struct acm32_can_config can_config[] =
{
#ifdef BSP_USING_CAN1
    {
        .instance = CAN1,
        .name = "can1",
        .enable_id = CAN1_MODULE_CLOCK,
        .irq_type = CAN1_IRQ_NUMBER,
        .irq_priority = CAN1_IRQ_PRIORITY,
        .tx_port = CAN1_TX_PORT,
        .tx_pin = CAN1_TX_PIN,
        .tx_alternate = CAN1_TX_ALTERNATE,
        .rx_port = CAN1_RX_PORT,
        .rx_pin = CAN1_RX_PIN,
        .rx_alternate = CAN1_RX_ALTERNATE,
    },
#endif
#ifdef BSP_USING_CAN2
    {
        .instance = CAN2,
        .name = "can2",
        .enable_id = CAN2_MODULE_CLOCK,
        .irq_type = CAN2_IRQ_NUMBER,
        .irq_priority = CAN2_IRQ_PRIORITY,
        .tx_port = CAN2_TX_PORT,
        .tx_pin = CAN2_TX_PIN,
        .tx_alternate = CAN2_TX_ALTERNATE,
        .rx_port = CAN2_RX_PORT,
        .rx_pin = CAN2_RX_PIN,
        .rx_alternate = CAN2_RX_ALTERNATE,
    },
#endif
};

rt_inline rt_bool_t _set_baudrate(CAN_HandleTypeDef *handle, rt_uint32_t baudrate)
{
    /* baudrate calculate, pclk=90MHz
     * baudrate = 1 / (((SJW+1)+(TSEG1+1)+(TSEG2+1)) * (((BRP+1)*2)/pclk))
     * example 500kbps = 1 / ((1+4+5)*2*9*(1/90M))
     * */
    switch(baudrate)
    {
        case CAN1MBaud:         /* 1 Mbps   */
            handle->Init.CAN_SJW=CAN_SJW_1tq;
            handle->Init.CAN_BRP=8;
            handle->Init.CAN_TSEG1=CAN_TSEG1_4tq;
            handle->Init.CAN_TSEG2=CAN_TSEG2_5tq;
            break;
        case CAN800kBaud:       /* 800 kbps */
            handle->Init.CAN_SJW=CAN_SJW_1tq;
            handle->Init.CAN_BRP=3;
            handle->Init.CAN_TSEG1=CAN_TSEG1_8tq;
            handle->Init.CAN_TSEG2=CAN_TSEG2_5tq;
            break;
        case CAN500kBaud:       /* 500 kbps */
            handle->Init.CAN_SJW=CAN_SJW_1tq;
            handle->Init.CAN_BRP=8;
            handle->Init.CAN_TSEG1=CAN_TSEG1_4tq;
            handle->Init.CAN_TSEG2=CAN_TSEG2_5tq;
            break;
        case CAN250kBaud:       /* 250 kbps */
            handle->Init.CAN_SJW=CAN_SJW_1tq;
            handle->Init.CAN_BRP=17;
            handle->Init.CAN_TSEG1=CAN_TSEG1_4tq;
            handle->Init.CAN_TSEG2=CAN_TSEG2_5tq;
            break;
        case CAN125kBaud:       /* 125 kbps */
            handle->Init.CAN_SJW=CAN_SJW_1tq;
            handle->Init.CAN_BRP=35;
            handle->Init.CAN_TSEG1=CAN_TSEG1_4tq;
            handle->Init.CAN_TSEG2=CAN_TSEG2_5tq;
            break;
        default:
            return RT_FALSE;
    }
    return RT_TRUE;
}

rt_inline rt_bool_t _set_mode(CAN_HandleTypeDef *handle, rt_uint32_t mode)
{
    switch(mode)
    {
        case RT_CAN_MODE_NORMAL:
            HAL_CAN_OperatingModeRequest(handle, CAN_OperatingMode_Normal);
            break;
        case RT_CAN_MODE_LISEN:
            HAL_CAN_OperatingModeRequest(handle, CAN_OperatingMode_Listen);
            break;
        case RT_CAN_MODE_LOOPBACK:
            HAL_CAN_OperatingModeRequest(handle, CAN_OperatingMode_SelfTest);
            break;
//        case RT_CAN_MODE_LOOPBACKANLISEN:
//            HAL_CAN_OperatingModeRequest(handle, CAN_OperatingMode_SelfTest|CAN_OperatingMode_Listen);
//            break;
        default:
            return RT_FALSE;
    }
    return RT_TRUE;
}

static rt_err_t _configure(struct rt_can_device *can, struct can_configure *cfg)
{
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    struct acm32_can_device *canObj;
    canObj = rt_container_of(can, struct acm32_can_device, device);

    canObj->handle.Instance=canObj->config.instance;
    canObj->handle.Init.CAN_ABOM=CAN_ABOM_DISABLE;       /* enable bus off recover */
    canObj->handle.Init.CAN_Mode = CAN_Mode_Normal;

    if(RT_TRUE != _set_baudrate(&canObj->handle, cfg->baud_rate))
    {
        return -RT_EINVAL;
    }

    if(RT_TRUE != _set_mode(&canObj->handle, cfg->mode))
    {
        return -RT_EINVAL;
    }

    HAL_CAN_Init(&canObj->handle);

    HAL_CAN_ConfigFilter(&canObj->handle, &canObj->filter);

    return RT_EOK;
}

static rt_err_t _control(struct rt_can_device *can, int cmd, void *arg)
{
    RT_ASSERT(can != RT_NULL);

    struct acm32_can_device *canObj;
    canObj = rt_container_of(can, struct acm32_can_device, device);

    switch(cmd)
    {
        case RT_DEVICE_CTRL_SET_INT:
            RT_ASSERT(arg != RT_NULL);
            switch((rt_uint32_t)arg)
            {
                case RT_DEVICE_FLAG_INT_RX:
                    /* enable the can receive interrupt */
                    canObj->handle.Instance->IER |= CAN_IER_RIE;
                    break;
                case RT_DEVICE_FLAG_INT_TX:
                    /* enable the can transmit interrupt */
                    canObj->handle.Instance->IER |= CAN_IER_TIE;
                    break;
                case RT_DEVICE_CAN_INT_ERR:
                    /* enable the can transmit interrupt */
                    canObj->handle.Instance->IER |= CAN_IER_DOIE;
                    break;
                default:
                    return -RT_EINVAL;
            }
            break;
        case RT_DEVICE_CTRL_CLR_INT:
            RT_ASSERT(arg != RT_NULL);
            switch((rt_uint32_t)arg)
            {
                case RT_DEVICE_FLAG_INT_RX:
                    /* enable the can receive interrupt */
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Initialization);
                    CLEAR_BIT(canObj->handle.Instance->IER, CAN_IER_RIE);
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Normal);
                    break;
                case RT_DEVICE_FLAG_INT_TX:
                    /* enable the can transmit interrupt */
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Initialization);
                    CLEAR_BIT(canObj->handle.Instance->IER, CAN_IER_TIE);
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Normal);
                    break;
                case RT_DEVICE_CAN_INT_ERR:
                    /* enable the can transmit interrupt */
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Initialization);
                    CLEAR_BIT(canObj->handle.Instance->IER, CAN_IER_DOIE);
                    HAL_CAN_OperatingModeRequest(&canObj->handle,CAN_OperatingMode_Normal);
                    break;
                default:
                    return -RT_EINVAL;
            }
            break;
#ifdef RT_CAN_USING_HDR
        case RT_CAN_CMD_SET_FILTER:
            {
                if (RT_NULL == arg)
                {
                    /* default filter configure */
                    HAL_CAN_ConfigFilter(&canObj->handle, &canObj->filter);
                }
                else
                {
                    struct rt_can_filter_config *filter_cfg = (struct rt_can_filter_config *)arg;

                    if(filter_cfg->count == 1) /* single filter */
                    {
                        canObj->filter.CAN_FilterMode = CAN_FilterMode_Single;
                        if(filter_cfg->items[0].ide == RT_CAN_STDID)
                        {
                            canObj->filter.CAN_FilterId1 = (filter_cfg->items[0].id<<18) | (filter_cfg->items[0].rtr<<17);
                            canObj->filter.CAN_FilterMaskId1 = ~(filter_cfg->items[0].mask<<21);
                            CLEAR_BIT(canObj->filter.CAN_FilterMaskId1, BIT20);
                        }
                        else
                        {
                            /* maybe has a bug, cannot filt rtr bit */
                            canObj->filter.CAN_FilterId1 = filter_cfg->items[0].id;
                            canObj->filter.CAN_FilterMaskId1 = ~(filter_cfg->items[0].mask<<3);
                        }
                    }
                    else if(filter_cfg->count == 2) /* dual filter */
                    {
                        canObj->filter.CAN_FilterMode = CAN_FilterMode_Dual;

                        if(filter_cfg->items[0].ide == RT_CAN_STDID)
                        {
                            canObj->filter.CAN_FilterId1 = (filter_cfg->items[0].id<<18) | (filter_cfg->items[0].rtr<<17);
                            canObj->filter.CAN_FilterMaskId1 = ~(filter_cfg->items[0].mask<<21);
                            CLEAR_BIT(canObj->filter.CAN_FilterMaskId1, BIT20);
                        }
                        else
                        {
                            /* notice:only filt by bit28-bit18 */
                            canObj->filter.CAN_FilterId1 = filter_cfg->items[0].id;
                            canObj->filter.CAN_FilterMaskId1 = ~(filter_cfg->items[0].mask<<3);
                        }

                        if(filter_cfg->items[1].ide == RT_CAN_STDID)
                        {
                            canObj->filter.CAN_FilterId2 = (filter_cfg->items[1].id<<18) | (filter_cfg->items[1].rtr<<17);
                            canObj->filter.CAN_FilterMaskId2 = ~(filter_cfg->items[1].mask<<21);
                            CLEAR_BIT(canObj->filter.CAN_FilterMaskId2, BIT20);
                        }
                        else
                        {
                            /* notice:only filt by bit28-bit18 */
                            canObj->filter.CAN_FilterId2 = filter_cfg->items[1].id;
                            canObj->filter.CAN_FilterMaskId2 = ~(filter_cfg->items[1].mask<<3);
                        }
                    }
                    else
                    {
                        return -RT_EINVAL;
                    }
                    /* filter configure */
                    HAL_CAN_ConfigFilter(&canObj->handle, &canObj->filter);
                }
                break;
            }
#endif
        case RT_CAN_CMD_SET_BAUD:
            RT_ASSERT(arg != RT_NULL);
            if(RT_TRUE != _set_baudrate(&canObj->handle, (rt_uint32_t)arg))
            {
                return -RT_EINVAL;
            }
            break;

        case RT_CAN_CMD_SET_MODE:
            if(RT_TRUE != _set_mode(&canObj->handle, (rt_uint32_t)arg))
            {
                return -RT_EINVAL;
            }
            break;

        case RT_CAN_CMD_SET_PRIV:
            break;

        case RT_CAN_CMD_GET_STATUS:
            RT_ASSERT(arg != RT_NULL);
            canObj->device.status.rcverrcnt = HAL_CAN_GetReceiveErrorCounter(&canObj->handle);
            canObj->device.status.snderrcnt = HAL_CAN_GetTransmitErrorCounter(&canObj->handle);
            canObj->device.status.errcode = HAL_CAN_GetErrorCode(&canObj->handle, CAN_ErrorType_ErrCode);
            rt_memcpy(arg, &canObj->device.status, sizeof(canObj->device.status));
            break;

        default:
            return -RT_EINVAL;
    }

    return RT_EOK;
}
uint8_t flag = 0;
static int _transmit(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    struct acm32_can_device *canObj;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    CanTxRxMsg txMsg = {0};

    canObj = rt_container_of(can, struct acm32_can_device, device);

    if(pmsg->ide == RT_CAN_STDID)
    {
        txMsg.StdId = pmsg->id;
        txMsg.IDE = CAN_Id_Standard;
    }
    else
    {
        txMsg.ExtId = pmsg->id;
        txMsg.IDE = CAN_Id_Extended;
    }

    if(pmsg->rtr == RT_CAN_DTR)
    {
        txMsg.RTR = CAN_RTR_Data;
    }
    else
    {
        txMsg.RTR = CAN_RTR_Remote;
    }
    txMsg.DLC = pmsg->len;

    rt_memcpy(txMsg.Data, pmsg->data, txMsg.DLC);

    HAL_CAN_Transmit(&canObj->handle, &txMsg);

    return RT_EOK;
}

static int _receive(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    struct acm32_can_device *canObj;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    CanTxRxMsg rxMsg = {0};

    canObj = rt_container_of(can, struct acm32_can_device, device);

    HAL_CAN_GetRxMessage(&canObj->handle, &rxMsg);

    if(rxMsg.IDE == CAN_Id_Standard)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = rxMsg.StdId;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = rxMsg.ExtId;
    }

    if(rxMsg.RTR == CAN_RTR_Data)
    {
        pmsg->rtr = RT_CAN_DTR;
    }
    else
    {
        pmsg->rtr = RT_CAN_RTR;
    }

    pmsg->len = rxMsg.DLC;

    rt_memcpy(pmsg->data, rxMsg.Data, rxMsg.DLC);

    return RT_EOK;
}

static const struct rt_can_ops acm32_can_ops =
{
    .configure = _configure,
    .control = _control,
    .sendmsg = _transmit,
    .recvmsg = _receive,
};

static struct acm32_can_device devObjs[sizeof(can_config)/sizeof(can_config[0])] = {0};

int rt_hw_can_init(void)
{
    struct can_configure cfg = CANDEFAULTCONFIG;

    cfg.ticks = 50;
#ifdef RT_CAN_USING_HDR
    cfg.maxhdr = 2;
#endif
    cfg.baud_rate = CAN500kBaud;

    for(size_t i = 0; i < sizeof(can_config)/sizeof(can_config[0]); i++)
    {
        devObjs[i].config = can_config[i];
        devObjs[i].device.config = cfg;

        devObjs[i].filter.CAN_FilterMode = CAN_FilterMode_Single;
        devObjs[i].filter.CAN_FilterId1 = 0;
        devObjs[i].filter.CAN_FilterId2 = 0;
        devObjs[i].filter.CAN_FilterMaskId1 = 0xFFFFFFFF;
        devObjs[i].filter.CAN_FilterMaskId2 = 0xFFFFFFFF;

        rt_hw_can_register(&devObjs[i].device,
                            devObjs[i].config.name,
                            &acm32_can_ops,
                            RT_NULL);
    }

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_can_init);

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    struct acm32_can_device *canObj;
    GPIO_InitTypeDef   GPIO_InitStructure;

    RT_ASSERT(hcan != RT_NULL);

    /* get can object */
    canObj = rt_container_of(hcan, struct acm32_can_device, handle);

    /* Enable CAN clock */
    System_Module_Enable(canObj->config.enable_id);

    /* Initialization GPIO */
    GPIO_InitStructure.Pin = canObj->config.tx_pin;
    GPIO_InitStructure.Alternate=canObj->config.tx_alternate;
    GPIO_InitStructure.Pull=GPIO_PULLUP;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(canObj->config.tx_port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = canObj->config.rx_pin;
    GPIO_InitStructure.Alternate=canObj->config.rx_alternate;
    GPIO_InitStructure.Pull=GPIO_PULLUP;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(canObj->config.rx_port, &GPIO_InitStructure);

    /* Initialization interrupt */
    NVIC_ClearPendingIRQ(canObj->config.irq_type);
    NVIC_SetPriority(canObj->config.irq_type, canObj->config.irq_priority);
    NVIC_EnableIRQ(canObj->config.irq_type);
}

static void can_isr(rt_uint32_t idx)
{
    volatile uint32_t lu32_IR;
    lu32_IR = devObjs[idx].handle.Instance->IR; /* read clear */

    if(lu32_IR & CAN_IR_RI) /* receive interrupt */
    {
        // flag = 1;
        rt_hw_can_isr(&devObjs[idx].device, RT_CAN_EVENT_RX_IND | HAL_CAN_GetReceiveFiFoCounter(&devObjs[idx].handle) << 8);
    }

    if(lu32_IR & CAN_IR_TI) /* transmit interrupt */
    {
        rt_hw_can_isr(&devObjs[idx].device, RT_CAN_EVENT_TX_DONE | (0 << 8));
    }

    if(lu32_IR & CAN_IR_DOI) /* data overflow interrupt */
    {
        rt_hw_can_isr(&devObjs[idx].device, RT_CAN_EVENT_RXOF_IND | HAL_CAN_GetReceiveFiFoCounter(&devObjs[idx].handle) << 8);
    }

    if(lu32_IR & CAN_IR_EI) /* error alarm interrupt */
    {
    }

    if(lu32_IR & CAN_IR_WUI) /* wakeup interrupt */
    {
    }

    if(lu32_IR & CAN_IR_EPI) /* error-passive interrupt */
    {
    }

    if(lu32_IR & CAN_IR_ALI) /* arbitration failed interrupt */
    {
    }

    if(lu32_IR & CAN_IR_BEI) /* bus error interrupt */
    {
    }

    if(devObjs[idx].handle.Instance->MOD & CAN_MOD_RM) /* reset mode */
    {
        /* CAN bus off error recover */
        HAL_CAN_OperatingModeRequest(&devObjs[idx].handle,CAN_OperatingMode_Normal);//enter CAN_OperatingMode_Normal
        while(devObjs[idx].handle.Instance->SR & CAN_SR_BS);//wait bus off recover
    }
}

#ifdef BSP_USING_CAN1
void CAN1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* can interrupt service routine */
    can_isr(CAN1_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_CAN2
void CAN2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* can interrupt service routine */
    can_isr(CAN2_INDEX);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#endif /* defined(BSP_USING_CAN1) || defined(BSP_USING_CAN2) */
#endif /* RT_USING_CAN */

