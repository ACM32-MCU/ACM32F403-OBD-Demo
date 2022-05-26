/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-09     AisinoChip   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "board.h"

#ifdef RT_USING_CAN

#include "lib_iso15765.h"

static rt_device_t can_dev;            /* CAN 设备句柄 */
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */

static uint8_t send_frame(cbus_id_type id_type, uint32_t id, cbus_fr_format fr_fmt, uint8_t dlc, uint8_t* dt);
static void indication(n_indn_t* info);
static void on_error(n_rslt err_type);
static uint32_t getms(void);

static iso15765_t handler =
{
    .addr_md = N_ADM_NORMAL,             /* Selected address mode of the TP */
    .fr_id_type = CBUS_ID_T_STANDARD,   /* CANBus frame type */
    .config.stmin = 0x05,               /* Default min. frame transmission separation */
    .config.bs = 0x0F,                  /* Maximun size of the block sequence */
    .config.n_bs = 800,                 /* Time until reception of the next FlowControl N_PDU */
    .config.n_cr = 250,                 /* Time until reception of the next ConsecutiveFrame N_PDU */
//    .clbs.ff_indn = ff_indication,      /* First Frame Indication Callback: Will be fired when a
//                                           FF is received, giving back some useful information */
//    .clbs.cfm = confirm,                /* This callback confirms to the higher layers that the requested
//                                           service has been carried out */
//    .clbs.cfg_cfm = config_confirm,     /* This callback confirms to the higher layers that the requested
//                                           service has been carried out */
//    .clbs.pdu_custom_pack = pdu_pack,   /* Custom CAN ID packing for 11bits ID. If assinged the default
//                                           packing will be skipped */
//    .clbs.pdu_custom_unpack = pdu_unpack,/* Custom CAN ID unpacking for 11bits ID. If assinged the default
//                                           packing will be skipped */
    .clbs.get_ms = getms,               /* Time-source for the library in ms(required) */
    .clbs.on_error = on_error,          /* Callback which will be executed in any occured error. */
    .clbs.send_frame = send_frame,      /* This callback will be fired when a transmission of a canbus frame is ready. */
    .clbs.indn = indication             /* Indication Callback: Will be fired when a reception
                                           is available or an error occured during the reception. */
};

static rt_bool_t rcvpkg = RT_FALSE;

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{

    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    //rt_sem_release(&rx_sem);
    rcvpkg = RT_TRUE;

    return RT_EOK;
}

static uint8_t send_frame(cbus_id_type id_type, uint32_t id, cbus_fr_format fr_fmt, uint8_t dlc, uint8_t* dt)
{
    struct rt_can_msg msg = {0};

    msg.id = id;
    msg.ide = id_type == CBUS_ID_T_STANDARD ? RT_CAN_STDID:RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = dlc;
    msg.hdr = -1;
    rt_memmove(msg.data, dt, dlc);

    rt_kprintf("send can frame:\n");
    rt_kprintf("id %X ide %X rtr %X data : ", msg.id, msg.ide, msg.rtr);
    for(int i = 0; i< dlc; i++) rt_kprintf("%02X ", msg.data[i]);
    rt_kprintf("\n");
    if(0 == rt_device_write(can_dev, 0, &msg, sizeof(msg)))
    {
        return 1;
    }

    return 0;
}

rt_inline void show_indication(n_indn_t* info)
{
    const char* frm_fmt[] = {"CLASSICAL CAN", "CAN FD"};
    rt_kprintf("%s\n", frm_fmt[info->fr_fmt - 1]);
    rt_kprintf("info->n_ai.n_pr %02X\n", info->n_ai.n_pr);
    rt_kprintf("info->n_ai.n_sa %02X\n", info->n_ai.n_sa);
    rt_kprintf("info->n_ai.n_ta %02X\n", info->n_ai.n_ta);
    rt_kprintf("info->n_ai.n_ae %02X\n", info->n_ai.n_ae);
    rt_kprintf("info->n_ai.n_tt %02X\n", info->n_ai.n_tt);

    rt_kprintf("info->n_pci.fs %02X\n", info->n_pci.fs);
    rt_kprintf("info->n_pci.bs %02X\n", info->n_pci.bs);
    rt_kprintf("info->n_pci.sn %02X\n", info->n_pci.sn);
    rt_kprintf("info->n_pci.st %02X\n", info->n_pci.st);
    rt_kprintf("info->n_pci.pt %02X\n", info->n_pci.pt);
    rt_kprintf("info->n_pci.dl %04X\n", info->n_pci.dl);

    rt_kprintf("result is %04X\n", info->rslt);
    rt_kprintf("message is \n");
    for(int i =0 ;i<info->msg_sz;i++)
    {
        rt_kprintf("%02X ", info->msg[i]);
    }
    rt_kprintf("\n");
}

static void indication(n_indn_t* info)
{
    RT_ASSERT(info != RT_NULL);
    show_indication(info);
}

static void ff_indication(n_ff_indn_t* info)
{
    RT_ASSERT(info != RT_NULL);
}

static void confirm(n_cfm_t* cfm)
{

}

static void config_confirm(n_chg_param_cfm_t* cfm)
{

}

static void pdu_pack(n_pdu_t* pdu, uint32_t* id)
{

}

static void pdu_unpack(n_pdu_t* pdu, uint32_t* id)
{

}

static void on_error(n_rslt err_type)
{
    rt_kprintf("ERROR OCCURED!:%04x", err_type);
}

static uint32_t getms()
{
    return rt_tick_get_millisecond();
}

static void iso15765_thread(void* arg)
{
    canbus_frame_t frame;
    struct rt_can_msg rxmsg = {0};
    
    iso15765_init(&handler);

    while(1)
    {
        iso15765_process(&handler);

        if(rcvpkg) /* 有收到CAN数据包 */
        {
            rxmsg.hdr = -1;
            /* 从 CAN 读取一帧数据 */
            rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
                
            frame.fr_format = CBUS_FR_FRM_STD;
            frame.id_type = rxmsg.ide == RT_CAN_STDID ? CBUS_ID_T_STANDARD : CBUS_ID_T_EXTENDED;
            frame.dlc = rxmsg.len;
            frame.id = rxmsg.id;
            rt_memcpy(frame.dt, rxmsg.data, frame.dlc);

            rt_kprintf("recv can frame:\n");
            rt_kprintf("id %X ide %X rtr %X data : ", rxmsg.id, rxmsg.ide, rxmsg.rtr);
            for(int i = 0; i< frame.dlc; i++) rt_kprintf("%02X ", rxmsg.data[i]);
            rt_kprintf("\n");

            iso15765_enqueue(&handler, &frame);

            rcvpkg = RT_FALSE;
        }
        rt_thread_yield();
    }
}

static rt_uint8_t char_to_uint(char ch)
{
    if(ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    else if(ch >= 'A' && ch <= 'Z')
    {
        return ch - 'A' + 10;
    }
    else if(ch >= 'a' && ch <= 'z')
    {
        return ch - 'a' + 10;
    }
    return 0xFF;
}

static rt_int32_t hexstring_to_int(char* hex)
{
    rt_int32_t id = 0;
    rt_uint8_t val = 0;
    rt_size_t size = rt_strlen(hex);

    RT_ASSERT(size < 9);

    for(int i = 0; i<size; i++)
    {
        val = char_to_uint(hex[i]);
        if(val == 0xFF)
        {
            return -1;
        }
        id <<= 4;
        id |=val;
    }

    return id;
}

static rt_uint32_t hexstring_to_bytes(char* hex, rt_uint8_t *bytes)
{
    rt_size_t size = rt_strlen(hex);
    rt_uint8_t val;

    RT_ASSERT((size%2) == 0);
    RT_ASSERT(bytes != RT_NULL);

    for(int i = 0; i<size; i+=2)
    {
        val = char_to_uint(hex[i]);
        if(val == 0xFF)
        {
            return 0;
        }
        bytes[i/2] = val<<4;

        val = char_to_uint(hex[i+1]);
        if(val == 0xFF)
        {
            return 0;
        }
        bytes[i/2] |= val;
    }

    return size/2;
}

static void build_frame(n_req_t *frame, uint32_t id, uint8_t* dt, rt_uint32_t dlc)
{
    frame->n_ai.n_pr = 0x07; /* lower priority */
    frame->n_ai.n_sa = 0x07; /* source address */
    frame->n_ai.n_ta = 0x03; /* target address */
    frame->n_ai.n_ae = 0x00; /* optional address extension */
    frame->n_ai.n_tt = N_TA_T_PHY;
    frame->fr_fmt = CBUS_FR_FRM_STD;
    frame->msg_sz = dlc;
    rt_memmove(frame->msg, dt, dlc);
}

int iso15765_sample(int argc, char *argv[])
{
    rt_thread_t thread;
    char can_cmd[16] = {0};
    char can_data[256] = {0};
    rt_size_t size;
    rt_uint8_t data[256] = {0};
    rt_err_t res;

    if (argc == 3)
    {
        rt_strncpy(can_cmd, argv[1], 16);
        rt_strncpy(can_data, argv[2], 16);
    }
    else
    {
        rt_kprintf("input invalid\n for example:\tiso15765_sample init can_name\tiso15765 cmd cmd\n");
        return -1;
    }

    if(0 == rt_strcmp(can_cmd, "cmd"))
    {
        size = hexstring_to_bytes(can_data, data);
        if(size == 0)
        {
            rt_kprintf("can data %s invalid!\n", can_data);
            return -RT_ERROR;
        }

        n_req_t frame;
        build_frame(&frame, (rt_uint32_t)0, data, size);
        iso15765_send(&handler, &frame);
    }
    else if(0 == rt_strcmp(can_cmd, "init"))
    {
        /* 查找 CAN 设备 */
        can_dev = rt_device_find(can_data);
        if (!can_dev)
        {
            rt_kprintf("find %s failed!\n", can_data);
            return -RT_ERROR;
        }

        /* 初始化 CAN 接收信号量 */
        rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(can_dev, can_rx_call);

        /* 以中断接收及发送方式打开 CAN 设备 */
        res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
        RT_ASSERT(res == RT_EOK);  
//        res = rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);
//        RT_ASSERT(res == RT_EOK);
        //res = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_LOOPBACK);
        //RT_ASSERT(res == RT_EOK);

        /* 创建数据接收线程 */
        thread = rt_thread_create("i15765", iso15765_thread, RT_NULL, 1024, 25, 10);
        if (thread != RT_NULL)
        {
            rt_thread_startup(thread);
        }
        else
        {
            rt_kprintf("create can_rx thread failed!\n");
            return -1;
        }
    }
    else
    {
        rt_kprintf("command invalid\n for example:\tinit\n\tcmd\n");
        return -1;
    }

    return 0;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(iso15765_sample, iso15765 transport sample);
#endif

