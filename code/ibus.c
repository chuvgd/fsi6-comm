/************************************************
 * @file ibus.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief ibus 协议解析
 * @version 1.0.0
 * @date 2025-10-04
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 * @note
 * 富斯遥控器 IBUS 通信的解析代码，协议见官网：
 * [富斯AFHDS3 IBUS 协议通道数据格式公布公告](https://www.flyskytech.com/info_detail/18.html)
 *
 *
 ********************************/

/* ================================================================ include ================================================================ */

#include "ibus.h"

/* ================================================================ macro ================================================================ */

#define IBUS_FARME_LEN 32
#define IBUS_FARME_HEAD_0 0x20
#define IBUS_FARME_HEAD_1 0x40

/* ================================================================ typedef ================================================================ */

/* ================================================================ variable ================================================================ */

/* ================================================================ prototype ================================================================ */

static uint8_t IBUS_Checksum(const uint8_t *data, uint8_t len, uint32_t checksum);

/* ================================================================ function ================================================================ */

/**
 * @brief 接收数据
 *
 * @param hibus
 * @param data
 * @param len
 * @return uint8_t
 */
uint8_t IBUS_Parse(IBUS_Data_t *hibus, uint8_t *data, uint8_t len)
{
    // 数据校验
    if (
        data[0] != IBUS_FARME_HEAD_0 ||
        data[1] != IBUS_FARME_HEAD_1 ||
        IBUS_Checksum(data, IBUS_FARME_LEN - 2, data[30] | (data[31] << 8)))
    {
        hibus->state = -1;
        return hibus->state;
    }

    // 数据解析
    uint8_t i = 0; // channel index
    uint8_t t = 0; // data temp index

    // channel 1 --- 14
    for (; i < 14; i++)
    {
        t = 2 * i;
        hibus->channel[i] = (uint16_t)((data[t + 2] |
                                        (data[t + 3] << 8)) &
                                       0x07FF);
    }
    // channel 15 -- 18
    for (; i < 18; i++)
    {
        t = (i - 14) * 6;
        hibus->channel[i] = (uint16_t)((((data[t + 3] & 0xF0) >> 4) |
                                        (data[t + 5] & 0xF0) |
                                        ((data[t + 7] & 0xF0) << 4)) &
                                       0x07FF);
    }

    hibus->state = 0;
    return hibus->state;
}

/**
 * @brief 计算校验和
 *
 * @param data
 * @param len
 * @param checksum
 * @return uint8_t
 *
 * @note checksum = sum(byte[0:len]) ^ 0xFFFF
 */
static uint8_t IBUS_Checksum(const uint8_t *data, uint8_t len, uint32_t checksum)
{
    uint16_t checksum_now = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        checksum_now += data[i];
    }
    checksum_now ^= 0xFFFF;

    return (checksum_now == checksum) ? 0 : 1;
}
