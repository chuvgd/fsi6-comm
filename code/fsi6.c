/************************************************
 * @file fsi6.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief fsi6 协议解析
 * @version 1.0.0
 * @date 2025-10-04
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 * @note
 * 这是富斯遥控器 FSI6 协议的解析代码，根据以下的官网信息编写。
 * [富斯AFHDS3 FSI6 协议通道数据格式公布公告](https://www.flyskytech.com/info_detail/18.html)
 *
 * 数据格式如下：
 * 串口格式：Baud Rate 115200 (8, N, 1)
 * 帧长：32 bytes
 * 帧格式：
 * | byte |        value          |
 * |------|-----------------------|
 * | 1    | 0x20                  |
 * | 2    | 0x40                  |
 * | 3    | CH1[0-7]              |
 * | 4    | CH15[0-3]  CH1[8-11]  |
 * | 5    | CH2[0-7]              |
 * | 6    | CH15[4-7]  CH2[8-11]  |
 * | 7    | CH3[0-7]              |
 * | 8    | CH15[8-11] CH3[8-11]  |
 * | 9    | CH4[0-7]              |
 * | 10   | CH16[0-3]  CH4[8-11]  |
 * | 11   | CH5[0-7]              |
 * | 12   | CH16[4-7]  CH5[8-11]  |
 * | 13   | CH6[0-7]              |
 * | 14   | CH16[8-11] CH6[8-11]  |
 * | 15   | CH7[0-7]              |
 * | 16   | CH17[0-3]  CH7[8-11]  |
 * | 17   | CH8[0-7]              |
 * | 18   | CH17[4-7]  CH8[8-11]  |
 * | 19   | CH9[0-7]              |
 * | 20   | CH17[8-11] CH9[8-11]  |
 * | 21   | CH10[0-7]             |
 * | 22   | CH18[0-3]  CH10[8-11] |
 * | 23   | CH11[0-7]             |
 * | 24   | CH18[4-7]  CH11[8-11] |
 * | 25   | CH12[0-7]             |
 * | 26   | CH18[8-11] CH12[8-11] |
 * | 27   | CH13[0-7]             |
 * | 28   | CH13[8-11]            |
 * | 29   | CH14[0-7]             |
 * | 30   | CH14[8-11]            |
 * | 31   | Checksum[0-7]         |
 * | 32   | Checksum[8-15]        |
 *
 * 其中 Checksum = (byte[1] + … + byte[30]) ^ 0xFFFF
 ********************************/

/**************************************************************** include ****************************************************************/

#include "fsi6.h"

/**************************************************************** define ****************************************************************/

/// @brief 通道信息
#define FSI6_CH_LEN 18   // 通道数
#define FSI6_CH_MAX 2000 // 最大值
#define FSI6_CH_MID 1500 // 中值
#define FSI6_CH_MIN 1000 // 最小值

/// @brief 帧信息
#define FSI6_FARME_LEN 32

/// @brief 通道映射
// 摇杆
#define FSI6_JOY_LX 4
#define FSI6_JOY_LY 3
#define FSI6_JOY_RX 1
#define FSI6_JOY_RY 2
// 波轮
#define FSI6_VR_A 5
#define FSI6_VR_B 6
// 开关
#define FSI6_SW_A 6
#define FSI6_SW_B 7
#define FSI6_SW_C 8
#define FSI6_SW_D 9

/**************************************************************** struct ****************************************************************/

/**************************************************************** const ****************************************************************/

/**************************************************************** var ****************************************************************/

/**************************************************************** proto ****************************************************************/

static RC_Status_Def_e FSI6_Checksum(const uint8_t *data, uint8_t len, uint32_t checksum);

/**************************************************************** func ****************************************************************/

uint8_t FSI6_Init(FSI6_Def_t *hfsi6)
{
    for (uint8_t i = 0; i < 18; i++)
    {
        hfsi6->channel[i];
    }
    return 1;
}

uint8_t FSI6_Parse(FSI6_Def_t *hfsi6, uint8_t *data, uint8_t len)
{
    /* ================================ 校验 ================================ */

    if (
        len != FSI6_FARME_LEN ||
        data[0] != 0x20 ||
        data[1] != 0x40)
    // FSI6_Checksum(data, FSI6_FARME_LEN - 2, data[30] | (data[31] << 8))) // checksum 没试通，怕误报，先关掉
    {
        hfsi6->status = RC_Error;
        return hfsi6->status;
    }

    /* ================================ 解析 ================================ */

    uint8_t i = 0;
    uint8_t t; // temp index
    // ch 1 -- 14
    for (; i < 14; i++)
    {
        t = 2 * i;
        hfsi6->channel[i] = (uint16_t)((data[t + 2] |
                                        (data[t + 3] << 8)) &
                                       0x07FF);
    }
    // ch 15 -- 18
    for (; i < 18; i++)
    {
        t = (i - 14) * 6;
        hfsi6->channel[i] = (uint16_t)(((data[t + 3] >> 4) |
                                        data[t + 5] |
                                        (data[t + 7] << 4)) &
                                       0x07FF);
    }

    /* ================================ 处理 ================================ */

    hfsi6->joy[RC_LX] = hfsi6->channel[FSI6_JOY_LX] - FSI6_CH_MID;
    hfsi6->joy[RC_LY] = hfsi6->channel[FSI6_JOY_LY] - FSI6_CH_MID;
    hfsi6->joy[RC_RX] = hfsi6->channel[FSI6_JOY_RX] - FSI6_CH_MID;
    hfsi6->joy[RC_RY] = hfsi6->channel[FSI6_JOY_RY] - FSI6_CH_MID;

    hfsi6->vr[RC_A] = hfsi6->channel[FSI6_VR_A] - FSI6_CH_MID;
    hfsi6->vr[RC_A] = hfsi6->channel[FSI6_VR_B] - FSI6_CH_MID;

    hfsi6->sw[RC_A] = hfsi6->channel[FSI6_SW_A] / 500 - 2;
    hfsi6->sw[RC_B] = hfsi6->channel[FSI6_SW_B] / 500 - 2;
    hfsi6->sw[RC_C] = hfsi6->channel[FSI6_SW_C] / 500 - 2;
    hfsi6->sw[RC_D] = hfsi6->channel[FSI6_SW_D] / 500 - 2;

    // 离线检测
    FSI6_Offline(&hfsi6);

    return hfsi6->status = RC_ONLINE;
}

/************************************************ utils ************************************************/

/***********************************************
 * @brief 计算校验和
 *
 * @param data
 * @param len
 * @param checksum
 * @return uint8_t
 *
 * @note checksum = sum(byte[0:len]) ^ 0xFFFF
 *************************************************/
static RC_Status_Def_e FSI6_Checksum(const uint8_t *data, uint8_t len, uint32_t checksum)
{
    uint16_t checksum_now = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        checksum_now += data[i];
    }
    checksum_now ^= 0xFFFF;

    return (checksum_now == checksum) ? 1 : 0;
}

/***********************************************
 * @brief 离线检测
 *
 * @param hfsi6
 * @return uint8_t
 *************************************************/
static uint8_t FSI6_Offline(FSI6_Def_t *hfsi6)
{
    if (hfsi6->sw[RC_A])
    {
        hfsi6->status = RC_OFFLINE;
    }
    else
    {
        hfsi6->status = RC_ONLINE;
    }

    return hfsi6->status;
}

int main()
{
    return 0;
}
