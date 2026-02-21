/**
  ******************************************************************************
  @file     Saber_uart.c
  @brief    Atom-Robotics Saber系列惯导模块串口协议驱动： 
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-02-09 (Created) | 2026-02-21 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-21 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example

  ------------------------------------------------------------------------------
  @attention
    - 代码未经测试，谨慎使用
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_ATOMROBOTICS

/* Includes ------------------------------------------------------------------*/
#include "Saber_uart.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Saber_Data Saber_RAW_Data;//从Saber接收的原始数据
Euler_t Euler_offset={0.0f, 0.0f, 0.0f};//IMU的上电偏移值
Euler_t Euler;
uint8_t imu_init_flag = 0;
ProtocolHandler saber_uart={
        .package_length = IMU_DATA_PL,
        .header = Saber_Preamble1 << 8 | Saber_Preamble2,
        .header_length = IMU_PREAMBLE_LENGTH,
        .tail_flag = true,
        .tail = Saber_Tail,
        .tail_length = IMU_TAIL_LENGTH,
        .callback = Saber_GetMessage,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
uint8_t Atom_BCC(uint8_t *addr, uint8_t len);

/* function prototypes -------------------------------------------------------*/

/**
 * BCC校验
 * @param addr
 * @param len
 * @return
 * @note refer to Atom Robotics
 */
uint8_t Atom_BCC(uint8_t* addr, uint8_t len) {
    uint8_t XorData = 0;

    for (uint8_t i = 0; i < len; i++) {
        XorData ^= addr[i]; // 使用异或运算
    }

    return XorData;
}

/**
 * 限制角度输出 -180.0° ~ 180.0°
 * @param data
 * @return
 */
float limit_angle(float data) {
    if (data > 180.0f)
        return (data - 360.0f);
    else if (data < (0.0f - 180.0f))
        return (data + 360.0f);
    else
        return data;
}

/**
 * 按格式读取IMU原始数据，并解析Euler值
 * @param data
 */
void Saber_GetMessage(uint8_t* data) {
    Saber_RAW_Data.BCC = (int) data[25];                 //BCC
    if(Atom_BCC(data, IMU_DATA_PL - IMU_BCC_LENGTH - IMU_TAIL_LENGTH) != Saber_RAW_DATA.BCC){
        return;
    }

    //存入原始数据
    Saber_RAW_Data.Preamble.Preamble1 = (int) data[0];   //Preamble1         0x41
    Saber_RAW_Data.Preamble.Preamble2 = (int) data[1];   //Preamble2         0x78
    Saber_RAW_Data.MADDR = (int) data[2];                //MADDR             0xFF：广播模式
    Saber_RAW_Data.CID.RFD = (int) (data[3] >> 4) & 0x0F;//Return from Device
    Saber_RAW_Data.CID.CID = (int) data[3] & 0x0F;       //CID               host Configure
    Saber_RAW_Data.MID = (int) data[4];
    Saber_RAW_Data.PL = (int) data[5];                   //Package Length    0x13：19 Byte
    Saber_RAW_Data.Euler.PID = (uint16_t) data[6] | ((uint16_t) data[7] << 8);    //PID   0xB001：Saber_Euler
    Saber_RAW_Data.Euler.Length = (int) data[8];                                 //PL    0x10:16 Byte
    Saber_RAW_Data.Euler.Roll =
            (uint32_t) data[9] | ((uint32_t) data[10] << 8) | ((uint32_t) data[11] << 16) | ((uint32_t) data[12] << 24);
    Saber_RAW_Data.Euler.Pitch = (uint32_t) data[13] | ((uint32_t) data[14] << 8) | ((uint32_t) data[15] << 16) |
                                 ((uint32_t) data[16] << 24);
    Saber_RAW_Data.Euler.Yaw = (uint32_t) data[17] | ((uint32_t) data[18] << 8) | ((uint32_t) data[19] << 16) |
                               ((uint32_t) data[20] << 24);
    Saber_RAW_Data.TAIL = (int) data[26];                //TAIL              0x6D

    //对原始数据进行转换
    union {
        uint32_t intValue;
        float floatValue;
    } converter;
    converter.intValue = Saber_RAW_Data.Euler.Roll;
    Euler.Roll = converter.floatValue - Euler_offset.Roll;
    Euler.Roll = limit_angle(Euler.Roll);
    converter.intValue = Saber_RAW_Data.Euler.Pitch;
    Euler.Pitch = converter.floatValue - Euler_offset.Pitch;
    Euler.Pitch = limit_angle(Euler.Pitch);
    converter.intValue = Saber_RAW_Data.Euler.Yaw;
    Euler.Yaw = converter.floatValue - Euler_offset.Yaw;
    Euler.Yaw = limit_angle(Euler.Yaw);

    //Atom Robotics在使用手册中给出了SetAttitudeOffSet的通信报文，但是实测并没有成功
    //此处采用手动记录上电偏差，对数据进行置零
    if (!imu_init_flag) {
        Euler_offset.Roll = Euler.Roll;
        Euler_offset.Pitch = Euler.Pitch;
        Euler_offset.Yaw = Euler.Yaw;
        imu_init_flag = 1;
    }
//    usart5_printf("%f,%f,%f\r\n",Euler.Yaw,Euler.Roll,Euler.Pitch);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
