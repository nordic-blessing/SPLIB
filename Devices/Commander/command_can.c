/**
******************************************************************************
  @file     command_can.c
  @brief    私有CAN协议
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-01-30 (Created) | 2026-04-28 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-01-30 [v1.0] Icol Boom: 创建初始版本，完成初步测试
    - 2026-06-20 [v1.1] Icol Boom: 完善私有协议，补全注释
  ------------------------------------------------------------------------------
  @example
    - 关于CAN ID: [DEVICE_ID(4 bits)] | [CMD_ID(5 bits)] | [para(2 bits)]
    - 其中 DEVICE_ID: ((COMMAND_SLAVE << 3)|DEVICE_ADDR)) (4 bits)
    （以上部分是自己定义的私有协议格式，可以根据需要自行修改）
  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于`bsp_can.c/h`，请务必在`splib_config.h`中使能`USE_SPLIB_CAN`或
        `USE_SPLIB_FDCAN`
    - 代码未经测试，谨慎使用
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_CONMMAND

/* Includes ------------------------------------------------------------------*/
#include "command_can.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 发送数据
 * @param CMD_ID    消息ID COMMAND_CMD_x (5 bits)
 * @param para      保留位 (2 bits)
 * @param pData
 */
void command_transmit(uint8_t CMD_ID, uint8_t para, uint8_t *pData) {
    uint16_t std_id = (DEVICE_ID<<7) | (CMD_ID << 2) | (para);
    CAN_SendStdData(&COMMAND_CAN, std_id, pData, 8);
}

/**
 * 接收数据
 * @param pData
 * @note
 *  CMD_2
 */
void command_receive(CAN_RxBuffer* rxBuffer) {
    // 识别设备身份
    bool isMaster = (rxBuffer->header.Identifier >> 10)&0x1;

    if (isMaster) {
        // 主控制器
        uint8_t CmdID = (rxBuffer->header.Identifier >> 2) & 0x1F;
        uart_printf("CmdID:%d\r\n", CmdID); // 接收指令ID

        // echo
        uint8_t data[8] = {0};
        command_transmit(COMMAND_CMD_4, 0x01, data);

        switch (CmdID) {
            // Spear夹取准备
            case COMMAND_CMD_2: {
                osEventFlagsSet(gTaskEvtHandle, EVT_TASK_SPEAR_PREPARE);
            }
            break;
            // Spear抓取
            case COMMAND_CMD_3: {
                osEventFlagsSet(gTaskEvtHandle, EVT_TASK_SPEAR_CATCH);
            }
            break;
            // Spear释放
            case COMMAND_CMD_4: {
                osEventFlagsSet(gTaskEvtHandle, EVT_TASK_SPEAR_RELEASE);
            }
            break;
            // 有关KFS的命令
            case COMMAND_CMD_5: {
                uart_printf("%d\r\n", rxBuffer->data[0]);
                switch (rxBuffer->data[0]) {
                    // 400平台
                    case 1: {
                        // 抓取通道处KFS，抓取后放入存储区
                        osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_1_1);
                    }
                    break;
                    // 200平台
                    case 2: {
                        if (rxBuffer->data[1] == 1) {
                            // 抓取200平台上的KFS，抓取后放入存储区
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_2_1);
                        }else if (rxBuffer->data[1] == 2) {
                            // 抓取200平台上的KFS，抓取后保持在吸盘
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_2_0);
                        }
                    }
                    break;
                    // N200平台
                    case 3: {
                        if (rxBuffer->data[1] == 1) {
                            // 抓取-200平台上的KFS，抓取后放入存储区
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_3_1);
                        }else if (rxBuffer->data[1] == 2) {
                            // 抓取-200平台上的KFS，抓取后保持在吸盘
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_3_0);
                        }
                    }
                    break;
                    // 地面
                    case 4: {
                        osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_GRAB_4_0);
                    }
                    break;
                    // 后备箱
                    case 5: {
                        osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_UNLOAD);
                    }
                    break;

                    // 清理
                    case 6: {
                        if (rxBuffer->data[1] == 1) {
                            // 向右清理200
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_UP_1);
                        }else if (rxBuffer->data[1] == 2) {
                            // 向左清理200
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_UP_2);
                        }
                    }
                    break;
                    case 7: {
                        if (rxBuffer->data[1] == 1) {
                            // 向右清理N200
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_DOWN_1);
                        }else if(rxBuffer->data[1] == 2){
                            // 向左清理N200
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_DOWN_2);
                        }
                    }
                    break;
                    case 8: {
                        if (rxBuffer->data[1] == 1) {
                            // 向右清理400
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_TOP_1);
                        }else if (rxBuffer->data[1] == 2) {
                            // 向左清理400
                            osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_CLEAN_TOP_2);
                        }
                    }
                    break;
                    default:
                        break;
                }
            }
            break;
            // Spear对准
            case COMMAND_CMD_6: {
                osEventFlagsSet(gTaskEvtHandle, EVT_TASK_SPEAR_AIM);
            }
            break;
            // KFS释放
            case COMMAND_CMD_7: {
                osEventFlagsSet(gTaskEvtHandle, EVT_TASK_KFS_PUT);
            }
            break;
            default:
                break;
        }
    }else {
        // 从控制器
    }
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
