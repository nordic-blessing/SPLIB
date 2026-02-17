//
// Created by Icol_Lee on 2025/9/27.
//
#include "splib.h"

#if USE_SPLIB_CAN | USE_SPLIB_FDCAN

#include "bsp_can.h"

static void (*pCAN1_RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN2_RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN3_RxCpltCallback)(CAN_RxBuffer *);

/**
 * 开启CAN中断，设置回调函数
 * @param hcan  can句柄
 * @param pFunc 回调函数
 */
void CAN_Start_IT(CAN_HandleTypeDef* hcan, void (*pFunc)(CAN_RxBuffer*)) {
#if USE_SPLIB_FDCAN
    /* 配置回调函数 */
    if (hcan->Instance == FDCAN1) {
        pCAN1_RxCpltCallback = pFunc;
    } else if (hcan->Instance == FDCAN2) {
        pCAN2_RxCpltCallback = pFunc;
    } else if (hcan->Instance == FDCAN3) {
        pCAN3_RxCpltCallback = pFunc;
    }

    /* 开启FDCAN外设 */
    HAL_FDCAN_Start(hcan);// !FDCAN外设的开启需要在中断的使能之前

    /* 使能中断 */
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
#endif

#if USE_SPLIB_CAN
    /* 配置回调函数 */
    if (hcan->Instance == CAN1) {
        pCAN1_RxCpltCallback = pFunc;
    } else if (hcan->Instance == CAN2) {
        pCAN2_RxCpltCallback = pFunc;
    }

    /* 开启CAN外设 */
    HAL_CAN_Start(hcan);// !CAN外设的开启需要在中断的使能之前

    /* 使能中断 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
}

/**
 * 配置CAN过滤器掩码模式
 * @param hcan
 * @param para  [|Filter编号|FIFOx(1bit)|ID类型(1bit)|帧类型(1bit)|]
 * @param Id    [ID]
 * @param Mask  [掩码(0x3ff,0x1FFFFFFF)]
 * @brief   CAN模式下需要配置过滤器过滤帧的帧类型
 *          FDCAN模式下无需关注帧类型，只过滤ID
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef* hcan, uint8_t para, uint32_t Id, uint32_t Mask) {
#if USE_SPLIB_FDCAN
    FDCAN_FilterTypeDef canFliter = {0};

    canFliter.IdType = ((para >> 1) & 0x01) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;    // ID类型
    canFliter.FilterIndex = para >> 3;                                                  // 过滤器索引
    canFliter.FilterType = FDCAN_FILTER_MASK;                                           // 过滤器类型
    canFliter.FilterConfig = (para >> 2) & 0x01;                                        // 过滤器关联FIFO
    canFliter.FilterID1 = Id;
    canFliter.FilterID2 = Mask;                                                         // FDCAN模式下直接填入ID掩码即可

    /*FDCAN初始化*/
    HAL_FDCAN_ConfigFilter(hcan, &canFliter);
#endif

#if USE_SPLIB_CAN
    CAN_FilterTypeDef canFliter = {0};

    if ((para >> 1 & 0x01)) {
        /* ExtId */
        canFliter.FilterIdHigh = (Id << 3) << 16;
        canFliter.FilterIdLow = (Id << 3) | (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
        canFliter.FilterMaskIdHigh = (Mask << 3) << 16;
        canFliter.FilterMaskIdLow = (Mask << 3) | (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
    } else {
        /* StdId */
        canFliter.FilterIdHigh = (Id << 5) << 16;
        canFliter.FilterIdLow = (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
        canFliter.FilterMaskIdHigh = (Mask << 5) << 16;
        canFliter.FilterMaskIdLow = (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
    }
    canFliter.FilterFIFOAssignment = (para >> 2) ? CAN_FILTER_FIFO1 : CAN_FILTER_FIFO0;
    canFliter.FilterBank = para >> 3;
    canFliter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFliter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFliter.FilterActivation = ENABLE;
    canFliter.SlaveStartFilterBank = 14;

    /*CAN初始化*/
    HAL_CAN_ConfigFilter(hcan, &canFliter);
#endif
}

/**
 * CAN发送标准数据帧
 * @param hcan
 * @param StdId
 * @param pData
 * @param Len
 */
void CAN_SendStdData(CAN_HandleTypeDef* hcan, uint16_t StdId, uint8_t* pData, uint8_t Len) {
#if USE_SPLIB_FDCAN
    FDCAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.Identifier = StdId;
    CAN_TxHeader.DataLength = Len;
    CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN_TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &CAN_TxHeader, pData);
    }
#endif

#if USE_SPLIB_CAN
    CAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.StdId = StdId;
    CAN_TxHeader.ExtId = 0;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = Len;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, pData, NULL);
    }
#endif
}

/**
 * CAN发送扩展数据帧
 * @param hcan
 * @param ExtId
 * @param pData
 * @param Len
 */
void CAN_SendExtData(CAN_HandleTypeDef* hcan, uint16_t ExtId, uint8_t* pData, uint8_t Len) {
#if USE_SPLIB_FDCAN
    FDCAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.Identifier = ExtId;
    CAN_TxHeader.DataLength = Len;
    CAN_TxHeader.IdType = FDCAN_EXTENDED_ID;
    CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN_TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &CAN_TxHeader, pData);
    }
#endif

#if USE_SPLIB_CAN
    CAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.StdId = 0;
    CAN_TxHeader.ExtId = ExtId;
    CAN_TxHeader.IDE = CAN_ID_EXT;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = Len;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, pData, NULL);
    }
#endif
}

/**
 * @brief 重定义RxFifo0的中断回调函数
 */
#if USE_SPLIB_FDCAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (hfdcan->Instance == FDCAN1) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN1_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
                if (pCAN1_RxCpltCallback != NULL) {
                    pCAN1_RxCpltCallback(&CAN1_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN2) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN2_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
                if (pCAN2_RxCpltCallback != NULL) {
                    pCAN2_RxCpltCallback(&CAN2_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN3) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN3_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_RxBuffer.header, CAN3_RxBuffer.data) == HAL_OK) {
                if (pCAN3_RxCpltCallback != NULL) {
                    pCAN3_RxCpltCallback(&CAN3_RxBuffer);
                }
            } else {}
        }
    }
}
#endif

#if USE_SPLIB_CAN
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN1_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
            pCAN1_RxCpltCallback(&CAN1_RxBuffer);
        } else {}
    } else if (hcan->Instance == CAN2) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN2_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
            pCAN2_RxCpltCallback(&CAN2_RxBuffer);
        } else {}
    }
}
#endif

/**
 * @brief 重定义RxFifo1的中断回调函数
 */
#if USE_SPLIB_FDCAN
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        /* Switch to user call back function. */
        if (hfdcan->Instance == FDCAN1) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN1_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
                pCAN1_RxCpltCallback(&CAN1_RxBuffer);
            } else {}
        } else if (hfdcan->Instance == FDCAN2) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN2_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
                pCAN2_RxCpltCallback(&CAN2_RxBuffer);
            } else {}
        } else if (hfdcan->Instance == FDCAN3) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN3_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN3_RxBuffer.header, CAN3_RxBuffer.data) == HAL_OK) {
                pCAN3_RxCpltCallback(&CAN3_RxBuffer);
            } else {}
        }
    }
}
#endif

#if USE_SPLIB_CAN
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN1_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
            pCAN1_RxCpltCallback(&CAN1_RxBuffer);
        } else {}
    } else if (hcan->Instance == CAN2) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN2_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
            pCAN2_RxCpltCallback(&CAN2_RxBuffer);
        } else {}
    }
}
#endif

#endif
