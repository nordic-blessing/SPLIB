//
// Created by Icol_Lee on 2026/2/2.
//

#include "splib.h"

#if USE_SPLIB_VISUAL_UART

/* Includes ------------------------------------------------------------------*/
#include "visual_uart.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Visual_t visualData;
ProtocolHandler visual_uart={
    .package_length =VISUAL_DATA_LENGTH,
    .header = VISUAL_HEADER1,
    .header_length = VISUAL_HEADER_LENGTH,
    .tail_flag = 1,
    .tail = VISUAL_TAIL2,
    .tail_length = VISUAL_TAIL_LENGTH,
    .callback = Visual_Receive,
    
    .buffer_index = 0,
    .header_found = false,
    .buffer = {0}
};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

void Visual_Receive(uint8_t *data) {
    uint32_t temp;
    float floatValue;

    temp = data[5] << 24 | data[4] << 16 | data[3] << 8 | data[2];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data1 = floatValue;
    temp = data[9] << 24 | data[8] << 16 | data[7] << 8 | data[6];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data2 = floatValue;
    temp = data[13] << 24 | data[12] << 16 | data[11] << 8 | data[10];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data3 = floatValue;
    temp = data[17] << 24 | data[16] << 16 | data[15] << 8 | data[14];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data4 = floatValue;
    temp = data[21] << 24 | data[20] << 16 | data[19] << 8 | data[18];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data5 = floatValue;
    temp = data[25] << 24 | data[24] << 16 | data[23] << 8 | data[22];
    memcpy(&floatValue, &temp, sizeof(float));
    visualData.data6 = floatValue;

//    uart_printf("%f, %f\r\n", visualData.data1, visualData.data2);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
