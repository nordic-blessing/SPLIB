//
// Created by Icol_Lee on 2025/11/26.
//

#include "splib.h"

#if USE_SPLIB_ROBOSTRIDE

#include "Robstride.h"

#define P_MIN (-12.57f)
#define P_MAX 12.57f
#define V_MIN (-50.0f)
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-6.0f)
#define T_MAX 6.0f

RobStride_Motor EL05;

// 初始化函数
void RobStride_Motor_Init(RobStride_Motor *motor, uint8_t CAN_Id) {
    motor->CAN_ID = CAN_Id;
    motor->Master_CAN_ID = 0xFD;
    motor->Motor_Set_All.set_motor_mode = CSP_control_mode;
    data_read_write_init(&motor->drw, RobStride_Index_List);
}

// IEEE 754浮点数转十六进制
union float_to_hex{
    float floatValue;
    uint32_t hexValue;
};
union float_to_hex f_h;

// uint16_t型转float型浮点数
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
    uint32_t span = (1 << bits) - 1;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

// float浮点数转int型
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int) (((x - offset) * ((float) ((1 << bits) - 1))) / span);
}

// uint8_t数组转float浮点数
float Byte_to_float(const uint8_t* byteData) {
    uint32_t data = byteData[7] << 24 | byteData[6] << 16 | byteData[5] << 8 | byteData[4];
    return *(float*)(&data);
}

// 接收处理函数
void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId) {
    if ((uint8_t) ((ID_ExtId & 0xFF00) >> 8) == motor->CAN_ID) {
        if ((int) ((ID_ExtId & 0x1F000000) >> 24) == Communication_Type_MotorRequest) { // 通信类型2
            motor->Pos_Info.Angle = uint16_to_float(DataFrame[0] << 8 | DataFrame[1], P_MIN, P_MAX, 16);
            motor->Pos_Info.Speed = uint16_to_float(DataFrame[2] << 8 | DataFrame[3], V_MIN, V_MAX, 16);
            motor->Pos_Info.Torque = uint16_to_float(DataFrame[4] << 8 | DataFrame[5], T_MIN, T_MAX, 16);
            motor->Pos_Info.Temp = (float) ((DataFrame[6] << 8 | DataFrame[7]) * 0.1);
            motor->error_code = (uint8_t) ((ID_ExtId & 0x3F0000) >> 16);
            motor->Pos_Info.pattern = (uint8_t) ((ID_ExtId & 0xC00000) >> 22);
        } else if ((int) ((ID_ExtId & 0x3F000000) >> 24) == Communication_Type_GetSingleParameter) { // 通信类型17
            for (int index_num = 0; index_num <= 13; index_num++) {
                if ((DataFrame[1] << 8 | DataFrame[0]) == RobStride_Index_List[index_num]) {
                    switch (index_num) {
                        case 0:
                            motor->drw.run_mode.data = (uint8_t) (DataFrame[4]);
                            break;
                        case 1:
                            motor->drw.iq_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 2:
                            motor->drw.spd_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 3:
                            motor->drw.imit_torque.data = Byte_to_float(DataFrame);
                            break;
                        case 4:
                            motor->drw.cur_kp.data = Byte_to_float(DataFrame);
                            break;
                        case 5:
                            motor->drw.cur_ki.data = Byte_to_float(DataFrame);
                            break;
                        case 6:
                            motor->drw.cur_filt_gain.data = Byte_to_float(DataFrame);
                            break;
                        case 7:
                            motor->drw.loc_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 8:
                            motor->drw.limit_spd.data = Byte_to_float(DataFrame);
                            break;
                        case 9:
                            motor->drw.limit_cur.data = Byte_to_float(DataFrame);
                            break;
                        case 10:
                            motor->drw.mechPos.data = Byte_to_float(DataFrame);
                            break;
                        case 11:
                            motor->drw.iqf.data = Byte_to_float(DataFrame);
                            break;
                        case 12:
                            motor->drw.mechVel.data = Byte_to_float(DataFrame);
                            break;
                        case 13:
                            motor->drw.VBUS.data = Byte_to_float(DataFrame);
                            break;
                        default:
                            break;
                    }
                }
            }
        } else if ((uint8_t) (ID_ExtId & 0xFF) == 0xFE) {
            motor->CAN_ID = (uint8_t) ((ID_ExtId & 0xFF00) >> 8);
            memcpy(&motor->Unique_ID, DataFrame, 8);
        }
    }
}

/**
 * 通信类型0：获取设备MCU
 * @param motor
 */
void RobStride_Get_CAN_ID(RobStride_Motor *motor) {
    uint32_t Id = Communication_Type_Get_ID << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};
    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通信类型1：运控模式电机控制指令
 * @param motor
 * @param Torque 力矩 [0~65535] 对应 -6~6 Nm
 * @param Angle 目标角度 [0~65535] 对应 -12.57~12.57 rad
 * @param Speed 目标速度 [0~65535] 对应 0.0~500.0 rad/s
 * @param Kp [0~65535] 对应 0.0f~500.0f
 * @param Kd [0~65535] 对应 0.0f~5.0f
 */
void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd) {
    uint32_t Id;
    uint8_t txData[8] = {0};

    motor->Motor_Set_All.set_Torque = (Torque < T_MIN) ? T_MIN : (Torque > T_MAX ? T_MAX : Torque);
    motor->Motor_Set_All.set_angle = (Angle < P_MIN) ? P_MIN : (Angle > P_MAX ? P_MAX : Angle);
    motor->Motor_Set_All.set_speed = (Speed < V_MIN) ? V_MIN : (Speed > V_MAX ? V_MAX : Speed);
    motor->Motor_Set_All.set_Kp = (Kp < KP_MIN) ? KP_MIN : (Kp > KP_MAX ? KP_MAX : Kp);
    motor->Motor_Set_All.set_Kd = (Kd < KD_MIN) ? KD_MIN : (Kd > KD_MAX ? KD_MAX : Kd);

    if (motor->drw.run_mode.data != move_control_mode) {
        Set_RobStride_Motor_parameter(motor, 0X7005, move_control_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        RobStride_Enable_Motor(motor);
        motor->Motor_Set_All.set_motor_mode = move_control_mode;
    }

    if (motor->Pos_Info.pattern != 2) {
        RobStride_Enable_Motor(motor);
    }

    Id = Communication_Type_MotionControl << 24 |
                           float_to_uint(motor->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8 | motor->CAN_ID;
    txData[0] = float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16) >> 8;
    txData[1] = float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
    txData[2] = float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16) >> 8;
    txData[3] = float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
    txData[4] = float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16) >> 8;
    txData[5] = float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16);
    txData[6] = float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16) >> 8;
    txData[7] = float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16);

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通信类型3：电机使能
 * @param motor
 */
void RobStride_Enable_Motor(RobStride_Motor *motor) {
    uint32_t Id = Communication_Type_MotorEnable << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通讯类型4：电机失能
 * @param motor
 * @param clear_error
 */
void RobStride_Disable_Motor(RobStride_Motor *motor, uint8_t clear_error) {
    uint32_t Id = Communication_Type_MotorStop << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    txData[0] = clear_error;

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
    Set_RobStride_Motor_parameter(motor, 0X7005, move_control_mode);
}

/**
 * 通讯类型6：设置电机机械零位
 * @param motor
 */
void Set_ZeroPos(RobStride_Motor *motor) {
    uint32_t Id = Communication_Type_SetPosZero << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    txData[0] = 1;

    RobStride_Disable_Motor(motor, 0);
    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
    RobStride_Enable_Motor(motor);
}

/**
 * 通讯类型7：设置电机CAN_ID
 * @param motor
 * @param Set_CAN_ID
 */
void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID) {
    uint32_t Id = Communication_Type_Can_ID << 24 | Set_CAN_ID << 16 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    RobStride_Disable_Motor(motor, 0);
    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
    RobStride_Enable_Motor(motor);
}

/**
 * 通讯类型17：单个参数读取
 * @param motor
 * @param Index
 */
void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index) {
    uint32_t Id = Communication_Type_GetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    txData[0] = Index;
    txData[1] = Index >> 8;

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通讯类型18：单个参数写入（掉电丢失）
 * @param motor
 * @param Index
 * @param Value
 */
void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value) {
    uint32_t Id = Communication_Type_SetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0};

    txData[0] = Index;
    txData[1] = Index >> 8;
    if(Index == 0x7005){
        txData[4] = (uint8_t)Value;
    }else {
        f_h.floatValue = Value;
        txData[4] = f_h.hexValue;
        txData[5] = f_h.hexValue >> 8;
        txData[6] = f_h.hexValue >> 16;
        txData[7] = f_h.hexValue >> 24;
    }

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通讯类型22：电机数据保存帧
 * @param motor
 */
void RobStride_Motor_MotorDataSave(RobStride_Motor *motor) {
    uint32_t Id = Communication_Type_MotorDataSave << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通讯类型23：电机波特率修改帧
 * @param motor
 * @param F_CMD 1:1M，2:500K，3:250K，4:125K
 */
void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD) {
    uint32_t Id = Communication_Type_BaudRateChange << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 00};

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 通讯类型24：电机主动上报帧
 * @param motor
 * @param F_CMD 0:关闭，1:开启（默认上报间隔位10ms）
 */
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD) {
    uint32_t Id = Communication_Type_ProactiveEscalationSet << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    uint8_t txData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x00};

    CAN_SendExtData(&Robostride_hcan, Id, txData, 8);
}

/**
 * 数据的参数地址初始化
 * @param drw
 * @param index_list
 */
void data_read_write_init(RobStride_data_read_write *drw, const uint16_t *index_list) {
    drw->run_mode.index = index_list[0];
    drw->iq_ref.index = index_list[1];
    drw->spd_ref.index = index_list[2];
    drw->imit_torque.index = index_list[3];
    drw->cur_kp.index = index_list[4];
    drw->cur_ki.index = index_list[5];
    drw->cur_filt_gain.index = index_list[6];
    drw->loc_ref.index = index_list[7];
    drw->limit_spd.index = index_list[8];
    drw->limit_cur.index = index_list[9];
    drw->mechPos.index = index_list[10];
    drw->iqf.index = index_list[11];
    drw->mechVel.index = index_list[12];
    drw->VBUS.index = index_list[13];
}

/**
 * 电流模式
 * @param motor
 * @param current
 */
void RobStride_Motor_current_control(RobStride_Motor *motor, float current) {
    motor->Motor_Set_All.set_current = current;

    if (motor->Motor_Set_All.set_motor_mode != 3) {
        Set_RobStride_Motor_parameter(motor, 0X7005, Elect_control_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        RobStride_Enable_Motor(motor);
        motor->Motor_Set_All.set_motor_mode = Elect_control_mode;
    }

    if (motor->Pos_Info.pattern != 2) {
        RobStride_Enable_Motor(motor);
    }

    Set_RobStride_Motor_parameter(motor, 0X7006, motor->Motor_Set_All.set_current);
}

/**
 * 速度模式
 * @param motor
 * @param Speed
 * @param limit_cur
 */
void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur, float Acceleration) {
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_limit_cur = limit_cur;
    motor->Motor_Set_All.set_acceleration = Acceleration;

    if (motor->drw.run_mode.data != Speed_control_mode) {
        Set_RobStride_Motor_parameter(motor, 0X7005, Speed_control_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Speed_control_mode;
        RobStride_Enable_Motor(motor);
    }

    if (motor->Pos_Info.pattern != 2) {
        RobStride_Enable_Motor(motor);
    }

    Set_RobStride_Motor_parameter(motor, 0X7018, motor->Motor_Set_All.set_limit_cur);
    Set_RobStride_Motor_parameter(motor, 0X7022, motor->Motor_Set_All.set_acceleration);
    DELAY(1);
    Set_RobStride_Motor_parameter(motor, 0X700A, motor->Motor_Set_All.set_speed);
}

/**
 * 位置模式(CSP位置模式控制)
 * @param motor
 * @param Angle
 * @param limit_spd
 */
void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd) {
    motor->Motor_Set_All.set_angle = Angle;
    motor->Motor_Set_All.set_limit_speed = limit_spd;

    if (motor->drw.run_mode.data != CSP_control_mode) {
        Set_RobStride_Motor_parameter(motor, 0X7005, CSP_control_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        RobStride_Enable_Motor(motor);
    }

    if (motor->Pos_Info.pattern != 2) {
        RobStride_Enable_Motor(motor);
    }

    DELAY(1);

    Set_RobStride_Motor_parameter(motor, 0X7017, motor->Motor_Set_All.set_limit_speed);
    Set_RobStride_Motor_parameter(motor, 0X7016, motor->Motor_Set_All.set_angle);
}

/**
 * 位置模式(PP插补位置模式控制)
 * @param motor
 * @param Angle
 * @param Speed
 * @param Acceleration
 */
void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Angle, float Speed, float Acceleration) {
    motor->Motor_Set_All.set_angle = Angle;
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_acceleration = Acceleration;

    if (motor->drw.run_mode.data != 1) {
        Set_RobStride_Motor_parameter(motor, 0X7005, Pos_control_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        RobStride_Enable_Motor(motor);
        motor->Motor_Set_All.set_motor_mode = Pos_control_mode;
    }

    if (motor->Pos_Info.pattern != 2) {
        RobStride_Enable_Motor(motor);
    }

    Set_RobStride_Motor_parameter(motor, 0X7024, motor->Motor_Set_All.set_speed);
    Set_RobStride_Motor_parameter(motor, 0X7025, motor->Motor_Set_All.set_acceleration);
    DELAY(1);
    Set_RobStride_Motor_parameter(motor, 0X7016, motor->Motor_Set_All.set_angle);
}

void RobStride_SetMode(RobStride_Motor *motor, uint8_t mode) {
    Set_RobStride_Motor_parameter(motor, 0X7005, Pos_control_mode);
    Get_RobStride_Motor_parameter(motor, 0x7005);
}

#endif
