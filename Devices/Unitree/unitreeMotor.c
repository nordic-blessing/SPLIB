#include "unitreeMotor.h"

#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}

/// @brief 将发送给电机的浮点参数转换为定点类型参数
/// @param motor_s 要转换的电机指令结构体
void unitree_modify_data(MotorCmd_t *motor_s)
{
    if(motor_s->motorType == A1){
		motor_s->A1B1_motor_send_data.head.start[0] = 0xFE;
		motor_s->A1B1_motor_send_data.head.start[1] = 0xEE;

		SATURATE(motor_s->id, 0, 15);
		SATURATE(motor_s->mode, 0, 10);
		SATURATE(motor_s->K_P, 0.0f, 25.599f);
		SATURATE(motor_s->K_W, 0.0f, 25.599f);
		SATURATE(motor_s->T, -127.99f, 127.99f);
		SATURATE(motor_s->W, -804.00f, 804.00f);
		SATURATE(motor_s->Pos, -411774.0f, 411774.0f);
		
		motor_s->A1B1_motor_send_data.head.motorID = motor_s->id;
		motor_s->A1B1_motor_send_data.Mdata.mode = motor_s->mode;
		motor_s->A1B1_motor_send_data.Mdata.T = motor_s->T * 256;
		motor_s->A1B1_motor_send_data.Mdata.W = motor_s->W * 128;
		motor_s->A1B1_motor_send_data.Mdata.Pos = motor_s->Pos / 6.28f * 16384.0f;
		motor_s->A1B1_motor_send_data.Mdata.K_P = motor_s->K_P * 2048;
		motor_s->A1B1_motor_send_data.Mdata.K_W = motor_s->K_W * 1024;
		motor_s->A1B1_motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)&motor_s->A1B1_motor_send_data, 7);
		
    }else if(motor_s->motorType == GO_M8010_6){
		motor_s->GO_M8010_6_motor_send_data.head[0] = 0xFE;
		motor_s->GO_M8010_6_motor_send_data.head[1] = 0xEE;

		SATURATE(motor_s->id, 0, 15);
		SATURATE(motor_s->mode, 0, 10);
		SATURATE(motor_s->K_P, 0.0f, 25.599f);
		SATURATE(motor_s->K_W, 0.0f, 25.599f);
		SATURATE(motor_s->T, -127.99f, 127.99f);
		SATURATE(motor_s->W, -804.00f, 804.00f);
		SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

		motor_s->GO_M8010_6_motor_send_data.mode.id = motor_s->id;
		motor_s->GO_M8010_6_motor_send_data.mode.status = motor_s->mode;
		motor_s->GO_M8010_6_motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768.0f;
		motor_s->GO_M8010_6_motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768.0f;
		motor_s->GO_M8010_6_motor_send_data.comd.pos_des = motor_s->Pos / 6.28318f * 32768.0f;
		motor_s->GO_M8010_6_motor_send_data.comd.spd_des = motor_s->W / 6.28318f * 256.0f;
		motor_s->GO_M8010_6_motor_send_data.comd.tor_des = motor_s->T * 256.0f;
		motor_s->GO_M8010_6_motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->GO_M8010_6_motor_send_data, sizeof(RIS_ControlData_t) - sizeof(motor_s->GO_M8010_6_motor_send_data.CRC16));
	}
}

	

/// @brief 将接收到的定点类型原始数据转换为浮点参数类型
/// @param motor_r 要转换的电机反馈结构体
uint8_t unitree_extract_data(MotorData_t* motor_r)
{
	if(motor_r->motorType == GO_M8010_6){
		if (motor_r->GO_M8010_6_motor_recv_data.head[0] != 0xFD || motor_r->GO_M8010_6_motor_recv_data.head[1] != 0xEE){
			motor_r->correct = 0;
			return 1;
		}
		motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->GO_M8010_6_motor_recv_data, sizeof(RIS_MotorData_t) - sizeof(motor_r->GO_M8010_6_motor_recv_data.CRC16));
		if (motor_r->GO_M8010_6_motor_recv_data.CRC16 != motor_r->calc_crc){
			memset(&motor_r->GO_M8010_6_motor_recv_data, 0, sizeof(RIS_MotorData_t));
			motor_r->correct = 0;
			return 1;
		} else {
			motor_r->motor_id = motor_r->GO_M8010_6_motor_recv_data.mode.id;
			motor_r->mode = motor_r->GO_M8010_6_motor_recv_data.mode.status;
			motor_r->Temp = motor_r->GO_M8010_6_motor_recv_data.fbk.temp;
			motor_r->MError = motor_r->GO_M8010_6_motor_recv_data.fbk.MError;
			motor_r->W = ((float)motor_r->GO_M8010_6_motor_recv_data.fbk.speed / 256.0f) * 6.28318f;
			motor_r->T = ((float)motor_r->GO_M8010_6_motor_recv_data.fbk.torque) / 256.0f;
			motor_r->Pos = 6.28318f * ((float)motor_r->GO_M8010_6_motor_recv_data.fbk.pos) / 32768.0f / 6.33f;
			motor_r->correct = 1;
			return 0;
		}
	} else if(motor_r->motorType == A1) {
		if(motor_r->A1B1_motor_recv_data.head.start[0] != 0xFE || motor_r->A1B1_motor_recv_data.head.start[1] != 0xEE){
			motor_r->correct = 0;
			return 1;
		}
        // crc 检验存在问题，目前暂时直接忽略了检验
//		motor_r->calc_crc = crc32_core((uint32_t *)&motor_r->A1B1_motor_recv_data, 18);
//		if(motor_r->A1B1_motor_recv_data.CRCdata.u32 != motor_r->calc_crc){
//			memset(&motor_r->A1B1_motor_recv_data, 0, sizeof(ServoComdDataV3));
//			motor_r->correct = 0;
//			return 1;
//		} else {
			motor_r->motor_id = motor_r->A1B1_motor_recv_data.head.motorID;
			motor_r->mode = motor_r->A1B1_motor_recv_data.Mdata.mode;
			motor_r->Temp = motor_r->A1B1_motor_recv_data.Mdata.Temp;
			motor_r->MError = motor_r->A1B1_motor_recv_data.Mdata.MError;
			motor_r->T = ((float)motor_r-> A1B1_motor_recv_data.Mdata.T) / 256.0f;
			motor_r->W = (((float)motor_r->A1B1_motor_recv_data.Mdata.W) / 128.0f) * 6.28318f;
			motor_r->Acc = (int)motor_r-> A1B1_motor_recv_data.Mdata.Acc;

			motor_r->Pos = 6.2832f * ((float)motor_r->A1B1_motor_recv_data.Mdata.Pos) / 16384.0f / 9.0f;
			motor_r->correct = 1;
			return 0;
//		}
	}
}
