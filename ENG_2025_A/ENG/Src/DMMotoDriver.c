#include "DMMotoDriver.h"

uint8_t rx_data_dm[8]; // FIFO接收缓存区
DMMotoStateTD DMMotoState[8];

// 达妙速度位置控制模式(0x100)需要上位机先调成这个模式
// angle角度,单位rad(减速后)
// max_speed最大速度,单位rad/s(减速后)
void DMSetMoto_speed_and_position_single(DM_CAN_Ctrl_ID motor_can_ID, float angle, float max_speed)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = 0x100 + motor_can_ID; // 0x100 + 上位机can ID
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = *(uint8_t *)(&angle);
	TX_Data[1] = *((uint8_t *)(&angle) + 1);
	TX_Data[2] = *((uint8_t *)(&angle) + 2);
	TX_Data[3] = *((uint8_t *)(&angle) + 3);
	TX_Data[4] = *(uint8_t *)(&max_speed);
	TX_Data[5] = *((uint8_t *)(&max_speed) + 1);
	TX_Data[6] = *((uint8_t *)(&max_speed) + 2);
	TX_Data[7] = *((uint8_t *)(&max_speed) + 3);

	if(motor_can_ID == DM_Motor4_can_ID) HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, TX_Data, &send_mail_box);
	else HAL_CAN_AddTxMessage(&DM_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}

// 达妙速度控制模式(0x200)需要上位机先调成这个模式
// speed速度,单位rad/s(减速后)
void DMSetMoto_speed_single(DM_CAN_Ctrl_ID motor_can_ID, float speed)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = 0x200 + motor_can_ID; // 0x200 + 上位机can ID
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = *(uint8_t *)(&speed);
	TX_Data[1] = *((uint8_t *)(&speed) + 1);
	TX_Data[2] = *((uint8_t *)(&speed) + 2);
	TX_Data[3] = *((uint8_t *)(&speed) + 3);
	TX_Data[4] = 0;
	TX_Data[5] = 0;
	TX_Data[6] = 0;
	TX_Data[7] = 0;

	if(motor_can_ID == DM_Motor4_can_ID)
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, TX_Data, &send_mail_box);
	else HAL_CAN_AddTxMessage(&DM_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void DMSetMoto_MIT_single(DM_CAN_Ctrl_ID motor_can_ID, float pos, float vel,float kp, float kd, float tor)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = motor_can_ID; // 上位机can ID
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(pos, -PMAX, PMAX, 16);
	vel_tmp = float_to_uint(vel, -VMAX, VMAX, 12);
	tor_tmp = float_to_uint(tor, -TMAX, TMAX, 12);
	kp_tmp  = float_to_uint(kp,  0.0f, 500.0f, 12);
	kd_tmp  = float_to_uint(kd,  0.0f, 5.0f, 12);

	TX_Data[0] = (pos_tmp >> 8);
	TX_Data[1] = pos_tmp;
	TX_Data[2] = (vel_tmp >> 4);
	TX_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	TX_Data[4] = kp_tmp;
	TX_Data[5] = (kd_tmp >> 4);
	TX_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	TX_Data[7] = tor_tmp;

	if(motor_can_ID == DM_Motor4_can_ID) HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, TX_Data, &send_mail_box);
	else HAL_CAN_AddTxMessage(&DM_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}



// 达妙使能
void DM_Enable(DM_CAN_Ctrl_ID motor_can_ID, DM_Mode_ID Mode)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = Mode + motor_can_ID;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = 0xFF;
	TX_Data[1] = 0xFF;
	TX_Data[2] = 0xFF;
	TX_Data[3] = 0xFF;
	TX_Data[4] = 0xFF;
	TX_Data[5] = 0xFF;
	TX_Data[6] = 0xFF;
	TX_Data[7] = 0xFC;

	if(motor_can_ID == DM_Motor4_can_ID) HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, TX_Data, &send_mail_box);
	else HAL_CAN_AddTxMessage(&DM_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);

}

// 达妙失能
void DM_Disable(DM_CAN_Ctrl_ID motor_can_ID, DM_Mode_ID Mode)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = Mode + motor_can_ID;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = 0xFF;
	TX_Data[1] = 0xFF;
	TX_Data[2] = 0xFF;
	TX_Data[3] = 0xFF;
	TX_Data[4] = 0xFF;
	TX_Data[5] = 0xFF;
	TX_Data[6] = 0xFF;
	TX_Data[7] = 0xFD;

	if(motor_can_ID == DM_Motor4_can_ID) HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, TX_Data, &send_mail_box);
	else HAL_CAN_AddTxMessage(&DM_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}

void DM_SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
	CAN_RxHeaderTypeDef Rx_Msg;

	HAL_CAN_GetRxMessage(hcan, RxFifo, &Rx_Msg, rx_data_dm); // 接收can1 fifo0邮箱的数据帧。不接收会导致fifo邮箱一直爆满，无限进入接收中断，卡死所有任务。

	switch (Rx_Msg.StdId) // can1 motor message decode
	{
		case DM_Motor1_master_ID: // 上位机中的master_ID
			DMUpdateMotoState(&DMMotoState[0]);
			break;
		case DM_Motor2_master_ID: // 上位机中的master_ID
			DMUpdateMotoState(&DMMotoState[1]);
			break;
		case DM_Motor3_master_ID: // 上位机中的master_ID
		{
			DMUpdateMotoState(&DMMotoState[2]);
			if(DMMotoState[2].angle <= -1.57) DMMotoState[2].angle += 2 * PI;
			break;
		}
//		case DM_Motor4_master_ID: // 上位机中的master_ID
//		{
//			DMUpdateMotoState(&DMMotoState[3]);
//			if(DMMotoState[3].angle >= 1.15) DMMotoState[3].angle -= 2 * PI;
//			break;
//		}
		default:
			break;
	}
}

/**
************************************************************************
* @brief:      	DMUpdateMotoState: 获取DM4310电机反馈数据函数
* @param[in]:   state:    指向DMMotoStateTD结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/

extern float vofa_send_data[10];

void DMUpdateMotoState(DMMotoStateTD *state)
{
	state->err = (rx_data_dm[0]) >> 4;
	state->angle_int = (rx_data_dm[1] << 8) | rx_data_dm[2];
	state->speed_int = (rx_data_dm[3] << 4) | (rx_data_dm[4] >> 4);
	state->current_int = ((rx_data_dm[4] & 0xF) << 8) | rx_data_dm[5];
	state->angle = uint_to_float(state->angle_int, -PMAX, PMAX, 16); // 一般在上位机上设置PMAX,VMAX,TMAX
	state->speed = uint_to_float(state->speed_int, -VMAX, VMAX, 12);
	state->current = uint_to_float(state->current_int, -TMAX, TMAX, 12);
	state->Tmos = (float)(rx_data_dm[6]);
	state->Tcoil = (float)(rx_data_dm[7]);
	
	// vofa调试电机
	//  vofa_send_data[0] = DMMotoState[0].speed;
	//  vofa_send_data[1] = DM1_position_pid.outPID;
//	my_vofa_printf(2);
}
