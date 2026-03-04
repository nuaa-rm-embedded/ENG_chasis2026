/*
无用，新版车已用舵机替代
*/

#include "MTMotoDriver.h"

extern uint8_t rx_data[8]; // FIFO接收缓存区

// 参数1：canID
// 参数2：电流控制值(0.01A/LSB)
void MT_SetMoto_Current(MT_CAN_ID motor_can_ID, int16_t current)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = 0x140 + motor_can_ID;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = 0xA1;
	TX_Data[1] = 0x00;
	TX_Data[2] = 0x00;
	TX_Data[3] = 0x00;
	TX_Data[4] = (uint8_t)(current);
	TX_Data[5] = (uint8_t)(current >> 8);
	TX_Data[6] = 0x00;
	TX_Data[7] = 0x00;

	HAL_CAN_AddTxMessage(&MT_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}

void MTSetMotoSpeed_single(MT_CAN_ID moto_id, int32_t speed)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = 0x140 + moto_id;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = 0xA4;
	TX_Data[1] = 0x00;
	TX_Data[2] = 0x00;
	TX_Data[3] = 0x00;
	TX_Data[4] = (uint8_t)(speed);
	TX_Data[5] = (uint8_t)(speed >> 8);
	TX_Data[6] = (uint8_t)(speed >> 16);
	TX_Data[7] = (uint8_t)(speed >> 24);

	/* send can command */
	HAL_CAN_AddTxMessage(&MT_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}

void MTSetMotoPosition_single(MT_CAN_ID moto_id, uint16_t MaxSpeed, int32_t Angle)
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = 0x140 + moto_id;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;

	TX_Data[0] = 0xA4;
	TX_Data[1] = 0x00;
	TX_Data[2] = (uint8_t)(MaxSpeed);
	TX_Data[3] = (uint8_t)(MaxSpeed >> 8);
	TX_Data[4] = (uint8_t)(Angle);
	TX_Data[5] = (uint8_t)(Angle >> 8);
	TX_Data[6] = (uint8_t)(Angle >> 16);
	TX_Data[7] = (uint8_t)(Angle >> 24);

	/* send can command */
	HAL_CAN_AddTxMessage(&MT_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}


void MTUpdateMotoState(MTMotoStateTD *state)
{
	state->temp = (int8_t)rx_data[1];
	state->current = (int16_t)(rx_data[2] | (rx_data[3] << 8));
	state->speed = (int16_t)(rx_data[4] | (rx_data[5] << 8));
	state->angle = (int16_t)(rx_data[6] | (rx_data[7] << 8));

}



