#ifndef _MOTO_H
#define _MOTO_H

#include "Includes.h"

// 电机组别，Ahead电机id 1-4 Back5-8
typedef enum 
{
	Ahead,
	Back
}MotoGroupe;

// 电机ID
typedef enum
{
	CAN_Motor_ALL_ID = 0x200,
	CAN_Motor1_ID = 0x201,
	CAN_Motor2_ID = 0x202,
	CAN_Motor3_ID = 0x203,
	CAN_Motor4_ID = 0x204,
	CAN_Motor5_ID = 0x205,
	CAN_Motor6_ID = 0x206,
	CAN_Motor7_ID = 0x207,
	CAN_Motor8_ID = 0x208,
}CAN_Message_ID;

typedef struct 
{
	int16_t		speed_actual;
	int16_t 	speed_last;
	int16_t 	speed_desired;
	int16_t 	real_current;//实际转矩电流返回值
	int16_t 	given_current;//给定转矩电流
	int32_t  	angle_desired;//总角度目标值(累积所得)
	uint16_t 	angle_actual;//角度实际值(0~8192)
	uint16_t 	angle_last;
	int32_t		angle;//总角度(累积所得)
	int16_t 	turns;//转过的圈数
	int8_t		temperature;
	int16_t		original_position;//电机转子上电初位置
	bool        first_run;//是否为第一次读取数据,用于转子角度的计算
}MotoStateTD;

extern MotoStateTD moto_chassis[4];
extern MotoStateTD moto_gimbal[2];

// can接收回调函数用
void SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

// 电机结构体初始化
void MotoStateInit(MotoStateTD* motostate);

// 电机电流赋值
void SetMotoCurrent(CAN_HandleTypeDef* hcan, MotoGroupe group, int16_t C1, int16_t C2, int16_t C3, int16_t C4);

#endif
