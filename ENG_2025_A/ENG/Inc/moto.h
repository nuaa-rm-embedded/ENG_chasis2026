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
	CAN_Motor1_ID = 0x201,//CAN_M3508_RF
	CAN_Motor2_ID = 0x202,//CAN_M3508_LF
	CAN_Motor3_ID = 0x203,//CAN_M3508_LB
	CAN_Motor4_ID = 0x204,//CAN_M3508_RB
	CAN_Motor5_ID = 0x205,//CAN_M3508_LT 左履带
	CAN_Motor6_ID = 0x206,//CAN_M3508_RT
	CAN_Motor7_ID = 0x207,//CAN_M3508_FL
	CAN_Motor8_ID = 0x208,//CAN_M3508_BL

	CAN1_J4340_Motor1_ID=0x01,//LF 此处电机ID与DMmotodriver中电机ID相同，移植时重复定义，后续删改
	CAN1_J4340_Motor2_ID=0x02,//RF
	CAN1_J4340_Motor3_ID=0x03,//LB
	CAN1_J4340_Motor4_ID=0x04,//RB
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

extern MotoStateTD moto_chassis[6];
extern MotoStateTD moto_gimbal[2];

// can接收回调函数用
void SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

// 电机结构体初始化
void MotoStateInit(MotoStateTD* motostate);

// 电机电流赋值
void SetMotoCurrent(CAN_HandleTypeDef* hcan, MotoGroupe group, int16_t C1, int16_t C2, int16_t C3, int16_t C4);

//J4340电机控制
void Control_J4340(CAN_HandleTypeDef *hcan,uint32_t p1,uint32_t p2,uint32_t p3,uint32_t p4,uint32_t v1,uint32_t v2,uint32_t v3,uint32_t v4);
#endif
