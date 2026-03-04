//V1.5版本。

#ifndef _SELFCHECK_TASK
#define _SELFCHECK_TASK

#include "Includes.h"

#define		USB_NUC										0	//NUC，USB通信方式
#define		CAN_M3508_LF							1	//左前轮M3508电机，CAN通信方式
#define		CAN_M3508_RF							2	//右前轮M3508电机，CAN通信方式
#define		CAN_M3508_LB							3	//左后轮M3508电机，CAN通信方式
#define		CAN_M3508_RB							4	//右后轮M3508电机，CAN通信方式
#define		CAN_M3508_FL							5	//机械臂抬升M3508电机，CAN通信方式
#define		CAN_M3508_BL							6	//图传抬升M3508电机，CAN通信方式
#define		CAN_M3508_LT							7	//左履带M3508电机，CAN通信方式
#define		CAN_M3508_RT							8	//右履带M3508电机，CAN通信方式
#define   CAN_J4340_LF              9 //左前轮J4340关节电机，CAN通信方式
#define   CAN_J4340_RF              10 //右前轮J4340关节电机，CAN通信方式
#define   CAN_J4340_LB              11 //左后轮J4340关节电机，CAN通信方式
#define   CAN_J4340_RB              12 //右后轮J4340关节电机，CAN通信方式
#define		CAN_RMD4015_yaw						13	//云台麦塔4015yaw，CAN通信方式
#define		CAN_RMD4015_pitch					14	//云台麦塔4015pitch，CAN通信方式
#define		UART_RC										15	//遥控器接收机，DBUS(UART)通信方式
#define		UART_IMAGE								16	//图传链路，UART通信方式
#define		UART_ARM_C								17	//底盘C板，UART通信方式
#define		UART_REF									18  //常规链路，UART通信方式

//外设状态
typedef enum{
		DEVICE_DISORDER			= 0,	//设备（通信）异常
		DEVICE_NORMAL				= 1,	//设备（通信）正常
}device_state;

#endif
