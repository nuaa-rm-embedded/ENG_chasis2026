//V1.4版本。

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
#define		CAN_RMD4015_yaw						7	//云台麦塔4015yaw，CAN通信方式
#define		CAN_RMD4015_pitch					8	//云台麦塔4015pitch，CAN通信方式
#define		UART_RC										9	//遥控器接收机，DBUS(UART)通信方式
#define		UART_IMAGE								10	//图传链路，UART通信方式
#define		UART_ARM_C								11	//底盘C板，UART通信方式
#define		UART_REF									12  //常规链路，UART通信方式

//外设状态
typedef enum{
		DEVICE_DISORDER			= 0,	//设备（通信）异常
		DEVICE_NORMAL				= 1,	//设备（通信）正常
}device_state;

#endif
