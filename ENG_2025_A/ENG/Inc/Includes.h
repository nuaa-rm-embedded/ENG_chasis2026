#ifndef __INCULEDS_H
#define __INCULEDS_H

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "crc8_crc16.h"
#include "crc.h"
#include "protocol.h"
#include <stdbool.h>

#include "ui.h"
#include "ui_g.h"
#include "ui_interface.h"
#include "ui_types.h"

#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "usb_device.h"
#include "robot_message.h"
#include "bsp_usb.h"

#include "pwm.h"
#include "time.h"
#include "moto.h"
#include "MTMotoDriver.h"
#include "PID.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "modes.h"
#include "RcDriver.h"
#include "Key.h"
#include "chassis.h"
#include "referee.h"
#include "wd.h"
#include "lift_moto_ctrl.h"
#include "Arm_Whole_Ctrl.h"
#include "relay.h"
#include "SelfCheck_Task.h"

//typedef struct RobotTD
//{
//    //底盘四电机
//    int16_t x;
//    int16_t y;
//    int16_t z;
//    //云台抬升两电机
//    int16_t high_gimbal;
//    int16_t high_arm;
//    //机械臂电机
//    int16_t angle_arm[6];
//    //模式参数
//    PoseMode posemod;
//    PoseMode last_posemod;
//}RobotTD;

extern RC_Ctl_t RC_CtrlData;


//extern int16_t vx,vy,vr;
//extern uint8_t chassis_auto_ctrl_flag;
//extern int16_t chassis_auto_ctrl_vy;

//extern PoseMode posemod;
//extern PoseMode last_posemod;

//extern bool posemode_change_flag;
//extern bool chassis_moto_flag;


#endif
