#ifndef __LIFT_MOTO_CTRL_H__
#define __LIFT_MOTO_CTRL_H__

#include "Includes.h"

#define LIFT_POS_P 0.15
#define LIFT_POS_I 0
#define LIFT_POS_D 0
#define LIFT_SPD_P 24
#define LIFT_SPD_I 0
#define LIFT_SPD_D 0

#define LIFT       (moto_gimbal[0].angle_desired)
#define LIFT_READ  (moto_gimbal[0].angle)
#define LIFT_MAX   -20000
#define LIFT_MIN   (-1150000)
#define LIFT_STD   (-150000)

//----------------------------------------------

#define CAMERA_LIFT_POS_P 0.15
#define CAMERA_LIFT_POS_I 0
#define CAMERA_LIFT_POS_D 0
#define CAMERA_LIFT_SPD_P 20
#define CAMERA_LIFT_SPD_I 0
#define CAMERA_LIFT_SPD_D 0

#define CAMERA_LIFT       (moto_gimbal[1].angle_desired)
#define CAMERA_LIFT_MAX   (400000)
#define CAMERA_LIFT_MIN   (10000)
#define CAMERA_LIFT_STD   (360000)

#define CAMERA_YAW_FORWARD 1
#define CAMERA_YAW_BACK 0

//----------------------------------------------

void arm_lift_reset(void);
void arm_lift_init(void);
void Arm_Lift_Ctrl(void);

void camera_lift_reset(void);
void camera_lift_init(void);
void Camera_Lift_Ctrl(void);

#endif
