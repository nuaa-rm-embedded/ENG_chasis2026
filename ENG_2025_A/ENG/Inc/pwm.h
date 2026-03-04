#ifndef __PWM_H
#define __PWM_H
#include "main.h"

#define CAMERA_YAW_MAX 2450
#define CAMERA_YAW_MIN 550
#define CAMERA_YAW_STD 1500

#define CAMERA_PITCH_MAX 2450
#define CAMERA_PITCH_MIN 550
#define CAMERA_PITCH_STD 1450


#define CAMERA_PITCH TIM4->CCR2
#define CAMERA_YAW TIM4->CCR1

void PWM_Start(void);
void Camera_Pitch_Set(void);
void Camera_Yaw_Set(void);


#endif
