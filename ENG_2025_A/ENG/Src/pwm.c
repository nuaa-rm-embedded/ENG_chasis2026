#include "pwm.h"

extern TIM_HandleTypeDef htim4;

uint16_t camera_pitch = 1450;
uint16_t camera_yaw = 2450;

void PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	
    CAMERA_PITCH = 1770;
    CAMERA_YAW   = 2450;
}

void Camera_Pitch_Set(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, camera_pitch);
}

void Camera_Yaw_Set(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, camera_yaw);
}