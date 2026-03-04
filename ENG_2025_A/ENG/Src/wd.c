#include "wd.h"

bool arm_lift_reset_flag = false;// 机械臂抬升微动

bool camera_lift_reset_flag = false;// 云台相机抬升微动


void wd_init(void)
{
    if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0) == GPIO_PIN_SET) arm_lift_reset_flag = true;
    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) == GPIO_PIN_SET) camera_lift_reset_flag = true;
	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == GPIO_PIN_0) {
        // 检测到上升沿
        if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0) == GPIO_PIN_SET) {
            arm_lift_reset_flag = true;
        }
        // 检测到下降沿
        else if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0) == GPIO_PIN_RESET) {
            arm_lift_reset_flag = false;
        }
    }
	
    if (GPIO_Pin == GPIO_PIN_12) {
        // 检测到上升沿
        if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) == GPIO_PIN_SET) {
             camera_lift_reset_flag = true;
        }
        // 检测到下降沿
        else if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) == GPIO_PIN_RESET) {
            camera_lift_reset_flag = false;
        }
    }
	
}

