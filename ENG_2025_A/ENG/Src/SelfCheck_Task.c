//V1.4版本。
/*
USB_NUC										 0 	//NUC，USB通信方式
CAN_M3508_LF							 1 	//左前轮M3508电机，CAN通信方式
CAN_M3508_RF							 2 	//右前轮M3508电机，CAN通信方式
CAN_M3508_LB							 3 	//左后轮M3508电机，CAN通信方式
CAN_M3508_RB							 4 	//右后轮M3508电机，CAN通信方式
CAN_M3508_FL							 5 	//机械臂抬升M3508电机，CAN通信方式
CAN_M3508_BL							 6 	//图传抬升M3508电机，CAN通信方式
CAN_RMD4015_yaw						 7 	//云台麦塔4015yaw，CAN通信方式
CAN_RMD4015_pitch					 8 	//云台麦塔4015pitch，CAN通信方式
UART_RC										 9 	//遥控器接收机，DBUS(UART)通信方式
UART_IMAGE								 10 //图传链路，UART通信方式
UART_ARM_C								 11 //底盘C板，UART通信方式
UART_REF									 12 //常规链路，UART通信方式
*/

#include "SelfCheck_Task.h"

uint16_t int_var1;
uint16_t int_var2;

extern uint8_t Break_While_ctrl;
extern uint8_t ShiJue_mode_ctrl;
uint8_t t_shijue_reset = 4;
device_state device[13];
uint16_t device_time[13] = {0};
uint8_t Eng_life=0;
uint8_t last_Eng_life=1;
extern uint8_t camera_lift_reset_flag;
extern uint8_t arm_lift_reset_flag;
extern uint16_t camera_pitch;
extern uint16_t camera_yaw;
uint16_t t_ts = 1;


void SelfCheck_mode(void const *argument)
{
	osDelay(350);
	while(1)
	{
		int_var1 = TIM4->CCR1;
		int_var2 = TIM4->CCR2;
		//卡住跳出
		if((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_Q))) Break_While_ctrl = 1;
		
		//各部位检测
		for(int i = 0;i<=12;i++)
		{
			device_time[i]++;
		}
		for(int i = 0;i<=12;i++)
		{
			if(device_time[i]>500) device[i] = DEVICE_DISORDER;
			else device[i] = DEVICE_NORMAL;
		}
		
		//死亡or复活判断
		if((device[CAN_M3508_LF]+device[CAN_M3508_RF]+device[CAN_M3508_LB]+device[CAN_M3508_RB])==0) 
		{
			last_Eng_life = Eng_life;
			Eng_life=0;
		}
		else
		{
			last_Eng_life = Eng_life;
			Eng_life=1;
		}
		
		//按CRTL+R复位
		if((Key_Check_Hold(&Keys.KEY_R))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			t_shijue_reset = 4;
			ShiJue_mode_ctrl = 2;
			vTaskDelay(500);
			__set_FAULTMASK(1);
			NVIC_SystemReset();
		}
		
		if((Key_Check_Hold(&Keys.KEY_W))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT) == 0))
		{
			camera_lift_reset_flag = true;
			arm_lift_reset_flag = true;
		}
		
		if((RC_CtrlData.mouse.press_r)&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			CAMERA_LIFT = CAMERA_LIFT - 600;
		}
		
		if((RC_CtrlData.mouse.press_l)&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			CAMERA_LIFT = CAMERA_LIFT + 600;
		}
		if(((Key_Check_Hold(&Keys.KEY_R))&&(Key_Check_Hold(&Keys.KEY_E)))||(t_ts))
		{
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);
			osDelay(200);
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
			t_ts = 0;
		}
		if((Key_Check_Hold(&Keys.KEY_D))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			int_var1++;
			if(int_var1>2450) int_var1 = 2450;
			if(int_var1<550) int_var1 = 550;
			CAMERA_YAW = int_var1;
		}
		if((Key_Check_Hold(&Keys.KEY_A))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			int_var1--;
			if(int_var1>2450) int_var1 = 2450;
			if(int_var1<550) int_var1 = 550;
			CAMERA_YAW = int_var1;
		}
		if((Key_Check_Hold(&Keys.KEY_S))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			int_var2++;
			if(int_var2>2450) int_var2 = 2450;
			if(int_var2<1500) int_var2 = 1500;
			CAMERA_PITCH = int_var2;
		}
		if((Key_Check_Hold(&Keys.KEY_W))&&(Key_Check_Hold(&Keys.KEY_CTRL))&&(Key_Check_Hold(&Keys.KEY_SHIFT)))
		{
			int_var2--;
			if(int_var2>2450) int_var2 = 2450;
			if(int_var2<1500) int_var2 = 1500;
			CAMERA_PITCH = int_var2;
		}
		osDelay(10);
	}
}
