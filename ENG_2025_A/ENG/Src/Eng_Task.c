#include "Eng_Task.h"

extern frame_t frame;
extern uint8_t Break_While_ctrl;
extern uint8_t ShiJue_mode_ctrl;
extern nuc_receive_data_t	nuc_receive_data;
extern int action_Num;

// AC板串口通信
void TransmitTask(void const *argument)
{
    usart_dma_init();
    sync_data_to_c_init();
    Robot_Custom_Data_Transmit_Init();
    while (1) {

        // 发送数据到C板 串口6
        HAL_UART_Transmit_DMA(&huart6, sync_data_to_c.bytes, SYNC_TO_C_SIZE);
        osDelay(17);

        // 通过图传链路给自定义控制器
        Robot_Custom_Data_Transmit();
        osDelay(17);
    }
}

void usb_task(void *argument)
{
//	cmd_id_init();
//	cmd_id_task_create(0x14,999);
	while(1)
	{
//		Nuc_data_unpacked();
//		if(ShiJue_mode_ctrl) 
//		{
//			memset(&nuc_receive_data, 0, sizeof(nuc_receive_data_t));
//			action_Num = 0;
//		}
//		cmd_id_queue_handle();
		osDelay(100);
	}
}

extern bool lift_inited;
extern bool camera_lift_inited;
extern int16_t lift_moto_current_set[2];

uint8_t lift_power_less_flag = 0;
int32_t Yaw_Flag = 1;

extern uint8_t Eng_life;
extern uint8_t last_Eng_life;

// lift电机控制任务
void LiftTask(void const *argument)
{
    osDelay(1000);
    wd_init();
    camera_lift_reset();
    osDelay(2);
    arm_lift_reset();
    for (;;) {
            Arm_Lift_Ctrl();
            Camera_Lift_Ctrl();
            SetMotoCurrent(&hcan2, Back, lift_moto_current_set[0], 0, 0, 0);
            osDelay(1);
            SetMotoCurrent(&hcan1, Back, 0, lift_moto_current_set[1], 0, 0);
            osDelay(1);
    }
}


//void Arm_Ctrl_Task(void *argument)
//{
//    for (;;) {

//        osDelay(10);
//    }
//}
