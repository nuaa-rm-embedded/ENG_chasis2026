#include "Arm_Whole_Ctrl.h"

extern frame_t frame;

// 自定义控制器控制
void Custom_Robot_Ctrl(void)
{
    YAW1            = custom_controller_data_t.yaw1;
    PITCH1          = custom_controller_data_t.pitch1;
    YAW2            = custom_controller_data_t.yaw2;
    ROLL1           = custom_controller_data_t.roll1;
    PITCH2          = custom_controller_data_t.pitch2;
//    LIFT -= RC_CtrlData.rc.ch2 * 15;
	
	if(custom_controller_data_t.lift_flag == 2) LIFT -= 4000;
	if(custom_controller_data_t.lift_flag == 1) LIFT += 4000;
	
	if(custom_controller_data_t.roll2_flag == 1) ROLL2 += 1;
	if(custom_controller_data_t.roll2_flag == 2) ROLL2 -= 1;
	
	
	if(custom_controller_data_t.pump_flag == 1 && frame.data.mode == 2) 
	{
		arm_xipan_open();
	}
	else if(custom_controller_data_t.pump_flag == 1 && frame.data.mode == 1) arm_xipan_open();
	else if(custom_controller_data_t.pump_flag == 0 && frame.data.mode == 1) arm_xipan_close();
	
	frame.data.mode = 1;
}

// 机械臂控自定义控制器
void Robot_Custom_Ctrl(void)
{
    frame.data.mode = 2;
	
}

void KeyArm_Ctrl(void)
{
    if (Key_Check_Hold(&Keys.KEY_Q)) LIFT += 3000;
    if (Key_Check_Hold(&Keys.KEY_A)) LIFT -= 3000;

    if (Key_Check_Hold(&Keys.KEY_W)) YAW1 += 0.4;
    if (Key_Check_Hold(&Keys.KEY_S)) YAW1 -= 0.4;

    if (Key_Check_Hold(&Keys.KEY_E)) PITCH1 += 0.4;
    if (Key_Check_Hold(&Keys.KEY_D)) PITCH1 -= 0.4;

    if (Key_Check_Hold(&Keys.KEY_R)) YAW2 += 0.4;
    if (Key_Check_Hold(&Keys.KEY_F)) YAW2 -= 0.4;

    if (Key_Check_Hold(&Keys.KEY_Z)) ROLL1 += 0.4;
    if (Key_Check_Hold(&Keys.KEY_X)) ROLL1 -= 0.4;

//    if (Key_Check_Hold(&Keys.KEY_C)) PITCH1 += 0.4;
//    if (Key_Check_Hold(&Keys.KEY_C)) PITCH1 -= 0.4;
}

