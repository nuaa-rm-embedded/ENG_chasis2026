#include "chassis.h"


PidTD pid_chassis_moto[6];
int16_t vx = 0,vy = 0,vr = 0;
int16_t chassis_auto_ctrl_vy = 0;
uint8_t chassis_auto_ctrl_flag = 0;

extern bool lift_inited;
extern bool camera_lift_inited;
extern PidTD camera_pid_lift_reset_spd;
extern int16_t lift_moto_current_set[2];
extern frame_t frame;

int16_t chassis_speed_coefficient = 1;
float chassis_speed_slow = 0.6f;

void MotoTask(void const *argument)
{
    chassis_pid_init();
    while (1) {

        chassis_control_RC();

        for (int i = 0; i < 4; i++) {
            pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
        }
        // 电机输出 chassis_moto_flag为电机标志位，为零则全电机无力
        SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
        osDelay(1);

//        if (lift_inited == false && camera_lift_inited == true) {
//            Camera_Lift_Ctrl();
//            SetMotoCurrent(&hcan1, Back, 0, lift_moto_current_set[1], 0, 0);
//            osDelay(1);
//        }
    }
}

void chassis_pid_init(void)
{
    for (int i = 0; i < 4; i++) {
        pidInit(&pid_chassis_moto[i], 0, 8000, KP_CHASSIS, KI_CHASSIS, KD_CHASSIS);
    }
}

void gimbal_pid_init(void)
{
    for (int i = 4; i < 6; i++) {
        pidInit(&pid_chassis_moto[i], 10000, 16384, KP_GIMBAL, KI_GIMBAL, KD_GIMBAL);
    }
}

extern int32_t Yaw_Flag;

// 遥控器控制
void chassis_control_RC(void)
{
    float norm = 0;
    arm_sqrt_f32(RC_CtrlData.rc.ch3 * RC_CtrlData.rc.ch3 + RC_CtrlData.rc.ch4 * RC_CtrlData.rc.ch4, &norm);
    if (norm > 650.0f) norm = 650.0f;
    norm /= 650.0f;
    vx = (float)RC_CtrlData.rc.ch3 * norm * MAX_MOVE_RMP / 650.0f; // 平移分量
    if (!chassis_auto_ctrl_flag) {
        vy = (float)RC_CtrlData.rc.ch4 * norm * MAX_MOVE_RMP / 650.0f; // 前进分量
    }
    vr = (float)RC_CtrlData.rc.ch1 * MAX_ROTATE_RMP / 650.0f;
    if (RC_CtrlData.mouse.x != 0) {
        vr = RC_CtrlData.mouse.x * MOUSE_ROTATE_PARA;
        if (abs(vr) > MOUSE_ROTATE_RMP)
            vr = MOUSE_ROTATE_RMP * (vr < 0 ? -1 : 1);
    }
	
	
    // 解算出各个轮子目标速度
//	if(RC_CtrlData.rc.sw1 == 3) chassis_speed_coefficient = 1;
//	else if(RC_CtrlData.rc.sw1 == 3) chassis_speed_coefficient = 1;
//	else if(RC_CtrlData.rc.sw1 == 3) chassis_speed_coefficient = 1;
	
	if(Yaw_Flag == CAMERA_YAW_FORWARD && frame.data.mode != 2)
	{
		moto_chassis[0].speed_desired = (-vy + vx + vr) * chassis_speed_coefficient; // 1  right front
		moto_chassis[1].speed_desired = (vy + vx + vr) * chassis_speed_coefficient;  // 2  left front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_coefficient;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_coefficient; // 4	right back
	}
	
	else if(Yaw_Flag == CAMERA_YAW_BACK)
	{
		vx = -vx;
		vy = -vy;
		
		moto_chassis[0].speed_desired = (-vy + vx + vr) * chassis_speed_coefficient; // 1  right front
		moto_chassis[1].speed_desired = (vy + vx + vr) * chassis_speed_coefficient;  // 2  left front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_coefficient;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_coefficient; // 4	right back
	}
	
	else if(Yaw_Flag == CAMERA_YAW_FORWARD && frame.data.mode == 2) 
	{
		moto_chassis[0].speed_desired = (-vy + vx + vr) * chassis_speed_slow; // 1  right front
		moto_chassis[1].speed_desired = (vy + vx + vr) * chassis_speed_slow;  // 2  left front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_slow;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_slow; // 4	right back
	}
}


