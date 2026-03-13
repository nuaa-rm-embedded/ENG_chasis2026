#include "chassis.h"
#define track_speed_desire 10

PidTD pid_chassis_moto[6];
int16_t vx = 0,vy = 0,vr = 0;
int16_t chassis_auto_ctrl_vy = 0;
uint8_t chassis_auto_ctrl_flag = 0;
uint8_t x=1;
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
		dm_ctrl_init();
    while (1) {
        x++;
        chassis_control_RC();
        moto_chassis[4].speed_desired=track_speed_desire;
        moto_chassis[5].speed_desired=track_speed_desire;			
        for (int i = 0; i < 6; i++) {
            pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
        }
        // 电机输出 chassis_moto_flag为电机标志位，为零则全电机无力
        SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
				SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);

				/*
				此处会使得抬升电机电流为0，目前仅考虑底盘状态，之后考虑兼容抬升机构代码
				4，5为左右履带期望速度，目前考虑通过宏定义设置履带上的电机期望速度，还未进行3.4
				*/
				//Control_J4340(CAN_HandleTypeDef *hcan,uint32_t p1,uint32_t p2,uint32_t p3,uint32_t p4);//LF,RF,LB,RB,p1234之后再求
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
    for (int i = 0; i < 6; i++) {
        pidInit(&pid_chassis_moto[i], 0, 8000, KP_CHASSIS, KI_CHASSIS, KD_CHASSIS);
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
		moto_chassis[0].speed_desired = (vy + vx + vr) * chassis_speed_coefficient;  // 1  left front
		moto_chassis[1].speed_desired = (-vy + vx + vr) * chassis_speed_coefficient; // 2  right front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_coefficient;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_coefficient; // 4	right back
	}
	
	else if(Yaw_Flag == CAMERA_YAW_BACK)
	{
		vx = -vx;
		vy = -vy;
		
		moto_chassis[0].speed_desired = (vy + vx + vr) * chassis_speed_coefficient;  // 1  left front
		moto_chassis[1].speed_desired = (-vy + vx + vr) * chassis_speed_coefficient; // 2  right front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_coefficient;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_coefficient; // 4	right back
	}
	
	else if(Yaw_Flag == CAMERA_YAW_FORWARD && frame.data.mode == 2) 
	{
		moto_chassis[0].speed_desired = (vy + vx + vr) * chassis_speed_slow;  // 1  left front
		moto_chassis[1].speed_desired = (-vy + vx + vr) * chassis_speed_slow; // 2  right front
		moto_chassis[2].speed_desired = (vy - vx + vr) * chassis_speed_slow;  // 3	left back
		moto_chassis[3].speed_desired = (-vy - vx + vr) * chassis_speed_slow; // 4	right back
	}
}

//上台阶动作（伪代码）
void stair_work(){
	uint32_t p1,p2,p3,p4,v1,v2,v3,v4;
	v1=1;v2=1;v3=1;v4=1;//关节电机旋转速度限制
	p1=135;p2=135;
	p3=245;p4=245;//实际调好数值之后把常规姿态放入电机初始化过程中
	/*
	四个4340电机的转向角度，示意代码，以竖直向上为0度，一圈共计360度
	常规姿态，前腿接近180度，略微往90度倾斜，后两腿接近270度，略微往180度倾斜
	过程中履带，关节电机不动，
	*/
	/*
	此处加入常规速度的控制
	*/
	moto_chassis[4].speed_desired=0;
	moto_chassis[5].speed_desired=0;
	for (int i = 0; i < 6; i++) {
	pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
	}
	SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
	SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);
  Control_J4340(&hcan1,p1,p2,p3,p4,v1,v2,v3,v4);	
	/*
	靠近台阶后，前腿向225度接近，脱离地面，以履带与后轮作为机器人支撑点，后腿向195度接近，使机器人重心上升
	过程中后轮与履带电机转动，前轮电机不动
	*/
	p1=235;p2=235;
	p3=195;p4=195;
	moto_chassis[0].speed_desired=0;
	moto_chassis[1].speed_desired=0;
	moto_chassis[2].speed_desired=10;
	moto_chassis[3].speed_desired=10;//仅代表速度方向为车运动正方向，大小仅代表相对速度
	moto_chassis[4].speed_desired=10;
	moto_chassis[5].speed_desired=10;
	for (int i = 0; i < 6; i++) {
	pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
	}
	SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
	SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);
	Control_J4340(&hcan1,p1,p2,p3,p4,v1,v2,v3,v4);	
	/*
	机器人前腿碰到台阶后，前腿慢慢向270水平方向靠近，后腿往175度方向接近，再度上升重心，前支点慢慢由履带转变为前轮，
	过程中履带与前轮主要提供动力，后轮为辅佐
	*/
	p1=270;p2=270;
	p3=175;p4=175;
	moto_chassis[0].speed_desired=10;
	moto_chassis[1].speed_desired=10;
	moto_chassis[2].speed_desired=5;
	moto_chassis[3].speed_desired=5;
	moto_chassis[4].speed_desired=10;
	moto_chassis[5].speed_desired=10;
	for (int i = 0; i < 6; i++) {
	pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
	}
	SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
	SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);
	Control_J4340(&hcan1,p1,p2,p3,p4,v1,v2,v3,v4);	
	/*
	机器人完全登上第一段台阶后，后轮腿悬空，后腿向290度方向接近，后续上台阶过程中无参与，
	机器人前支点为前轮，后支点为履带，前进至履带接触第二级台阶
	履带大力转动，前轮作辅助，上第二级台阶，
	*/
	p1=255;p2=255;
	p3=290;p4=290;
	moto_chassis[0].speed_desired=10;
	moto_chassis[1].speed_desired=10;
	moto_chassis[2].speed_desired=0;
	moto_chassis[3].speed_desired=0;
	moto_chassis[4].speed_desired=15;
	moto_chassis[5].speed_desired=15;
	for (int i = 0; i < 6; i++) {
	pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
	}
	SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
	SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);
	Control_J4340(&hcan1,p1,p2,p3,p4,v1,v2,v3,v4);
	//台阶上完，恢复正常姿态
	p1=235;p2=235;
	p3=195;p4=195;
	//省略速度
	//......
	/*
	可精简代码
		for (int i = 0; i < 6; i++) {
	pid_calculate(&pid_chassis_moto[i], moto_chassis[i].speed_desired, moto_chassis[i].speed_actual);
	}
	SetMotoCurrent(&hcan2, Ahead,pid_chassis_moto[0].outPID,pid_chassis_moto[1].outPID,pid_chassis_moto[2].outPID,pid_chassis_moto[3].outPID);
	SetMotoCurrent(&hcan2, Back,pid_chassis_moto[4].outPID,pid_chassis_moto[5].outPID,0,0);
	*/
}
