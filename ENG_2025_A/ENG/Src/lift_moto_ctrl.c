#include "lift_moto_ctrl.h"

//---------------------------------机械臂lift抬升----------------------------------------

// 上电复位的pid参数
#define LIFT_RESET_SPD_P 15
#define LIFT_RESET_SPD_I 0
#define LIFT_RESET_SPD_D 0

// 初始化定位时的速度环
PidTD pid_lift_reset_spd;

// 正常工作速度环和位置环
PidTD pid_lift_spd;
PidTD pid_lift_pos;

// 上电复位有关参数
bool lift_inited = false;
extern bool arm_lift_reset_flag;

// 两个抬升机构给的电流
int16_t lift_moto_current_set[2];

// 复位后(正常工作时)的参数给定(pid,电机状态参数)
void arm_lift_init(void)
{
    pidInit(&pid_lift_pos, 0, 10000, LIFT_POS_P, LIFT_POS_I, LIFT_POS_D);
    pidInit(&pid_lift_spd, 0, 6000, LIFT_SPD_P, LIFT_SPD_I, LIFT_SPD_D);
    MotoStateInit(&moto_gimbal[0]);
}

// 复位时的参数给定(pid,电机状态参数)
void lift_init_reset_stage()
{
    pidInit(&pid_lift_reset_spd, 3000, 10000, LIFT_RESET_SPD_P, LIFT_RESET_SPD_I, LIFT_RESET_SPD_D);
    MotoStateInit(&moto_gimbal[0]);
}

// 抬升机构上电自动复位
void arm_lift_reset(void)
{
    osDelay(500);

    float speed = 3000;
    lift_init_reset_stage();
    while (lift_inited == false) {          // 抬升机构上电自动定位过程
        if (arm_lift_reset_flag == false) { // 还未触碰微动开关时
            // 速度环
            pid_calculate(&pid_lift_reset_spd, speed, moto_gimbal[0].speed_actual);
            SetMotoCurrent(&hcan2, Back, (pid_lift_reset_spd.outPID), 0, 0, 0);
        } else { // 已触碰微动开关后,给定此时角度值为0
            SetMotoCurrent(&hcan2, Back, 0, 0, 0, 0);
            osDelay(20);
            arm_lift_init(); // 重置电机pid参数,初始化电机状态参数(如角度等)
            lift_inited = true;
        }
        osDelay(1);
    }
    moto_gimbal[0].angle_desired = LIFT_STD; // 回到标准位置
}

// 机械臂lift抬升复位后的控制函数
bool arm_lift_power_less_flag = false;

void Arm_Lift_Ctrl(void)
{
    // 限位保护
    if (LIFT >= LIFT_MAX) LIFT = LIFT_MAX;
    if (LIFT <= LIFT_MIN) LIFT = LIFT_MIN;

    pid_calculate(&pid_lift_pos, (float)moto_gimbal[0].angle_desired, (float)moto_gimbal[0].angle);
    moto_gimbal[0].speed_desired = (int)pid_lift_pos.outPID;
    pid_calculate(&pid_lift_spd, moto_gimbal[0].speed_desired, moto_gimbal[0].speed_actual);

    lift_moto_current_set[0] = pid_lift_spd.outPID;
		if(lift_moto_current_set[0]>8000)	lift_moto_current_set[0] = 8000;
		if(lift_moto_current_set[0]<-8000)	lift_moto_current_set[0] = -8000;
    //    SetMotoCurrent(&hcan1, Back, pid_lift_spd.outPID, 0, 0, 0);
}

//---------------------------------机械臂lift抬升----------------------------------------

//---------------------------------云台相机lift抬升----------------------------------------

// 上电复位的pid参数
#define CAMERA_LIFT_RESET_SPD_P 20
#define CAMERA_LIFT_RESET_SPD_I 0
#define CAMERA_LIFT_RESET_SPD_D 0

// 初始化定位时的速度环
PidTD camera_pid_lift_reset_spd;
// 正常工作速度环和位置环
PidTD camera_pid_lift_spd;
PidTD camera_pid_lift_pos;

bool camera_lift_inited = false;
extern bool camera_lift_reset_flag;

// 复位后(正常工作时)的参数给定(pid,电机状态参数)
void camera_lift_init()
{
    pidInit(&camera_pid_lift_pos, 0, 5000, CAMERA_LIFT_POS_P, CAMERA_LIFT_POS_I, CAMERA_LIFT_POS_D);
    pidInit(&camera_pid_lift_spd, 0, 5000, CAMERA_LIFT_SPD_P, CAMERA_LIFT_SPD_I, CAMERA_LIFT_SPD_D);
    MotoStateInit(&moto_gimbal[1]);
}

// 复位时的参数给定(pid,电机状态参数)
void camera_lift_init_reset_stage()
{
    pidInit(&camera_pid_lift_reset_spd, 0, 10000, CAMERA_LIFT_RESET_SPD_P, CAMERA_LIFT_RESET_SPD_I, CAMERA_LIFT_RESET_SPD_D);
    MotoStateInit(&moto_gimbal[1]);
}

// 抬升机构上电自动复位
void camera_lift_reset(void)
{
    float speed = -1500;
    camera_lift_init_reset_stage();
    while (camera_lift_inited == false) {      // 抬升机构上电自动定位过程
        if (camera_lift_reset_flag == false) { // 还未触碰微动开关时
            // 速度环
            moto_gimbal[1].speed_desired = speed;
            pid_calculate(&camera_pid_lift_reset_spd, moto_gimbal[1].speed_desired, moto_gimbal[1].speed_actual);
            SetMotoCurrent(&hcan1, Back, 0, camera_pid_lift_reset_spd.outPID, 0, 0);
        } else { // 已触碰微动开关后,给定此时角度值为0
            SetMotoCurrent(&hcan1, Back, 0, 0, 0, 0);
            osDelay(20);
            camera_lift_init(); // 重置电机pid参数,初始化电机状态参数(如角度等)
            camera_lift_inited = true;
        }
        osDelay(1);
    }
    moto_gimbal[1].angle_desired = CAMERA_LIFT_STD; // 回到标准位置
}

// 机械臂lift抬升复位后的控制函数
bool camera_lift_power_less_flag = false;

void Camera_Lift_Ctrl(void)
{
    // 限位保护
    if (CAMERA_LIFT >= CAMERA_LIFT_MAX) CAMERA_LIFT = CAMERA_LIFT_MAX;
    if (CAMERA_LIFT <= CAMERA_LIFT_MIN) CAMERA_LIFT = CAMERA_LIFT_MIN;

    pid_calculate(&camera_pid_lift_pos, (float)moto_gimbal[1].angle_desired, (float)moto_gimbal[1].angle);
    moto_gimbal[1].speed_desired = (int)camera_pid_lift_pos.outPID;
    pid_calculate(&camera_pid_lift_spd, moto_gimbal[1].speed_desired, moto_gimbal[1].speed_actual);

    lift_moto_current_set[1] = camera_pid_lift_spd.outPID;
		if(lift_moto_current_set[1]>8000)	lift_moto_current_set[1] = 8000;
		if(lift_moto_current_set[1]<-8000)	lift_moto_current_set[1] = -8000;
}

//---------------------------------云台相机lift抬升----------------------------------------
