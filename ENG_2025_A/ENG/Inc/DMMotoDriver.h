#ifndef __DMMOTODRIVER_H__
#define __DMMOTODRIVER_H__
#include "main.h"
#include "can.h"
#include "pid.h"
#include "cmsis_os.h"
#include "math_user.h"
//#include "VOFA.h"
#include "arm_math.h"

#define DM_MOTO_CAN_HANDEL hcan1
#define PMAX 12.5f // 电机位置最大值(正负)
#define VMAX 10.0f // 电机速度最大值(正负),现在全部改成10了
#define TMAX 10.0f // 电机扭矩最大值(正负),现在全部改成10了

// 达妙电机can_ID(控制帧)
typedef enum
{
    DM_Motor1_can_ID = 0x01,//LF
    DM_Motor2_can_ID = 0x02,//RF
    DM_Motor3_can_ID = 0x03,//LB
    DM_Motor4_can_ID = 0x04,//RB
} DM_CAN_Ctrl_ID;

// 达妙电机master_ID(反馈帧)
typedef enum
{
    DM_Motor1_master_ID = 0x01,//后续限位时+0x10,以辨别
    DM_Motor2_master_ID = 0x02,
    DM_Motor3_master_ID = 0x03,
    DM_Motor4_master_ID = 0x04,

} DM_CAN_Msg_ID;

// 达妙电机控制模式(电机使能,失能等都要用到)
typedef enum
{
	DM_MIT_Mode = 0x000,
    DM_Position_Speed_Mode = 0x100,
    DM_Speed_Mode = 0x200,
    DM_EMIT_Mode = 0x300,
} DM_Mode_ID;


typedef struct
{
    // 电机反馈数据(真实值)
    int err; // 电机当前状态(0:失能 1:使能 8:超压 9:欠压)
    int angle_int;
    int speed_int;
    int current_int;
    float angle;   // 位置
    float speed;   // 速度
    float current; // 电流(扭矩)
    float Tmos;    // 驱动板温度
    float Tcoil;   // 电机线圈温度
    // 控制数据(目标值)
    float angle_desired;
    float speed_desired;
    float current_desired;

} DMMotoStateTD;

void DMSetMoto_speed_and_position_single(DM_CAN_Ctrl_ID motor_can_ID, float angle, float max_speed);
void DMSetMoto_speed_single(DM_CAN_Ctrl_ID motor_can_ID, float speed);
void DMSetMoto_MIT_single(DM_CAN_Ctrl_ID motor_can_ID, float pos, float vel,float kp, float kd, float tor);

void DM_Enable(DM_CAN_Ctrl_ID motor_can_ID,DM_Mode_ID Mode);
void DM_Disable(DM_CAN_Ctrl_ID motor_can_ID,DM_Mode_ID Mode);

void DM_SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo);
void DMUpdateMotoState(DMMotoStateTD *state);
void dm_ctrl_init();
#endif
