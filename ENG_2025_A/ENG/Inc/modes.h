#ifndef __MODES_H
#define __MODES_H

#include "Includes.h"


#define LIFT_SILVER_UP (-450000) // 预取银矿位置
#define LIFT_SILVER_DOWN (LIFT_SILVER_UP + 250000) // 吸银矿

#define LIFT_ERR 50000       //机械臂抬升检测误差
#define DM_MOTO_ERR 5.0f   //机械臂达妙电机检测误差

#define LIFT_ERR_1 20000       //视觉兑矿机械臂抬升检测误差
#define DM_MOTO_ERR_1 2.0f   //视觉兑矿机械臂达妙电机检测误差

#define Break_While if(Break_While_ctrl){Break_While_ctrl = 0;break;}


typedef enum{
	NONE,
	EXCHANGE,
}PoseMode;

void None_Loop(void);
void AutoEngineer_Loop(void);
void EXCHANGE_Loop(void);
void DEBUG_Loop(void);


void Before_Fetch_Right_Silver_Loop(void);
void Before_Fetch_Left_Silver_Loop(void);

void Fetch_Silver_To_Storage_Loop(void);
void Fetch_Silver_OnTheOther_Silver_Loop(void);

void Before_Exchange_Above_Silver_Loop(void);
void Before_Exchange_Below_Silver_Loop(void);
void Before_Exchange_Below_Glod_Loop(void);

void Before_Fetch_Ground_Loop(void);
void Fetch_Ground_Loop(void);

void Before_Fetch_Gold_Loop(void);
void Fetch_Gold_To_Storage_Loop(void);
void Fetch_Gold_To_DogHole_Gesture_Loop(void);
void To_DogHole_Gesture_Loop(void);

void ShiJue_mode(void);

#endif

