/*
无用，新版车已用舵机替代
*/
#ifndef __MTMOTODRIVER_H__
#define __MTMOTODRIVER_H__
#include "Includes.h"

#define MT_MOTO_CAN_HANDEL hcan1


// 脉塔电机
typedef enum
{
    MT_Motor1_can_ID = 0x01,
    MT_Motor2_can_ID = 0x02,
    MT_Motor3_can_ID = 0x03,
    MT_Motor4_can_ID = 0x04,
    MT_Motor5_can_ID = 0x05,
} MT_CAN_ID;

// 脉塔电机控制模式
typedef enum
{
    MT_MIT_Mode = 0x000,
    MT_Position_Speed_Mode = 0x100,
    MT_Speed_Mode = 0x200,
    MT_EMIT_Mode = 0x300,
} MT_Mode_ID;

typedef struct
{
    // 电机反馈数据(真实值)
    int8_t temp;     // 温度(1℃/LSB)
    int16_t current; // 电流真实值(0.01A/LSB)
    int16_t speed;   // 转速真实值(1dps/LSB)
    int16_t angle;   // 角度真实值(1°/LSB,最大范围±32767)
    // 控制数据(目标值)
    float angle_desired;
    float speed_desired;
    float current_desired;

} MTMotoStateTD;

extern MTMotoStateTD MTMotoState[2];

void MT_SetMoto_Current(MT_CAN_ID motor_can_ID, int16_t current);
void MTSetMotoSpeed_single(MT_CAN_ID moto_id, int32_t speed);
void MTSetMotoPosition_single(MT_CAN_ID moto_id, uint16_t MaxSpeed, int32_t Angle);
void MT_SaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo);
void MTUpdateMotoState(MTMotoStateTD *state);

#endif
