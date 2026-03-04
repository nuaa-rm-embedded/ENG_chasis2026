#ifndef __PID_H
#define __PID_H

#include "Includes.h"

/*
	必须提供符合以下条件的定时器：
		1. 频率1MHz
		2. 向上计数
		3. 重装载值0xFFFFFFFF
*/
#define PID_TIMER_HANDEL htim2

typedef struct Pid_Object
{
	float desired;      //< set point
  
	float error;        //< error
	float lastError;		//上次
	float prevError;    //上上次
  
	float integ;        //< integral积分
	float deriv;        //< derivative微分
  
	float kp;           //< proportional gain比例增益
	float ki;           //< integral gain积分增益
	float kd;           //< derivative gain微分增益
  
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
  float outPID;       //< pid total output
  
	float iLimit;       //< integral limit 积分输出限幅
	float iLimitLow;    //< integral limit -积分输出限幅
  
  float outLimit;      //< output limit 输出项限幅
  float outLimitLow;   //< output limit -输出项限幅
  
  TimeTD time;          //calculate dt
  bool first_cal;     //is first calculate pid 是否为首次进行pid计算，初始化为true
  
  void (*pid_calculate)(struct Pid_Object *pid,float desired,float measured); //< pid calculate
  
}PidTD;

void pidInit (PidTD* pid, 
							const float iLimit,
							const float outLimit,
							const float kp,
							const float ki,
							const float kd);
							
void pid_calculate(PidTD* pid,float desired,float measured);
void pid_calculate_inc(PidTD* pid,float desired,float measured);
void pidClearIntegral(PidTD *pid);

#endif
