#ifndef __TIME_H__
#define __TIME_H__

#include "main.h"

#define SECOND				0		//秒
#define MILLISECOND		1		//毫秒
#define MICROSECOND		2		//微秒

typedef struct 
{
	uint32_t 	timer_cnt_now;
	uint32_t 	timer_cnt_last;
	float	 		timer_cnt_total;//总时间
	float			dt;//记录时间的单位,有us、ms、s
}TimeTD;

void TimeInit(TimeTD *time);
void GetDt(TimeTD *time, uint32_t time_unit);
#endif

