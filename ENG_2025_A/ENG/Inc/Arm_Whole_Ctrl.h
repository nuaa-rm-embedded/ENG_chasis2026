#ifndef __ARM_WHOLE_CTRL_H__
#define __ARM_WHOLE_CTRL_H__

#include "Includes.h"
#include "arm_math.h"

#define YAW1   (sync_data_to_c.data.yaw1)
#define PITCH1 (sync_data_to_c.data.pitch1)
#define YAW2   (sync_data_to_c.data.yaw2)
#define ROLL1  (sync_data_to_c.data.roll1)
#define PITCH2 (sync_data_to_c.data.pitch2)
#define ROLL2  (sync_data_to_c.data.roll2)


#define YAW1_READ   (sync_data_from_c.data.yaw1)
#define PITCH1_READ (sync_data_from_c.data.pitch1)
#define YAW2_READ   (sync_data_from_c.data.yaw2)
#define ROLL1_READ  (sync_data_from_c.data.roll1)
#define PITCH2_READ (sync_data_from_c.data.pitch2)
#define ROLL2_READ  (sync_data_from_c.data.roll2)


void Custom_Robot_Ctrl(void);
void Robot_Custom_Ctrl(void);

#endif
