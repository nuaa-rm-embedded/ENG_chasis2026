#include "relay.h"

uint8_t pump_flag = 0;
uint8_t t_pumg = 0;
uint8_t t_pumg_last = 0;

extern DataUnion sync_data_to_c; 

void arm_xipan_open()
{
	t_pumg_last = t_pumg;
	t_pumg = 1;
	sync_data_to_c.data.arm_valve = 1;
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
}

void arm_xipan_close()
{
	t_pumg_last = t_pumg;
	t_pumg = 0;
	sync_data_to_c.data.arm_valve = 0;
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
}


