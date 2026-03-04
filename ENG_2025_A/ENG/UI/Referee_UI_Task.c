#include "Includes.h"

uint8_t t_UI = 1;
extern int ui_self_id;
extern robot_status_t robot_state;
extern uint8_t ShiJue_mode_ctrl;

extern uint8_t t_shijue;
extern uint8_t t_pumg;
extern uint8_t t_pumg_last;

void ui_task(void const* argument)
{
	ui_self_id = 2;
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_Ungroup();
	vTaskDelay(100);
	ui_init_g_Ungroup();
	vTaskDelay(100);
	ui_init_g_Ungroup();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_1();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_4();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_pumg();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_shijue();
	vTaskDelay(100);
	ui_init_g_close();
	vTaskDelay(100);
	ui_init_g_close();
	vTaskDelay(100);
	ui_init_g_close();
	vTaskDelay(100);
	ui_init_g_die();
	vTaskDelay(100);
	ui_init_g_die();
	vTaskDelay(100);
	ui_init_g_die();
	vTaskDelay(100);
	while(1)
	{			
		ui_init_g_pumg();
		vTaskDelay(100);
		ui_init_g_shijue();
		vTaskDelay(100);
		if((t_pumg_last == 0) && (t_pumg == 1))
		{
			ui_remove_g_close();
			vTaskDelay(100);
			ui_remove_g_close();
			vTaskDelay(100);
			ui_remove_g_close();
			vTaskDelay(100);
			ui_remove_g_close();
			vTaskDelay(100);
			ui_remove_g_close();
			vTaskDelay(100);
		}
		if((t_pumg_last == 1) && (t_pumg == 0))
		{
			ui_remove_g_open();
			vTaskDelay(100);
			ui_remove_g_open();
			vTaskDelay(100);
			ui_remove_g_open();
			vTaskDelay(100);
			ui_remove_g_open();
			vTaskDelay(100);
			ui_remove_g_open();
			vTaskDelay(100);
		}
		if(t_pumg == 1)
		{
			ui_init_g_open();
			vTaskDelay(100);
		}
		if(t_pumg == 0)
		{
			ui_init_g_close();
			vTaskDelay(100);
		}
		if(t_UI == 1)
		{
			ui_init_g_1();
			vTaskDelay(100);
			ui_init_g_4();
			vTaskDelay(100);
		}
		else if(t_UI == 2)
		{
			ui_init_g_2();
			vTaskDelay(100);
		}
	osDelay(1);
	}
}
