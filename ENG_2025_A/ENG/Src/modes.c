#include "modes.h"

PoseMode posemod;
PoseMode last_posemod;
extern frame_t frame;
extern int action_Num;
extern uint8_t t_UI;
int set_error_roll = 15;
uint8_t t_r = 0;


// 空模式
void None_Loop(void)
{
    frame.data.mode = 1;
    ROLL2           = 0.0f;
}

// 手动兑矿模式
void EXCHANGE_Loop(void)
{
    Custom_Robot_Ctrl();
}

void DEBUG_Loop(void)
{
    /*
        调试模式
    */
}

extern uint16_t camera_pitch;
extern uint16_t camera_yaw;

extern int32_t Yaw_Flag;

uint8_t Spin_Flag = 0;

uint8_t Fetch_Two_Silver_ing = 0;
uint8_t Fetch_Left_Silver_ing = 0;
uint8_t Fetch_Right_Silver_ing = 0;
uint8_t Before_Exchange_Above_Silver_ing = 0;
uint8_t Before_Exchange_Below_Silver_ing = 0;
uint8_t Before_Exchange_Below_Glod_ing = 0;
uint8_t Fetch_Ground_ing = 0;
uint8_t Fetch_Gold_ing = 0;
uint8_t Gold_To_Storage_ing = 0;
uint8_t Gold_To_ing = 0;
uint8_t Gold_To_DogHole_ing = 0;
uint8_t Gold_To_DogHole_Back_ing = 0;
uint8_t To_DogHole_ing = 0;
uint8_t To_DogHole_Back_ing = 0;
uint8_t SHIJUE_mode_ing = 0;
uint8_t ShiJue_mode_ctrl = 2;
uint8_t Break_While_ctrl = 0;
uint8_t SHIJUE_mode_arm = 0;
uint8_t xipan_debug_flag = 0;

void ModeTask(void const *argument)
{
    osDelay(500);
    Key_Init();
    for (;;) {
		
		if((RC_CtrlData.rc.sw2 == 3)&&(t_r == 0))
		{
			t_r = 1;
			YAW1   = -90.0f;
			YAW2   = -30.0f;
			ROLL1  = -145.0f;
			PITCH2 = 90.0f;
			ROLL2 += 0;			
			PITCH1 = -7.0f;
			LIFT = -100000;
		}

        if (RC_CtrlData.rc.sw1 == 3) 
		{
			Spin_Flag = 0;
            Robot_Custom_Ctrl();
			sync_data_to_c.data.arm_camera = 0;
			CAMERA_PITCH = 1650;

			if(Key_Check_Hold(&Keys.KEY_CTRL) && (RC_CtrlData.mouse.press_l == 1))
			{
				osDelay(20);
				LIFT = LIFT_MIN;				
				YAW1 = 30;
				PITCH1 = 5;
				YAW2 = 30;
				ROLL1 = -180;
				PITCH2 = 60;
//				while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
//					osDelay(1);
//					Break_While
//				}
				ROLL2 -= 90;
				osDelay(400);				
			}
		
			if(Key_Check_Hold(&Keys.KEY_CTRL) && (RC_CtrlData.mouse.press_r == 1))
			{
				osDelay(20);
				LIFT = LIFT_MIN;
				PITCH2 = 60;
				YAW1 = -30;
				PITCH1 = 5;
				YAW2 = 150;
				ROLL1 = 0;
//				while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
//					osDelay(1);
//					Break_While
//				}
				ROLL2 += 90;
				osDelay(400);
			}
			
        }

        else if (RC_CtrlData.rc.sw1 == 1) 
		{
			Spin_Flag = 0;
			if(RC_CtrlData.rc.sw2 == 1) 
			{
				sync_data_to_c.data.arm_camera = 0;
				CAMERA_PITCH = 1770;
			}
			if(RC_CtrlData.rc.sw2 == 3) 
			{
				sync_data_to_c.data.arm_camera = 0;
				CAMERA_PITCH = 1550;
			}
			else if(RC_CtrlData.rc.sw2 == 2) 
			{
				sync_data_to_c.data.arm_camera = 1;
				CAMERA_PITCH = 2450;
			}
            Custom_Robot_Ctrl();
        }

        else
		{
			if(!Spin_Flag)
			{
				CAMERA_PITCH = 1770;
				Spin_Flag = 1;
			}
			sync_data_to_c.data.arm_camera = 0;
            frame.data.mode = 1;
		}

		if((RC_CtrlData.rc.sw1 != 1)&&(RC_CtrlData.rc.sw1 != 0))
		{
			if (RC_CtrlData.rc.sw2 == 1) arm_xipan_open();
			if (RC_CtrlData.rc.sw2_last == 1 && RC_CtrlData.rc.sw2 == 3 && xipan_debug_flag == 0) 
			{
				arm_xipan_close();
				xipan_debug_flag = 1;
			}
		}
		
		

		//###################################################取银矿###################################################
		
        // 按下X,进入一键双银模式
        if (Key_Check_Hold(&Keys.KEY_X) || Fetch_Two_Silver_ing == 1) {
            Fetch_Two_Silver_ing = 1;
            Before_Fetch_Right_Silver_Loop();

            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
            Fetch_Silver_To_Storage_Loop();
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			Before_Fetch_Left_Silver_Loop();
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			Fetch_Silver_OnTheOther_Silver_Loop();
			Fetch_Two_Silver_ing = 0;
        }
		
        // 按下Z,进入一键左银模式
        if ((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_Z)) || Fetch_Left_Silver_ing == 1) {
            Fetch_Left_Silver_ing = 1;
			
            Before_Fetch_Left_Silver_Loop();

            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
            Fetch_Silver_OnTheOther_Silver_Loop();
			
			Fetch_Left_Silver_ing = 0;
        }
		
        // 按下C,进入一键右银模式
        if ((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_C)) || Fetch_Right_Silver_ing == 1) {
            Fetch_Right_Silver_ing = 1;
			
            Before_Fetch_Right_Silver_Loop();

            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
            Fetch_Silver_OnTheOther_Silver_Loop();
			
			Fetch_Right_Silver_ing = 0;
        }
		//###################################################取银矿###################################################

		
		//###################################################取金矿###################################################
		// 按下A,进入一键金模式
        if (Key_Check_Hold(&Keys.KEY_A) && (Key_Check_Hold(&Keys.KEY_CTRL) == 0)|| Gold_To_ing == 1) {
            Gold_To_ing = 1;
            Before_Fetch_Gold_Loop();
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			LIFT = LIFT - 50000;
			PITCH1 = 3.5f;
			PITCH2 = -3.3f;
			osDelay(500);
			
            while (RC_CtrlData.mouse.press_l != 1) {
				LIFT -= RC_CtrlData.rc.ch2;
                osDelay(1);
            }
			
			Gold_To_ing = 0;
        }
				
		// 按下S,将矿放入矿仓
		if(Key_Check_Hold(&Keys.KEY_S) && (Key_Check_Hold(&Keys.KEY_CTRL) == 0)|| Gold_To_Storage_ing == 1){
              Gold_To_Storage_ing = 1;
		
			  Fetch_Gold_To_Storage_Loop();
		
		      Gold_To_Storage_ing = 0;

		}
		
		// 按下D,带矿过狗洞姿态
		if((Key_Check_Hold(&Keys.KEY_D) && Key_Check_Hold(&Keys.KEY_CTRL) == 0) || Gold_To_DogHole_ing == 1){
            Gold_To_DogHole_ing = 1;
			
		
			Fetch_Gold_To_DogHole_Gesture_Loop();
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			CAMERA_LIFT = CAMERA_LIFT_MIN;			
			CAMERA_PITCH = 1450;
			osDelay(500);
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			CAMERA_PITCH = 1450 + 25/90*950;
			CAMERA_LIFT = CAMERA_LIFT_STD;
		    Gold_To_DogHole_ing = 0;
			Spin_Flag = 0;
		}
		
		// 按下F,带矿倒过狗洞姿态
		if((Key_Check_Hold(&Keys.KEY_F) && Key_Check_Hold(&Keys.KEY_CTRL) == 0) || Gold_To_DogHole_Back_ing == 1){
            Gold_To_DogHole_Back_ing = 1;
			
			Fetch_Gold_To_DogHole_Gesture_Loop();
			
			CAMERA_LIFT = CAMERA_LIFT_MIN;
			CAMERA_PITCH = 1450;
			
			Yaw_Flag = 0;
			CAMERA_YAW = CAMERA_YAW_MIN;
			
			t_UI = 2;
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			CAMERA_PITCH = 1450 + 25/90*950;
			
			Yaw_Flag = 1;
			CAMERA_YAW = CAMERA_YAW_MAX;
			
			CAMERA_LIFT = CAMERA_LIFT_STD;
			t_UI = 1;
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
		    Gold_To_DogHole_Back_ing = 0;
			Spin_Flag = 0;

		}
		
		// 按下D,不带矿过狗洞姿态
		if((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_D) && (Key_Check_Hold(&Keys.KEY_SHIFT) == 0 )) || To_DogHole_ing == 1){
            To_DogHole_ing = 1; 
			
		
			To_DogHole_Gesture_Loop();
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			CAMERA_LIFT = CAMERA_LIFT_MIN;
			
			CAMERA_PITCH = 1450;
			
			osDelay(500);
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			CAMERA_PITCH = 1450;
			
			CAMERA_LIFT = CAMERA_LIFT_STD;
		    To_DogHole_ing = 0;
			Spin_Flag = 0;

		}
		
		// 按下F,不带矿倒过狗洞姿态
		if((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_F)) || To_DogHole_Back_ing == 1){
            To_DogHole_Back_ing = 1;
			
			To_DogHole_Gesture_Loop();
			
			CAMERA_LIFT = CAMERA_LIFT_MIN;
			
			CAMERA_PITCH = 1450;
			
			Yaw_Flag = 0;
			CAMERA_YAW = CAMERA_YAW_MIN;
			
			t_UI = 2;
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_1();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_4();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			ui_remove_g_Ungroup();
			vTaskDelay(10);
			
            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
			
			CAMERA_PITCH = 1450 + 25/90*950;
			
			Yaw_Flag = 1;
			CAMERA_YAW = CAMERA_YAW_MAX;
			
			CAMERA_LIFT = CAMERA_LIFT_STD;
			t_UI = 1;
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
			ui_remove_g_2();
			vTaskDelay(10);
		    To_DogHole_Back_ing = 0;
			Spin_Flag = 0;
		}
		
		
		
		//###################################################取金矿###################################################

				
		//###################################################取地矿###################################################
		
        // 按下G,进入地矿模式
        if (Key_Check_Hold(&Keys.KEY_G) || Fetch_Ground_ing == 1) {
            Fetch_Ground_ing = 1;
			
            Before_Fetch_Ground_Loop();

            while (RC_CtrlData.mouse.press_l != 1) {
                osDelay(1);
            }
            Fetch_Ground_Loop();
			
            while (RC_CtrlData.mouse.press_l != 1) {
				ROLL2 += RC_CtrlData.rc.ch2 * 0.001f;
                osDelay(1);
            }
			
			Fetch_Ground_ing = 0;
        }
		
		//###################################################取地矿###################################################
		
		
		
		//###################################################兑银矿###################################################

		// 按下V,准备兑上银矿
        if (Key_Check_Hold(&Keys.KEY_V) || Before_Exchange_Above_Silver_ing == 1) {
            Before_Exchange_Above_Silver_ing = 1;
			
			Before_Exchange_Above_Silver_Loop();
			
			Before_Exchange_Above_Silver_ing = 0;
        }
		
		// 按下B,准备兑下银矿
        if ((Key_Check_Hold(&Keys.KEY_CTRL) == 0 && Key_Check_Hold(&Keys.KEY_B)) || Before_Exchange_Below_Silver_ing == 1) {
            Before_Exchange_Below_Silver_ing = 1;
			
			Before_Exchange_Below_Silver_Loop();
			
			Before_Exchange_Below_Silver_ing = 0;
        }
		
		// 按下ctrl+B,准备兑下金矿
        if ((Key_Check_Hold(&Keys.KEY_CTRL) && Key_Check_Hold(&Keys.KEY_B)) || Before_Exchange_Below_Glod_ing == 1) {
            Before_Exchange_Below_Glod_ing = 1;
			
			Before_Exchange_Below_Glod_Loop();
			
			Before_Exchange_Below_Glod_ing = 0;
        }
		
		//###################################################兑银矿###################################################

				
				
		//##################################################视觉兑矿##################################################
		
        // 按下ctrl+e,进入视觉兑矿
		//再见了dingsang
		
		//##################################################视觉兑矿##################################################
		
        osDelay(10);
    }
}

// ##################################################模式过程函数######################################################

// ########################################银矿########################################


// 取右矿,机械臂准备
void Before_Fetch_Right_Silver_Loop(void)
{
	ROLL2 = 0;

    YAW1   = -38.0f;
    YAW2   = 145.0f;
    ROLL1  = -109.0f;
    PITCH2 = 75.0f;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
    }
	
	LIFT = LIFT_SILVER_UP;
	PITCH1 = -19.81f;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
    }
	
}

// 取左矿,机械臂准备
void Before_Fetch_Left_Silver_Loop(void)
{
	ROLL2 = 0;
	PITCH1 = 10.0f;
    YAW1   = 38.0f;
    YAW2   = 35.0f;
    ROLL1  = -70.0f;
    PITCH2 = 72.0f;

    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
    }
	
	LIFT = LIFT_SILVER_UP;
	PITCH1 = -19.81f;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
    }

}


// 取银矿并放置于矿仓内
void Fetch_Silver_To_Storage_Loop(void)
{
    arm_xipan_open();

    LIFT = LIFT_SILVER_DOWN;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }

    LIFT = LIFT_SILVER_DOWN - 750000;

    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
    PITCH1 = 0.0f;

	while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
    }
	
	YAW1   = -60.0f;
    PITCH1 = -0.440f;
    YAW2   = -40.0f;
    ROLL1  = -90.0f;
    PITCH2 = 88.890f;
	
	
    ROLL2  += 23.0f;
	
	while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
    }
	
	LIFT = -600000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
	osDelay(100);
    arm_xipan_close();
	osDelay(1000);
	
    LIFT -= 250000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}
	
}

// 取银矿放在另一块矿上
void Fetch_Silver_OnTheOther_Silver_Loop(void)
{
    arm_xipan_open();

    LIFT = LIFT_SILVER_DOWN;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }

    LIFT = LIFT_SILVER_DOWN - 750000;

    while (abs(LIFT - LIFT_READ) > 5000) {
        osDelay(1);
			Break_While
    }
	
	YAW1   = -21.47f;
    PITCH1 = 0.0f;
    YAW2   = -40.63f;
    ROLL1  = -172.0f;
    PITCH2 = 34.47f;
	ROLL2  = -10.0f;
	
    while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
	
    LIFT = -540000;
	
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }

}

// 取上银矿并准备兑换
void Before_Exchange_Above_Silver_Loop(void)
{
    arm_xipan_close();
	osDelay(1500);
	
	YAW1   = -5.47f;
    PITCH1 = 0.0f;
    YAW2   = -40.63f;
    ROLL1  = -172.0f;
    PITCH2 = 34.47f;
	ROLL2  = -10.0f;

    while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}

    LIFT = -1090000;

    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
	arm_xipan_open();

	YAW1   = -60.5f;
    PITCH1 = 0.0f;
    YAW2   = -43.94f;
    ROLL1  = -88.0f;
    PITCH2 = 90.0f;
	
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
	
    LIFT += 300000;
	
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
    LIFT -= 300000;
	
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
	YAW1   = 0.0f;
    PITCH1 = 0.0f;
    YAW2   = 0.0f;
    ROLL1  = -90.0f;
    PITCH2 = 70.0f;
	ROLL2 += 0;
	
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
	
	PITCH1 = -10.0f;
	while(fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR)
	{
		osDelay(1);
		Break_While
	}
}


// 取下银矿并准备兑换
void Before_Exchange_Below_Silver_Loop(void)
{	
    YAW1 = 0.0f;
	ROLL2 = 0;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
	}
	
    LIFT = -100000;
	YAW1   = -10.0f;
    PITCH1 = 0.0f;
    YAW2   = -48.0f;
    ROLL1  = -120.0f;
    PITCH2 = 27.0f;
	
    while (fabsf(YAW1 - YAW1_READ) > 10.0f || fabsf(PITCH1 - PITCH1_READ) > 10.0f  || fabsf(YAW2 - YAW2_READ) > 10.0f || fabsf(ROLL1 - ROLL1_READ) > 10.0f) {
        osDelay(1);
		Break_While
	}
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}

	arm_xipan_open();
	
	PITCH1 = -24.0f;
    while (fabsf(PITCH1 - PITCH1_READ) > 10.0f) {
        osDelay(1);
			Break_While
	}
	YAW1 = -24.0f;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
	}
	
	osDelay(500);
//	YAW1   = -43.25f;
//    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
//        osDelay(1);
//			Break_While
//    }
//	osDelay(100);
	
	LIFT -= 500000;
	osDelay(300);
	

	LIFT -= 450000;
	PITCH1 = -10.0f;
	
    osDelay(500);
	
	YAW1   = 0.0f;
	YAW2   = 0.0f;
	
	osDelay(300);
	
	ROLL1  = -90.0f;
    PITCH2 = 70.0f;
	ROLL2 += 30;

	while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
}

// 取下金矿并准备兑换
void Before_Exchange_Below_Glod_Loop(void)
{
    YAW1 = 0.0f;
	ROLL2 = 0;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
	}
	
    LIFT = -100000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}

	YAW1   = -10.0f;
    PITCH1 = 0.0f;
    YAW2   = -48.0f;
    ROLL1  = -120.0f;
    PITCH2 = 27.0f;

	arm_xipan_open();
	
    while (fabsf(YAW1 - YAW1_READ) > 10.0f || fabsf(PITCH1 - PITCH1_READ) > 10.0f  || fabsf(YAW2 - YAW2_READ) > 10.0f || fabsf(ROLL1 - ROLL1_READ) > 10.0f) {
        osDelay(1);
		Break_While
	}
	
	PITCH1 = -24.0f;
    while (fabsf(PITCH1 - PITCH1_READ) > 10.0f) {
        osDelay(1);
			Break_While
	}
	YAW1 = -24.0f;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
	}
	
	osDelay(500);
//	YAW1   = -43.25f;
//    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR) {
//        osDelay(1);
//			Break_While
//    }
//	osDelay(100);
	
	LIFT -= 500000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	

	LIFT -= 450000;
	PITCH1 = -10.0f;
	
    while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
	
	YAW1   = 0.0f;
	YAW2   = 0.0f;
	
	osDelay(300);
	
	ROLL1  = -90.0f;
    PITCH2 = 70.0f;
	ROLL2 -= 60;

	while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
}

// ########################################地矿########################################


// 取地矿准备
void Before_Fetch_Ground_Loop(void)
{
    arm_xipan_open();

	YAW1   = 30.0f;
    PITCH1 = 0.0f;
    YAW2   = 30.0f;
    ROLL1  = -70.0f;
    PITCH2 = 75.f;
	
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR  || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
	
		
    LIFT = -450000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}
	
	PITCH1 = -20.0f;
    while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
			Break_While
	}

}


// 取地矿
void Fetch_Ground_Loop(void)
{
    LIFT = -200000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}
	osDelay(500);

	LIFT = -850000;
	osDelay(500);
	
	YAW1   = 0.0f;
    PITCH1 = 0.0f;
	YAW2   = 0.0f;
    ROLL1  = -90.0f;
    PITCH2 = 90.0f;

    while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
	}
		


}

// 取金矿,机械臂准备
void Before_Fetch_Gold_Loop(void)
{
	arm_xipan_open();
	
    YAW1   = 0.0f;
	PITCH1 = 1.0f;
    YAW2   = 90.0f;
    ROLL1  = -90.0f;
	PITCH2 = 6.0f;
	ROLL2 = 0.0f;
    while (fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
		Break_While
    }
	
    LIFT = -20000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }	
	

}


// 放置于矿仓内
void Fetch_Gold_To_Storage_Loop(void)
{
    LIFT = -750000;
	PITCH2 = 90.0f;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
    ROLL2  += 90.0f;
//	YAW1   = -54.91f;
//    PITCH1 = 0.0f;
//    YAW2   = -39.0f;
//    ROLL1  = -90.0f;
//    PITCH2 = 88.890f;
	
	YAW1   = -60.0f;
    PITCH1 = -0.440f;
    YAW2   = -40.0f;
    ROLL1  = -90.0f;
    PITCH2 = 88.890f;
	
	while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
				Break_While
    }
	
	LIFT = -550000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
    }
	
	osDelay(100);
    arm_xipan_close();
	osDelay(1000);
	
    LIFT -= 250000;
    while (abs(LIFT - LIFT_READ) > LIFT_ERR) {
        osDelay(1);
			Break_While
	}
	
}

// 带矿过狗洞姿态
void Fetch_Gold_To_DogHole_Gesture_Loop(void)
{
	
	LIFT = -50000;
	YAW1   = 3.0f;
    PITCH1 = 0.0f;
    YAW2   = 0.0f;
    ROLL1  = -175.0f;
    PITCH2 = 90.0f;
	ROLL2 += 0;
		
	while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
	Break_While
    }
	
    PITCH1 = -25.0f;
	while (fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR){
        osDelay(1);
		Break_While
    }
}

//不带矿过狗洞
void To_DogHole_Gesture_Loop(void)
{
	PITCH1 = 0.0f;
	LIFT = -50000;
	while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
	Break_While
    }
	
	YAW1   = -65.0f;
    YAW2   = -30.0f;
    ROLL1  = -145.0f;
    PITCH2 = 90.0f;
	ROLL2 += 0;
		
	while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR || fabsf(YAW1 - YAW1_READ) > DM_MOTO_ERR || fabsf(YAW2 - YAW2_READ) > DM_MOTO_ERR || fabsf(ROLL1 - ROLL1_READ) > DM_MOTO_ERR) {
        osDelay(1);
	Break_While
    }
	
	PITCH1 = -7.0f;
	LIFT = -50000;
	while (abs(LIFT - LIFT_READ) > LIFT_ERR || fabsf(PITCH1 - PITCH1_READ) > DM_MOTO_ERR) {
        osDelay(1);
	Break_While
    }
}


