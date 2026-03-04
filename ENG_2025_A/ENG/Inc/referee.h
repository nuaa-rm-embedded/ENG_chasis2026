#ifndef REFEREE_H
#define REFEREE_H

//修订于裁判系统1.7.0
//改动较大

#include "includes.h"

typedef enum
{
  RED_HERO        = 1,
  RED_ENGINEER    = 2,
  RED_STANDARD_1  = 3,
  RED_STANDARD_2  = 4,
  RED_STANDARD_3  = 5,
  RED_AERIAL      = 6,
  RED_SENTRY      = 7,
  RED_RADAR       = 9,
  RED_OUTPOST     = 10,
  RED_BASE        = 11,
  BLUE_HERO       = 101,
  BLUE_ENGINEER   = 102,
  BLUE_STANDARD_1 = 103,
  BLUE_STANDARD_2 = 104,
  BLUE_STANDARD_3 = 105,
  BLUE_AERIAL     = 106,
  BLUE_SENTRY     = 107,
  BLUE_RADAR      = 109,
  BLUE_OUTPOST    = 110,
  BLUE_BASE       = 111,
} robot_id_t;

typedef enum
{
  RED_HERO_CILENT        = 0x0101,
  RED_ENGINEER_CLIENT    = 0x0102,
  RED_STANDARD_CLIENT_1  = 0x0103,
  RED_STANDARD_CLIENT_2  = 0x0104,
  RED_STANDARD_CLIENT_3  = 0x0105,
  RED_AERIAL_CLIENT      = 0x0106,
  BLUE_HERO_CILENT       = 0x0165,
  BLUE_ENGINEER_CLIENT   = 0x0166,
  BLUE_STANDARD_CLIENT_1 = 0x0167,
  BLUE_STANDARD_CLIENT_2 = 0x0168,
  BLUE_STANDARD_CLIENT_3 = 0x0169,
  BLUE_AERIAL_CLIENT     = 0x016A,  
} client_id_t;

typedef enum
{
  PROGRESS_UNSTART        = 0,
  PROGRESS_PREPARE        = 1,
  PROGRESS_SELFCHECK      = 2,
  PROGRESS_5sCOUNTDOWN    = 3,
  PROGRESS_BATTLE         = 4,
  PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef struct __packed //0001 
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef struct __packed //0002
{
  uint8_t winner;
} ext_game_result_t;

typedef struct __packed //0003
{
  uint16_t red_1_robot_HP;  
  uint16_t red_2_robot_HP;  
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;  
  uint16_t reserved1;  
  uint16_t red_7_robot_HP;  

  uint16_t red_outpost_HP; 
  uint16_t red_base_HP;	  
  uint16_t blue_1_robot_HP; 
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t reserved2;
  uint16_t blue_7_robot_HP;

  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef struct __packed //0101
{
  uint32_t event_type;
} ext_event_data_t;

typedef struct __packed //0104
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}referee_warning_t; 


typedef struct __packed //0105
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}dart_info_t;

typedef struct __packed //0201
{ 
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
}robot_status_t; 


typedef struct __packed //0x0202
{
  uint16_t reserved1;
  uint16_t reserved2; 
  float reserved3; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

typedef struct __packed //0203
{ 
  float x; 
  float y; 
  float angle; 
}robot_pos_t; 

typedef struct __packed //0204
{ 
  uint8_t recovery_buff;  
  uint8_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff;
  uint8_t remaining_energy; //能量工程有吗 
}ext_buff_t; 

typedef struct __packed //0x0206
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef struct __packed //0x0207
{
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed;  
} ext_shoot_data_t;

typedef struct __packed //0208
{
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
} ext_bullet_remaining_t;

typedef struct __packed //0209 RFID
{ 
	uint32_t rfid_status;
// bit 19：己方与兑换区不重叠的补给区/RMUL补给区 bit 20：己方与兑换区重叠的补给区 
} ext_rfid_status_t;
/*
  协议中0x020A-0x020E不是对工程机器人的命令
*/
typedef struct __packed //0301
{ 
  uint16_t data_cmd_id; 
  uint16_t sender_id; 
  uint16_t receiver_id; 
  uint8_t user_data[112]; 
}robot_interaction_data_t; 

typedef struct __packed
{
  uint8_t data[30];
} robot_interactive_data_t;

typedef struct __packed //画图删除指令
{
  uint8_t delete_type; 
  uint8_t layer; 
} ext_client_custom_graphic_delete_t; //0x0301 - 0x0100

typedef struct __packed  
{ 
  uint8_t figure_name[3];  
  uint32_t operate_tpye:3;  
  uint32_t figure_tpye:3;  
  uint32_t layer:4;  
  uint32_t color:4;  
  uint32_t details_a:9; 
uint32_t width:10;  
uint32_t start_x:11;  
uint32_t start_y:11;  
uint32_t details_c:10;  
uint32_t details_d:11;  
uint32_t details_e:11;   
} graphic_data_struct_t;  //画图 0x0301 - 0x0101 选手端画1个

typedef struct __packed //0x0301 - 0x0102 选手端画2个
{
  uint16_t comment_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_2_t;

typedef struct __packed //0x0301 - 0x0103 选手端画5个
{
  uint16_t comment_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_3_t;

typedef struct __packed //0x0301 - 0x0104 选手端画7个
{
  uint16_t comment_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_4_t;

typedef struct __packed //0x0301 - 0x0110 选手端写字
{
  uint16_t comment_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  graphic_data_struct_t  grapic_data_struct; 
  uint8_t data[30];
} ext_client_custom_character_t;

typedef struct __packed //0x0303
{
  float target_position_x; 
  float target_position_y; 
  uint8_t cmd_keyboard; 
  uint8_t target_robot_id; 
  uint16_t cmd_source; 
} ext_map_client_command_t;

typedef struct __packed //0x0305
{
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

typedef struct __packed//0x0304  //遥控器发送键鼠数据同步从图传链路此命令ID发送给机器人
{
  int16_t mouse_x; 
  int16_t mouse_y; 
  int16_t mouse_z; 
  int8_t left_button_down; 
  int8_t right_button_down; 
  uint16_t keyboard_value; 
  uint16_t reserved; 
} ext_robot_command_t;

typedef struct __packed//0x0302  //自定义控制器发送数据至机器人 30字节
{ 
  uint8_t data[30]; 
}custom_robot_data_t; 

typedef struct __packed//0x0309  //机器人发送数据至自定义控制器 30字节
{ 
  uint8_t data[30]; 
}robot_custom_data_t; 

//  新工程自定义控制器数据
typedef struct __packed
{
	uint8_t lift_flag; // 0为不动,1为上,2为下
	float yaw1;
	float pitch1;
	float yaw2;
	float roll1;
	float pitch2;
	uint8_t roll2_flag; // 0为不动,1为正转,2为反转
	uint8_t pump_flag;	// 1为开,0为关
	uint8_t undefine[7];
} custom_controller_t;

typedef struct __packed
{
    uint8_t header1;
    uint8_t header2;
    uint16_t channel0 : 11;
    uint16_t channel1 : 11;
    uint16_t channel2 : 11;
    uint16_t channel3 : 11;
    uint8_t gear_switch : 2;
    uint8_t pause_button : 1;
    uint8_t custom_button_left : 1;
    uint8_t custom_button_right : 1;
    uint16_t dial : 11;
    uint8_t toggle_switch : 1;
} DataFrame_t;


extern custom_controller_t custom_controller_data_t;

extern void referee_data_solve(uint8_t *frame);
extern uint8_t get_robot_id(void);
void draw_character(uint32_t opre_type);
void draw_front_sight(void);
void decodeDataFrame(uint8_t *data);
#endif
