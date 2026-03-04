#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

extern UART_HandleTypeDef huart6;
frame_header_t referee_receive_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
referee_warning_t referee_warning;


robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
robot_pos_t robot_pos;
ext_buff_t buff_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
robot_interaction_data_t robot_interaction_data;

custom_controller_t custom_controller_data_t;
DataFrame_t NEW_RC_DATA;

uint8_t char_data[30];


void referee_data_solve(uint8_t *frame)
{
  uint16_t cmd_id = 0;
  uint8_t state_first_run = 1;
  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_t));

  if(referee_receive_header.SOF == HEADER_SOF)
  {
    index += sizeof(frame_header_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
      case GAME_STATE_CMD_ID:
      {
          memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
      }
      break;
      case GAME_RESULT_CMD_ID:
      {
          memcpy(&game_result, frame + index, sizeof(game_result));
      }
      break;
      case GAME_ROBOT_HP_CMD_ID:
      {
          memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
      }
      break;
      case FIELD_EVENTS_CMD_ID:
      {
          memcpy(&field_event, frame + index, sizeof(field_event));
      }
      break;
      case REFEREE_WARNING_CMD_ID:
      {
          memcpy(&referee_warning, frame + index, sizeof(referee_warning));
      }
      break;

      case ROBOT_STATE_CMD_ID:
      {  
          memcpy(&robot_state, frame + index, sizeof(robot_state));
          if(state_first_run)
          {
            switch(get_robot_id())
            {
              case RED_ENGINEER:
              {
								
              }
                break;
              case BLUE_ENGINEER:
              {
								
              }
                break;
              default:
                break;
            }
            state_first_run = 0;
          }
      }
      break;
      case POWER_HEAT_DATA_CMD_ID:
      {
          memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
      }
      break;
      case ROBOT_POS_CMD_ID:
      {
          memcpy(&robot_pos, frame + index, sizeof(robot_pos));
      }
      break;
      case BUFF_MUSK_CMD_ID:
      {
          memcpy(&buff_t, frame + index, sizeof(buff_t));
      }
      break;
      case ROBOT_HURT_CMD_ID:
      {
          memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
      }
      break;
      case SHOOT_DATA_CMD_ID:
      {
          memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
      }
      break;
      case BULLET_REMAINING_CMD_ID:
      {
          memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
      }
      break;
      case STUDENT_INTERACTIVE_DATA_CMD_ID:
      {
          memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data));
      }
      break;
			case CUSTOM_ROBOT_DATA_ID:
      {
          memcpy(&custom_controller_data_t, frame + index, sizeof(custom_controller_data_t));
      }
      break;
	  
      default:
      {
          break;
      }
    }
  }
}

void decodeDataFrame(uint8_t *data) {
    // 检查帧头是否正确
    if (data[0] != 0xA9 || data[1] != 0x53) {
        return;
    }

    // 设置帧头
    NEW_RC_DATA.header1 = data[0];
    NEW_RC_DATA.header2 = data[1];

    // 解析通道数据
    NEW_RC_DATA.channel0 = ((uint16_t)(data[3] & 0x07) << 8) | data[2];
    NEW_RC_DATA.channel1 = ((uint16_t)(data[4] & 0x3F) << 5) | ((data[3] >> 3) & 0x1F);
    NEW_RC_DATA.channel2 = ((uint16_t)(data[6] & 0x01) << 10) | ((uint16_t)data[5] << 2) | ((data[4] >> 6) & 0x03);
    NEW_RC_DATA.channel3 = ((uint16_t)(data[7] & 0x0F) << 7) | (data[6] & 0xFE);

    // 解析开关和按钮状态
    NEW_RC_DATA.gear_switch = ((data[7] >> 4) & 0x03);
    NEW_RC_DATA.pause_button = (data[7] >> 6) & 0x01;
    NEW_RC_DATA.custom_button_left = (data[7] >> 7) & 0x01;
    NEW_RC_DATA.custom_button_right = data[8] & 0x01;

    // 解析拨轮和扳机键状态
    NEW_RC_DATA.dial = ((uint16_t)(data[9] & 0x0F) << 7) | (data[8] & 0xFE);
    NEW_RC_DATA.toggle_switch = (data[9] >> 4) & 0x01;
}

uint8_t get_robot_id(void)
{
  return robot_state.robot_id;
}
