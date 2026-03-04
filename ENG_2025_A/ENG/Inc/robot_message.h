#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

#include "struct_typedef.h"
#include "bsp_usb.h"
// #include "config.h"

#define True         1
#define False        0

#define message_data __attribute__((packed))

// RESVE_ID
#define VISION_ONE_TO_FIVE_ID 0x0A
#define VISION_SIX_TO_TEN_ID  0x0B

// SEND_ID
#define NUC_START_RECORD 0x0C

#define NUC_PRESSED_FLAG nuc_pressed_if

enum MOVEIT_STATE {
    M_SUCCESS = 1,  // 操作成功完成
    
    FAILURE = 99999,  // 通用失败状态，没有更具体的错误信息

    PLANNING_FAILED = -1,  // 规划失败，例如路径或动作无法找到有效解

    INVALID_MOTION_PLAN = -2,  // 运动规划无效，可能路径不安全或不符合约束

    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3,  // 环境变化导致原计划失效，如障碍物移动

    CONTROL_FAILED = -4,  // 控制执行失败，如控制器无法跟随轨迹

    UNABLE_TO_AQUIRE_SENSOR_DATA = -5,  // 无法获取传感器数据，可能影响避障或感知

    TIMED_OUT = -6,  // 操作超时，未在规定时间内完成

    PREEMPTED = -7,  // 操作被抢占，通常由更高优先级任务中断当前操作

    START_STATE_IN_COLLISION = -10,  // 起始位置处于碰撞状态，无法安全开始运动

    START_STATE_VIOLATES_PATH_CONSTRAINTS = -11,  // 起始状态违反路径约束，如关节限位、姿态限制等

    START_STATE_INVALID = -26,  // 起始状态无效，可能机器人配置错误或数据损坏

    GOAL_IN_COLLISION = -12,  // 目标位置与环境中的物体发生碰撞

    GOAL_VIOLATES_PATH_CONSTRAINTS = -13,  // 目标状态违反路径约束条件

    GOAL_CONSTRAINTS_VIOLATED = -14,  // 目标约束未满足，如末端执行器方向不对

    GOAL_STATE_INVALID = -27,  // 目标状态无效，如非法的姿态或配置

    UNRECOGNIZED_GOAL_TYPE = -28,  // 不支持的目标类型，无法解析目标指令

    INVALID_GROUP_NAME = -15,  // 无效的运动组名称，可能是拼写错误或配置问题

    INVALID_GOAL_CONSTRAINTS = -16,  // 提供的目标约束格式错误或不合法

    INVALID_ROBOT_STATE = -17,  // 机器人状态无效，如关节数据异常或不在允许范围内

    INVALID_LINK_NAME = -18,  // 无效的链接名称，如不存在该连杆

    INVALID_OBJECT_NAME = -19,  // 无效的对象名称，如碰撞世界中无此物体

    FRAME_TRANSFORM_FAILURE = -21,  // 坐标变换失败，如 TF 树中找不到对应变换

    COLLISION_CHECKING_UNAVAILABLE = -22,  // 碰撞检测不可用，可能相关模块未启动

    ROBOT_STATE_STALE = -23,  // 机器人状态过期，可能未及时更新

    SENSOR_INFO_STALE = -24,  // 传感器信息过期，可能未及时读取或同步

    COMMUNICATION_FAILURE = -25,  // 通信失败，如与硬件或服务端连接中断

    CRASH = -29,  // 系统崩溃，可能是程序异常终止

    ABORT = -30,  // 操作被显式中止，通常是用户请求取消

    NO_IK_SOLUTION = -31,  // 逆运动学求解失败，找不到可达到目标位姿的关节角组合
};

typedef struct {
    float len;
    float yaw1;
    float pitch1;
    float yaw2;
    float roll1;
    float pitch2;
    float roll2;
} action_data_t;

typedef struct {
    action_data_t action_data[150];
    int32_t state[2];
    uint16_t total_num;
} nuc_receive_data_t;

typedef struct {
    uint8_t nuc_start_flag;
} nuc_transmit_data_t;

typedef struct {
    uint8_t header;
    uint16_t length;
    uint8_t cmd_id;
} send_packge_config_t;

typedef struct {
    uint8_t cmd_id_queue[10];
    uint8_t cmd_id_frq[10];
    uint16_t total_num;
    uint16_t now_pos;
} cmd_id_queue_t;

extern nuc_receive_data_t nuc_receive_data;
extern nuc_transmit_data_t nuc_transmit_data;
extern cmd_id_queue_t cmd_id_queue;

/*
head
len
id

data[287]

crc

291

*/

void data_send_control(uint8_t cmd_id, uint16_t freq);
void data_update(uint8_t cmd_id);

void Nuc_data_unpacked(void);

void cmd_id_init(void);
void cmd_id_task_create(uint8_t cmd_id, uint16_t freq);
void cmd_id_queue_handle(void);

void send_data_to_nuc(void);
void float_to_u8(float *datain, uint8_t *dataout);
void u8_to_float(uint8_t *datain, float *dataout);
void u8_to_int32_t(uint8_t *datain, int32_t *dataout);
void u8_to_uint16_t(uint8_t *datain, uint16_t *dataout);
// uint8_t num_in_list_dect(uint8_t dect_id,uint8_t *dect_list,uint8_t lengths);
uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len);

static const uint8_t CRC08_Table[256] = {
    0x00,
    0x5e,
    0xbc,
    0xe2,
    0x61,
    0x3f,
    0xdd,
    0x83,
    0xc2,
    0x9c,
    0x7e,
    0x20,
    0xa3,
    0xfd,
    0x1f,
    0x41,
    0x9d,
    0xc3,
    0x21,
    0x7f,
    0xfc,
    0xa2,
    0x40,
    0x1e,
    0x5f,
    0x01,
    0xe3,
    0xbd,
    0x3e,
    0x60,
    0x82,
    0xdc,
    0x23,
    0x7d,
    0x9f,
    0xc1,
    0x42,
    0x1c,
    0xfe,
    0xa0,
    0xe1,
    0xbf,
    0x5d,
    0x03,
    0x80,
    0xde,
    0x3c,
    0x62,
    0xbe,
    0xe0,
    0x02,
    0x5c,
    0xdf,
    0x81,
    0x63,
    0x3d,
    0x7c,
    0x22,
    0xc0,
    0x9e,
    0x1d,
    0x43,
    0xa1,
    0xff,
    0x46,
    0x18,
    0xfa,
    0xa4,
    0x27,
    0x79,
    0x9b,
    0xc5,
    0x84,
    0xda,
    0x38,
    0x66,
    0xe5,
    0xbb,
    0x59,
    0x07,
    0xdb,
    0x85,
    0x67,
    0x39,
    0xba,
    0xe4,
    0x06,
    0x58,
    0x19,
    0x47,
    0xa5,
    0xfb,
    0x78,
    0x26,
    0xc4,
    0x9a,
    0x65,
    0x3b,
    0xd9,
    0x87,
    0x04,
    0x5a,
    0xb8,
    0xe6,
    0xa7,
    0xf9,
    0x1b,
    0x45,
    0xc6,
    0x98,
    0x7a,
    0x24,
    0xf8,
    0xa6,
    0x44,
    0x1a,
    0x99,
    0xc7,
    0x25,
    0x7b,
    0x3a,
    0x64,
    0x86,
    0xd8,
    0x5b,
    0x05,
    0xe7,
    0xb9,
    0x8c,
    0xd2,
    0x30,
    0x6e,
    0xed,
    0xb3,
    0x51,
    0x0f,
    0x4e,
    0x10,
    0xf2,
    0xac,
    0x2f,
    0x71,
    0x93,
    0xcd,
    0x11,
    0x4f,
    0xad,
    0xf3,
    0x70,
    0x2e,
    0xcc,
    0x92,
    0xd3,
    0x8d,
    0x6f,
    0x31,
    0xb2,
    0xec,
    0x0e,
    0x50,
    0xaf,
    0xf1,
    0x13,
    0x4d,
    0xce,
    0x90,
    0x72,
    0x2c,
    0x6d,
    0x33,
    0xd1,
    0x8f,
    0x0c,
    0x52,
    0xb0,
    0xee,
    0x32,
    0x6c,
    0x8e,
    0xd0,
    0x53,
    0x0d,
    0xef,
    0xb1,
    0xf0,
    0xae,
    0x4c,
    0x12,
    0x91,
    0xcf,
    0x2d,
    0x73,
    0xca,
    0x94,
    0x76,
    0x28,
    0xab,
    0xf5,
    0x17,
    0x49,
    0x08,
    0x56,
    0xb4,
    0xea,
    0x69,
    0x37,
    0xd5,
    0x8b,
    0x57,
    0x09,
    0xeb,
    0xb5,
    0x36,
    0x68,
    0x8a,
    0xd4,
    0x95,
    0xcb,
    0x29,
    0x77,
    0xf4,
    0xaa,
    0x48,
    0x16,
    0xe9,
    0xb7,
    0x55,
    0x0b,
    0x88,
    0xd6,
    0x34,
    0x6a,
    0x2b,
    0x75,
    0x97,
    0xc9,
    0x4a,
    0x14,
    0xf6,
    0xa8,
    0x74,
    0x2a,
    0xc8,
    0x96,
    0x15,
    0x4b,
    0xa9,
    0xf7,
    0xb6,
    0xe8,
    0x0a,
    0x54,
    0xd7,
    0x89,
    0x6b,
    0x35,
};

#endif
