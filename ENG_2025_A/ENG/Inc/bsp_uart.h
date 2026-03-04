#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "Includes.h"

#define FRAME_HEAD 0xAA

#define USART6_RX_BUFFER_SIZE 128
#define UART7_RX_BUFFER_SIZE 128
#define UART8_RX_BUFFER_SIZE 128
#define USART3_RX_BUFFER_SIZE 128

#define SYNC_TO_C_SIZE (sizeof(DataUnion))      // 发送给C板数据包大小
#define SYNC_FROM_C_SIZE (sizeof(BackDataUnion))// 从C板接收数据包大小



#define FRAME_SIZE2 30

//MyCtrlUnion my_ctrl_data;
#define FRAME_SIZE 30
#define DATA_SIZE 28
/*
typedef struct __attribute__((packed)) {
	uint8_t key1;
	uint8_t key2;
	float vx;
	float vy;
	float vz;
	float roll;
	float pitch;
	float yaw;
} MyCtrlTD;

typedef union{
	MyCtrlTD data;
	uint8_t bytes[sizeof(MyCtrlTD)];
}MyCtrlUnion;
*/
typedef struct __attribute__((packed)){
	uint8_t head;
// 基于机械臂建模的角度目标值 
	float yaw1;
	float pitch1;
	float yaw2;
	float roll1;	
	float pitch2;
	float roll2;
	uint8_t arm_enable_flag;// 机械臂使能标志位
	uint8_t arm_camera;
	uint8_t arm_valve;
}FiveJointCtrlDataTD;//A板数据

typedef struct __attribute__((packed)){
	uint8_t head;
	float yaw1;
	float pitch1;
	float yaw2;
	float roll1;	
	float pitch2;
	uint8_t arm_init_flag;
	uint8_t arm_enable_flag;
}FiveJointBackDataTD;//C板数据
/*
typedef struct __attribute__((packed)){
	uint8_t key1;
	uint8_t key2;
	uint8_t temp[2];
	float vx;
	float vy;
	float vz;
	float roll;
	float pitch;
	float yaw;
}MyCtrlDataTD;

//48bit(9+30+9)

typedef union{
	MyCtrlDataTD data;
	uint8_t bytes[sizeof(MyCtrlDataTD)];
}MyCtrlDataUnion;
*/
typedef union{
	FiveJointCtrlDataTD data;
	uint8_t bytes[sizeof(FiveJointCtrlDataTD)];
}DataUnion;


typedef union{
	FiveJointBackDataTD data;
	uint8_t bytes[sizeof(FiveJointBackDataTD)];
}BackDataUnion;
/*
typedef union {
	MyCtrlDataTD data;
    uint8_t bytes[DATA_SIZE]; 
} ControlData;
extern ControlData my_ctrl_data;
*/

extern DataUnion sync_data_to_c;
extern BackDataUnion sync_data_from_c;


int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void usart_dma_init(void);
void sync_data_to_c_init(void);
void decode_five_jiont_back_data(void);


// ####################走图传链路串口发送,与串口接收分开定义###########################

typedef struct __packed 
{
	uint8_t SOF;
	uint16_t data_lenth;
	uint8_t seq;
	uint8_t CRC8;
} frame_header;


typedef struct __packed
{
	uint8_t mode;  // 自控为1,臂控为2
	float yaw1;
	float pitch1;
	float yaw2;
	float roll1;
	float pitch2;
	uint32_t lift;
	uint8_t undefine[5];
}  data_t;


typedef struct __packed
{
	frame_header header;
	uint16_t cmd_id;
	data_t data;
	uint16_t CRC16;
} frame_t;



void Robot_Custom_Data_Transmit_Init(void);
void Robot_Custom_Data_Transmit(void);


// ####################走图传链路串口发送,与串口接收分开定义###########################


#endif