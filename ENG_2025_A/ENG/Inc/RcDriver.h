#ifndef __RC_DRIVER_H
#define __RC_DRIVER_H

#include "Includes.h"
#define USART1_RX_BUFFER_SIZE 128

typedef struct
{
	/* rocker channel information */
	//摇杆通道信息
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;

	/* left and right lever information */
	//左右摇杆信息
	uint8_t sw1;
	uint8_t sw2;
	uint8_t sw1_last;
	uint8_t sw2_last;
} __packed rc_info_t;

typedef struct
{
	int16_t x;//鼠标x方向移动速度
	int16_t y;//鼠标y方向移动速度
	int16_t z;//鼠标滚轮速度
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;//鼠标左键
	uint8_t press_r;//鼠标左键
} __packed rc_info_mouse;

typedef struct
{
	uint16_t info;//键盘数据
}	__packed rc_info_key;

typedef struct
{
	rc_info_t rc;
	rc_info_mouse mouse;
	rc_info_key key;
} __packed RC_Ctl_t;

void Rc_IRQ(void);
void usart_dma_init(void);

#endif
