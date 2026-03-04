#include "Includes.h"

#define RC_FRAME_LENGTH 18

uint8_t	USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE];

extern DMA_HandleTypeDef hdma_usart1_rx;

void DBUS_decode(uint8_t *buff)
{
	uint8_t temp_sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	uint8_t temp_sw2 = (buff[5] >> 4) & 0x0003;
	
	if(temp_sw1 != RC_CtrlData.rc.sw1){
		RC_CtrlData.rc.sw1_last = RC_CtrlData.rc.sw1;
	}
	if(temp_sw2 != RC_CtrlData.rc.sw2){
		RC_CtrlData.rc.sw2_last = RC_CtrlData.rc.sw2;
	}
	RC_CtrlData.rc.sw1 = temp_sw1;
	RC_CtrlData.rc.sw2 = temp_sw2;

	RC_CtrlData.rc.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	RC_CtrlData.rc.ch1 -= 1024;
	RC_CtrlData.rc.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	RC_CtrlData.rc.ch2 -= 1024;
	RC_CtrlData.rc.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	RC_CtrlData.rc.ch3 -= 1024;
	RC_CtrlData.rc.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	RC_CtrlData.rc.ch4 -= 1024;

	RC_CtrlData.mouse.x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8);

	RC_CtrlData.mouse.press_l = buff[12];
	RC_CtrlData.mouse.press_r = buff[13];

	RC_CtrlData.key.info = ((int16_t)buff[14]) | ((int16_t)buff[15] << 8);
	
	if(RC_CtrlData.rc.ch1<=6 && RC_CtrlData.rc.ch1>=-6)	RC_CtrlData.rc.ch1=0;
	if(RC_CtrlData.rc.ch2<=6 && RC_CtrlData.rc.ch2>=-6)	RC_CtrlData.rc.ch2=0;
	if(RC_CtrlData.rc.ch3<=6 && RC_CtrlData.rc.ch3>=-6)	RC_CtrlData.rc.ch3=0;
	if(RC_CtrlData.rc.ch4<=6 && RC_CtrlData.rc.ch4>=-6)	RC_CtrlData.rc.ch4=0;

	if(RC_CtrlData.rc.ch1>650)	RC_CtrlData.rc.ch1 =  650;
	if(RC_CtrlData.rc.ch1<-650)	RC_CtrlData.rc.ch1 = -650;
	if(RC_CtrlData.rc.ch2>650)	RC_CtrlData.rc.ch2 =  650;
	if(RC_CtrlData.rc.ch2<-650)	RC_CtrlData.rc.ch2 = -650;
	if(RC_CtrlData.rc.ch3>650)	RC_CtrlData.rc.ch3 =  650;
	if(RC_CtrlData.rc.ch3<-650)	RC_CtrlData.rc.ch3 = -650;
	if(RC_CtrlData.rc.ch4>650)	RC_CtrlData.rc.ch4 =  650;
	if(RC_CtrlData.rc.ch4<-650)	RC_CtrlData.rc.ch4 = -650;

}

void Rc_IRQ()
{
    // 检查UART1的空闲中断标志位是否被置位，并且空闲中断是否已使能
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE)) {

        // 清除UART1的空闲中断标志位
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // 禁用DMA接收，以便读取当前接收到的数据长度
        __HAL_DMA_DISABLE(&hdma_usart1_rx);

        // 检查接收到的数据长度是否等于预期的遥控器帧长度（RC_FRAME_LENGTH）
        if ((USART1_RX_BUFFER_SIZE - hdma_usart1_rx.Instance->NDTR) == RC_FRAME_LENGTH) {
            // 如果接收到的数据长度正确，调用解码函数处理接收到的数据
            DBUS_decode(USART1_Rx_Buffer);
        }

        // 重新设置DMA接收缓冲区的计数器为初始值（USART1_RX_BUFFER_SIZE）
        __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUFFER_SIZE);

        // 重新使能DMA接收，以便继续接收数据
        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
}

