#include "bsp_uart.h"

// AC板通信协议
/*
帧头 0x55
中间8字节数据
帧尾 0xAA
*/

uint8_t USART6_Rx_Buffer[USART6_RX_BUFFER_SIZE];
uint8_t USART3_Rx_Buffer[USART3_RX_BUFFER_SIZE];
uint8_t UART7_Rx_Buffer[UART7_RX_BUFFER_SIZE];
uint8_t UART8_Rx_Buffer[UART8_RX_BUFFER_SIZE];

extern uint8_t USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE];

extern uint8_t power_less_flag;

DataUnion sync_data_to_c;       // 同步数据到C板
BackDataUnion sync_data_from_c; // 从C板同步的数据

// 同步C板数据初始化
void sync_data_to_c_init()
{
    sync_data_to_c.data.head   = 0xAA;
    sync_data_to_c.data.yaw1   = 0.0f;
    sync_data_to_c.data.pitch1 = 0.0f;
    sync_data_to_c.data.yaw2   = 90.0;
    sync_data_to_c.data.roll1  = -90.0f;
    sync_data_to_c.data.pitch2 = 0.0f;
    sync_data_to_c.data.roll2  = 0.0f;
	sync_data_to_c.data.arm_camera = 0.0f;
	sync_data_to_c.data.arm_valve = 0.0f;
}

// 拷贝UART从C板接收的数据到sync_data_from_c联合体中,同时使用联合体进行数据类型转换
void decode_five_jiont_back_data()
{
    if (USART6_Rx_Buffer[0] == 0xAA) {
        memcpy(&sync_data_from_c, &USART6_Rx_Buffer, sizeof(sync_data_from_c));
    }
}

// UART接收函数配置(dma搬运),(?)与直接用这个函数HAL_UARTEx_ReceiveToIdle_DMA有什么区别
int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
    uint32_t tmp1 = 0;

    tmp1 = huart->RxState;

    if (tmp1 == HAL_UART_STATE_READY) {
        if ((pData == NULL) || (Size == 0)) {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode  = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*
         * Enable the DMA transfer for the receiver request by setting the DMAR bit
         * in the UART CR3 register
         */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

// UART和dma初始化配置
void usart_dma_init(void)
{
    /* open uart idle it */

    __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 遥控器
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);// 裁判系统常规链路()
    __HAL_UART_CLEAR_IDLEFLAG(&huart6); // C板
    __HAL_UART_CLEAR_IDLEFLAG(&huart8); // 裁判系统图传链路

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    uart_receive_dma_no_it(&huart1, USART1_Rx_Buffer, USART1_RX_BUFFER_SIZE);
    uart_receive_dma_no_it(&huart3, USART3_Rx_Buffer, USART3_RX_BUFFER_SIZE);
    uart_receive_dma_no_it(&huart6, USART6_Rx_Buffer, USART6_RX_BUFFER_SIZE);
    uart_receive_dma_no_it(&huart8, UART8_Rx_Buffer, UART8_RX_BUFFER_SIZE);
}

// ############################################图传链路串口发送给自定义控制器################################################

uint8_t Transmit_buff[128] = {0};

frame_t frame;

void Robot_Custom_Data_Transmit(void)
{
    frame.header.seq++;
    frame.header.CRC8 = calc_crc8((unsigned char *)(&frame.header), 4);

    frame.data.yaw1   = sync_data_from_c.data.yaw1;
    frame.data.pitch1 = sync_data_from_c.data.pitch1;
    frame.data.yaw2   = sync_data_from_c.data.yaw2;
    frame.data.roll1  = sync_data_from_c.data.roll1;
    frame.data.pitch2 = sync_data_from_c.data.pitch2;
	frame.data.lift = -LIFT_READ;

    frame.CRC16 = calc_crc16((unsigned char *)(&frame), 37);

    memcpy(Transmit_buff, &frame, 39);
    HAL_UART_Transmit_DMA(&huart3, Transmit_buff, 39);
}

void Robot_Custom_Data_Transmit_Init(void)
{
    frame.header.SOF        = 0xA5;
    frame.header.seq        = 0;
    frame.header.data_lenth = 30;
    frame.cmd_id            = 0x0309;
    frame.data.mode         = 1;
}
