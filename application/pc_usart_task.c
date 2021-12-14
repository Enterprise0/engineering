#include "pc_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "string.h"

#include "remote_control.h"
#include "can_receive.h"

extern UART_HandleTypeDef huart1;
extern RC_ctrl_t rc_ctrl;	////遥控器控制变量

static void set_task_mode(void);	//设置手动对位还是视觉对位

uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];

ControlFrame_t  rx_ctrl;	//接收上位机数据
FeedBackFrame_t tx_ctrl;	//回传上位机

ExpandBoard_t   expand_tx;	//拓展板数据

/* 上下位机通信任务 */
void pc_usart_task(void const* argument)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], sizeof(rx_ctrl));	//串口1接收双缓冲

	//初始化 发送结构体
    tx_ctrl.SOF = PC_HEADER_SOF;
    tx_ctrl.task_mode = NO_TASK;
    tx_ctrl.EOF = PC_TAIL_EOF;

	expand_tx.motor_sta = COMM_CONTROL;
	expand_tx.servo1_angle = 77;	//pitch
	expand_tx.servo2_angle = 118;	//yaw
	expand_tx.servo3_angle = 0;
	expand_tx.servo4_angle = 0;
	expand_tx.gas_sta = 0x0;
	expand_tx.infrared_sta = 0;		//红外
	
    while(1)
    {
        set_task_mode();
        usart1_tx_dma_enable((uint8_t*)&tx_ctrl, sizeof(FeedBackFrame_t));
		
		CAN_cmd_expand();	//电磁阀
		
        osDelay(10);

    }
}

void init_pc_struct_data(void)
{
    memset(&rx_ctrl, 0, sizeof(ControlFrame_t));
    memset(&tx_ctrl, 0, sizeof(FeedBackFrame_t));
}

extern DMA_HandleTypeDef hdma_usart1_rx;
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) //
        {
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_LENGHT;

//			//set memory buffer 1
//            //设定缓冲区1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if(*usart1_rx_buf[0] == PC_HEADER_SOF)
                memcpy(&rx_ctrl, usart1_rx_buf[1], sizeof(rx_ctrl));
        }
        else
        {
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_LENGHT;

            //set memory buffer 0
            //设定缓冲区0
//			hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//			DMA2_Stream5->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if(*usart1_rx_buf[0] == PC_HEADER_SOF)
                memcpy(&rx_ctrl, usart1_rx_buf[0], sizeof(rx_ctrl));
        }
    }
}

static void set_task_mode(void)
{
    if(tx_ctrl.task_mode == NO_TASK && (rc_ctrl.key.v & GOLD_HALF_KEY))	//金矿对位
    {
        tx_ctrl.task_mode = GOLD_HALF;
    }
    else if(tx_ctrl.task_mode == NO_TASK && (rc_ctrl.key.v & SILVER_HALF_KEY))	//银矿对位
    {
        tx_ctrl.task_mode = SILVER_HALF;
    }

    if(tx_ctrl.task_mode == GOLD_HALF && (rc_ctrl.key.v & GOLD_HALF_KEY) && (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT))
    {
        tx_ctrl.task_mode = NO_TASK;
    }
    else if(tx_ctrl.task_mode == SILVER_HALF && (rc_ctrl.key.v & GOLD_HALF_KEY) && (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT))
    {
        tx_ctrl.task_mode = NO_TASK;
    }
}
