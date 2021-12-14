#ifndef PC_USART_TASK_H
#define PC_USART_TASK_H

#include "struct_typedef.h"

/* 使用UART1与上位机PC通信 */

#define USART1_RX_BUF_LENGHT     18	//上位机发送的一帧数据长度：23byte
#define USART1_TX_BUF_LENGHT	 3	//回传上位机的一帧数据长度：7byte

#define PC_HEADER_SOF	0xFF	//帧头
#define PC_TAIL_EOF	    0xFE	//帧头

#define SILVER_HALF_KEY	KEY_PRESSED_OFFSET_R	//银矿对位
#define GOLD_HALF_KEY	KEY_PRESSED_OFFSET_F	//金矿对位


/**
  * @brief	 上位机控制工程车帧结构体,23byte
  */
typedef struct 
{
	uint8_t 	SOF;				
	float		x_distance;	// 单位：mm
	float		y_distance;
	float		z_distance;
	float		yaw_dev;
	uint8_t		EOF;
} ControlFrame_t;

enum TaskMode
{
	NO_TASK 	= (uint8_t)0x00,		//手动控制
	SILVER_HALF = (uint8_t)0x01,		//槽中银矿对位
	GOLD_HALF 	= (uint8_t)0x02,		//槽中金矿对位
};

/**
  * @brief	 工程车回传帧结构体,7byte
  */
typedef struct 
{
	uint8_t 		SOF;
	enum TaskMode 	task_mode;
	uint8_t			EOF;
} FeedBackFrame_t;

//工程拓展版
#define EXPAND_CAN_RX_ID					0x520
#define	COMM_CONTROL						0x01	// 通用控制，包括舵机，红外，气缸，电机控制

typedef struct
{
	uint8_t servo1_angle;	//yaw
	uint8_t servo2_angle;	//pitch
	uint8_t servo3_angle;
	uint8_t servo4_angle;
	uint8_t gas_sta;		//a:升降 b:前后伸缩 c:抓取
	uint8_t infrared_sta;	//红外
	uint8_t motor_sta;		//0x01
}ExpandBoard_t;

extern void pc_usart_task(void const * argument);
	

#endif /* PC_USART_TASK_H */


