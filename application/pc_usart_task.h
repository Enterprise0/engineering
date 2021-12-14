#ifndef PC_USART_TASK_H
#define PC_USART_TASK_H

#include "struct_typedef.h"

/* ʹ��UART1����λ��PCͨ�� */

#define USART1_RX_BUF_LENGHT     18	//��λ�����͵�һ֡���ݳ��ȣ�23byte
#define USART1_TX_BUF_LENGHT	 3	//�ش���λ����һ֡���ݳ��ȣ�7byte

#define PC_HEADER_SOF	0xFF	//֡ͷ
#define PC_TAIL_EOF	    0xFE	//֡ͷ

#define SILVER_HALF_KEY	KEY_PRESSED_OFFSET_R	//�����λ
#define GOLD_HALF_KEY	KEY_PRESSED_OFFSET_F	//����λ


/**
  * @brief	 ��λ�����ƹ��̳�֡�ṹ��,23byte
  */
typedef struct 
{
	uint8_t 	SOF;				
	float		x_distance;	// ��λ��mm
	float		y_distance;
	float		z_distance;
	float		yaw_dev;
	uint8_t		EOF;
} ControlFrame_t;

enum TaskMode
{
	NO_TASK 	= (uint8_t)0x00,		//�ֶ�����
	SILVER_HALF = (uint8_t)0x01,		//���������λ
	GOLD_HALF 	= (uint8_t)0x02,		//���н���λ
};

/**
  * @brief	 ���̳��ش�֡�ṹ��,7byte
  */
typedef struct 
{
	uint8_t 		SOF;
	enum TaskMode 	task_mode;
	uint8_t			EOF;
} FeedBackFrame_t;

//������չ��
#define EXPAND_CAN_RX_ID					0x520
#define	COMM_CONTROL						0x01	// ͨ�ÿ��ƣ�������������⣬���ף��������

typedef struct
{
	uint8_t servo1_angle;	//yaw
	uint8_t servo2_angle;	//pitch
	uint8_t servo3_angle;
	uint8_t servo4_angle;
	uint8_t gas_sta;		//a:���� b:ǰ������ c:ץȡ
	uint8_t infrared_sta;	//����
	uint8_t motor_sta;		//0x01
}ExpandBoard_t;

extern void pc_usart_task(void const * argument);
	

#endif /* PC_USART_TASK_H */


