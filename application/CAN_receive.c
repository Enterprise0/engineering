/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[8];	//底盘0-3, 障碍块4-5, 救援6-7
static motor_measure_t motor_catch[8];		//抓手pitch 0-1, 旋转2, 升降 4-5
	
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
//hcan1
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  user_tx_message;
static uint8_t              user_can_send_data[8];
//hcan2
static CAN_TxHeaderTypeDef  catch_tx_message;
static uint8_t              catch_can_send_data[8];
static CAN_TxHeaderTypeDef  up_tx_message;
static uint8_t              up_can_send_data[8];
//expand
static CAN_TxHeaderTypeDef	expand_tx_message;
static uint8_t 				expand_can_send_data[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan == &hcan1 )
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_BARRIER_L_ID:
			case CAN_BARRIER_R_ID:
			case CAN_SAVER_L_ID:
			case CAN_SAVER_R_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
//				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}

			default:
			{
				break;
			}
		}
	}
	else if( hcan == &hcan2 )
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case CAN_CATCH_PITCH_L_ID:
			case CAN_CATCH_PITCH_R_ID:
			case CAN_CATCH_ROTATION_ID:
			case CAN_3508_UP_L_ID:
			case CAN_3508_UP_R_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_catch[i], rx_data);
//				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}

			default:
			{
				break;
			}
		}
	}
}

//工程拓展版
#include "pc_usart_task.h"

extern ExpandBoard_t   expand_tx;	//拓展板数据
void CAN_cmd_expand( void )
{
    uint32_t send_mail_box;
    expand_tx_message.StdId = EXPAND_CAN_RX_ID;
    expand_tx_message.IDE = CAN_ID_STD;
    expand_tx_message.RTR = CAN_RTR_DATA;
    expand_tx_message.DLC = 0x08;
    expand_can_send_data[0] = expand_tx.servo1_angle;
    expand_can_send_data[1] = expand_tx.servo2_angle;
    expand_can_send_data[2] = expand_tx.servo3_angle;
    expand_can_send_data[3] = expand_tx.servo4_angle;
    expand_can_send_data[4] = expand_tx.gas_sta;
    expand_can_send_data[5] = expand_tx.infrared_sta;	//红外
    expand_can_send_data[6] = expand_tx.motor_sta;		//保留
    expand_can_send_data[7] = COMM_CONTROL;
    HAL_CAN_AddTxMessage(&EXPAND_CAN, &expand_tx_message, expand_can_send_data, &send_mail_box);
}

//升降
void CAN_cmd_up(int16_t up_l, int16_t up_r, int16_t rev, int16_t rev_1)
{
    uint32_t send_mail_box;
    up_tx_message.StdId = CAN_UP_DOWN_ALL_ID;
    up_tx_message.IDE = CAN_ID_STD;
    up_tx_message.RTR = CAN_RTR_DATA;
    up_tx_message.DLC = 0x08;
    up_can_send_data[0] = (up_l >> 8);
    up_can_send_data[1] = up_l;
    up_can_send_data[2] = (up_r >> 8);
    up_can_send_data[3] = up_r;
    up_can_send_data[4] = (rev >> 8);
    up_can_send_data[5] = rev;
    up_can_send_data[6] = (rev_1 >> 8);
    up_can_send_data[7] = rev_1;
    HAL_CAN_AddTxMessage(&UP_CAN, &up_tx_message, up_can_send_data, &send_mail_box);
}

//抓手
void CAN_cmd_catch(int16_t catch_l, int16_t catch_r, int16_t rotation, int16_t rev)
{
    uint32_t send_mail_box;
    catch_tx_message.StdId = CAN_CATCH_ALL_ID;
    catch_tx_message.IDE = CAN_ID_STD;
    catch_tx_message.RTR = CAN_RTR_DATA;
    catch_tx_message.DLC = 0x08;
    catch_can_send_data[0] = (catch_l >> 8);
    catch_can_send_data[1] = catch_l;
    catch_can_send_data[2] = (catch_r >> 8);
    catch_can_send_data[3] = catch_r;
    catch_can_send_data[4] = (rotation >> 8);
    catch_can_send_data[5] = rotation;
    catch_can_send_data[6] = (rev >> 8);
    catch_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&CATCH_CAN, &catch_tx_message, catch_can_send_data, &send_mail_box);
}

//障碍块、救援
void CAN_cmd_user(int16_t barrier_l, int16_t barrier_r, int16_t saver_l, int16_t saver_r)
{
    uint32_t send_mail_box;
    user_tx_message.StdId = CAN_USER_ALL_ID;
    user_tx_message.IDE = CAN_ID_STD;
    user_tx_message.RTR = CAN_RTR_DATA;
    user_tx_message.DLC = 0x08;
    user_can_send_data[0] = (barrier_l >> 8);
    user_can_send_data[1] = barrier_l;
    user_can_send_data[2] = (barrier_r >> 8);
    user_can_send_data[3] = barrier_r;
    user_can_send_data[4] = (saver_l >> 8);
    user_can_send_data[5] = saver_l;
    user_can_send_data[6] = (saver_r >> 8);
    user_can_send_data[7] = saver_r;
    HAL_CAN_AddTxMessage(&USER_CAN, &user_tx_message, user_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_barrier_l_motor_measure_point(void)
{
	return &motor_chassis[4];
}

const motor_measure_t *get_barrier_r_motor_measure_point(void)
{
	return &motor_chassis[5];
}

const motor_measure_t *get_saver_l_motor_measure_point(void)
{
	return &motor_chassis[6];
}

const motor_measure_t *get_saver_r_motor_measure_point(void)
{
	return &motor_chassis[7];
}

const motor_measure_t *get_catch_pitch_l_motor_measure_point(void)
{
	return &motor_catch[0];
}

const motor_measure_t *get_catch_pitch_r_motor_measure_point(void)
{
	return &motor_catch[1];
}

const motor_measure_t *get_rotation_motor_measure_point(void)
{
	return &motor_catch[2];
}

const motor_measure_t *get_up_l_motor_measure_point(void)
{
	return &motor_catch[4];
}

const motor_measure_t *get_up_r_motor_measure_point(void)
{
	return &motor_catch[5];
}
