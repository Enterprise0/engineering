/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#include "pc_usart_task.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
	
static uint16_t Angle_To_PWM( fp32 angle );

const RC_ctrl_t *servo_rc;

fp32 gimbal_pitch_angle = 120;
//底盘任务中，底盘跟随云台模式要用到yaw轴角度
fp32 gimbal_yaw_angle = 0;		//77;	

	
extern ExpandBoard_t   expand_tx;	//拓展板数据

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
	vTaskDelay(357);
	
//    servo_rc = get_remote_control_point();
	uint16_t pitch_pwm = 0;
	
	while(1)
    {
//		fp32 add_yaw_channel = 0;
//		fp32 add_pitch_channel = 0;

//		if( switch_is_up(servo_rc->rc.s[0]) )
//		{
//			rc_deadband_limit(servo_rc->rc.ch[SERVO_YAW_CHANNEL], add_yaw_channel, SERVO_RC_DEADBAND);
//			rc_deadband_limit(servo_rc->rc.ch[SERVO_PITCH_CHANNEL], add_pitch_channel, SERVO_RC_DEADBAND);
//		}
//	
//		gimbal_yaw_angle += (add_yaw_channel * SERVO_YAW_RC_SEN - servo_rc->mouse.x * SERVO_YAW_MOUSE_SEN);
//		gimbal_pitch_angle += (add_pitch_channel * SERVO_PITCH_RC_SEN + servo_rc->mouse.y * SERVO_PITCH_MOUSE_SEN);

//		//limit the pwm
//		//限制角度
//		if( gimbal_yaw_angle > 180.0f )
//			gimbal_yaw_angle = 180.0f;
//		else if( gimbal_yaw_angle < 0.0f )
//			gimbal_yaw_angle = 0.0f;
//		
//		if( gimbal_pitch_angle > 180.0f )
//			gimbal_pitch_angle = 180.0f;
//		else if( gimbal_pitch_angle < 0.0f )
//			gimbal_pitch_angle = 0.0f;
//        
//		pitch_pwm = Angle_To_PWM(77);
//		
//		expand_tx.servo1_angle = (uint8_t)gimbal_yaw_angle;
//		expand_tx.servo2_angle = (uint8_t)gimbal_pitch_angle;
		
		servo_pwm_set(Angle_To_PWM(90), 0);
		vTaskDelay(1000);
		servo_pwm_set(Angle_To_PWM(180), 0);
		vTaskDelay(1000);
		
        vTaskDelay(1);
    }
}

static uint16_t Angle_To_PWM( fp32 angle )
{
	fp32 temp = 0;
	
	temp = angle * 11;
	temp += 500;
	
	return	(uint16_t)temp;
}
