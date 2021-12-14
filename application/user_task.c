/*
	�޸���gimbal_task
*/
#include "user_task.h"

#include "cmsis_os.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "pid.h"

#include "shoot.h"
#include "chassis_task.h"
#include "user_lib.h"

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

#define user_total_pid_clear(user_clear)                                                   \
    {                                                                                          \
        user_PID_clear(&(user_clear)->barrier_l_motor.user_motor_relative_angle_pid);   \
        PID_clear(&(user_clear)->barrier_l_motor.user_motor_speed_pid);                    \
                                                                                               \
        user_PID_clear(&(user_clear)->barrier_r_motor.user_motor_relative_angle_pid); \
        PID_clear(&(user_clear)->barrier_r_motor.user_motor_speed_pid);                  \
																								\
        user_PID_clear(&(user_clear)->saver_l_motor.user_motor_relative_angle_pid);   \
        PID_clear(&(user_clear)->saver_l_motor.user_motor_speed_pid);                    \
                                                                                               \
        user_PID_clear(&(user_clear)->saver_r_motor.user_motor_relative_angle_pid); \
        PID_clear(&(user_clear)->saver_r_motor.user_motor_speed_pid);                  \
    }
	
static void user_init(user_control_t *init);
static void user_feedback_update(user_control_t *feedback_update);
static void user_set_control(user_control_t *set_control);
static void user_relative_angle_limit(user_motor_t *user_motor, fp32 add);
static void user_control_loop(user_control_t *control_loop);
static void user_motor_relative_angle_control(user_motor_t *user_motor);

static void user_PID_init(user_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 user_PID_calc(user_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void user_PID_clear(user_PID_t *user_pid_clear);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t user_high_water;
#endif

//�ϰ���3508�˶�����
user_control_t user_control;

/**
  * @brief          �ϰ��顢�������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void user_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(USER_TASK_INIT_TIME);
	user_init(&user_control);
	
    while (1)
    {
		user_feedback_update(&user_control);
		user_set_control(&user_control);                 //������̨������
		user_control_loop(&user_control);

//		if (toe_is_error(DBUS_TOE))
//		{
//			CAN_cmd_user(0, 0, 0, 0);
//		}
//		else
		{
			CAN_cmd_user(user_control.barrier_l_motor.given_current, 
							user_control.barrier_r_motor.given_current,
								user_control.saver_l_motor.given_current, 
									user_control.saver_r_motor.given_current);
		}
			
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(USER_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        user_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void user_init(user_control_t *init)
{
	//�ٶ�PID������3508�����ø�
    static const fp32 Speed_pid[3] = {USER_SPEED_PID_KP, USER_SPEED_PID_KI, USER_SPEED_PID_KD};
    
	//�������ָ���ȡ
    init->barrier_l_motor.user_motor_measure = get_barrier_l_motor_measure_point();
	init->barrier_r_motor.user_motor_measure = get_barrier_r_motor_measure_point();
    init->saver_l_motor.user_motor_measure = get_saver_l_motor_measure_point();
	init->saver_r_motor.user_motor_measure = get_saver_r_motor_measure_point();
	
    //ң��������ָ���ȡ
    init->user_RC = get_remote_control_point();

//    //��ʼ�����ģʽ
//    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    
	//��ʼ���ϰ���pitch���pid
    user_PID_init(&init->barrier_l_motor.user_motor_relative_angle_pid, BARRIER_ENCODE_RELATIVE_PID_MAX_OUT, BARRIER_ENCODE_RELATIVE_PID_MAX_IOUT, BARRIER_ENCODE_RELATIVE_PID_KP, BARRIER_ENCODE_RELATIVE_PID_KI, BARRIER_ENCODE_RELATIVE_PID_KD);
    user_PID_init(&init->barrier_r_motor.user_motor_relative_angle_pid, BARRIER_ENCODE_RELATIVE_PID_MAX_OUT, BARRIER_ENCODE_RELATIVE_PID_MAX_IOUT, BARRIER_ENCODE_RELATIVE_PID_KP, BARRIER_ENCODE_RELATIVE_PID_KI, BARRIER_ENCODE_RELATIVE_PID_KD);
	PID_init(&init->barrier_l_motor.user_motor_speed_pid, PID_POSITION, Speed_pid, USER_SPEED_PID_MAX_OUT, USER_SPEED_PID_MAX_IOUT);
	PID_init(&init->barrier_r_motor.user_motor_speed_pid, PID_POSITION, Speed_pid, USER_SPEED_PID_MAX_OUT, USER_SPEED_PID_MAX_IOUT);

	//��ʼ����ԮPID
    user_PID_init(&init->saver_l_motor.user_motor_relative_angle_pid, SAVER_ENCODE_RELATIVE_PID_MAX_OUT, SAVER_ENCODE_RELATIVE_PID_MAX_IOUT, SAVER_ENCODE_RELATIVE_PID_KP, SAVER_ENCODE_RELATIVE_PID_KI, SAVER_ENCODE_RELATIVE_PID_KD);
    user_PID_init(&init->saver_r_motor.user_motor_relative_angle_pid, SAVER_ENCODE_RELATIVE_PID_MAX_OUT, SAVER_ENCODE_RELATIVE_PID_MAX_IOUT, SAVER_ENCODE_RELATIVE_PID_KP, SAVER_ENCODE_RELATIVE_PID_KI, SAVER_ENCODE_RELATIVE_PID_KD);
	PID_init(&init->saver_l_motor.user_motor_speed_pid, PID_POSITION, Speed_pid, USER_SPEED_PID_MAX_OUT, USER_SPEED_PID_MAX_IOUT);
	PID_init(&init->saver_r_motor.user_motor_speed_pid, PID_POSITION, Speed_pid, USER_SPEED_PID_MAX_OUT, USER_SPEED_PID_MAX_IOUT);	
	
    //�������PID
    user_total_pid_clear(init);

	//����
	init->barrier_l_motor.ecd_count = 0;
	init->barrier_r_motor.ecd_count = 0;
	init->saver_l_motor.ecd_count = 0;
	init->saver_r_motor.ecd_count = 0;
	
	user_feedback_update(init);
	user_feedback_update(init);
	user_feedback_update(init);
	
	//�޷�
	init->barrier_l_motor.max_relative_angle = init->barrier_l_motor.relative_angle + BARRIER_L_MAX_ANGLE;
	init->barrier_l_motor.min_relative_angle = init->barrier_l_motor.relative_angle + BARRIER_L_MIN_ANGLE;
	init->barrier_r_motor.max_relative_angle = init->barrier_r_motor.relative_angle + BARRIER_R_MAX_ANGLE; 
	init->barrier_r_motor.min_relative_angle = init->barrier_r_motor.relative_angle + BARRIER_R_MIN_ANGLE;
	//
	init->saver_l_motor.max_relative_angle = init->saver_l_motor.relative_angle + SAVER_L_MAX_ANGLE;
	init->saver_l_motor.min_relative_angle = init->saver_l_motor.relative_angle + SAVER_L_MIN_ANGLE;
	init->saver_r_motor.max_relative_angle = init->saver_r_motor.relative_angle + SAVER_R_MAX_ANGLE;
	init->saver_r_motor.min_relative_angle = init->saver_r_motor.relative_angle + SAVER_R_MIN_ANGLE;

	//��ʼ�Ƕȡ��ٶ� ����
    init->barrier_l_motor.relative_angle_set = init->barrier_l_motor.min_relative_angle;//init->barrier_l_motor.relative_angle;
    init->barrier_l_motor.motor_speed_set = init->barrier_l_motor.motor_speed;

    init->barrier_r_motor.relative_angle_set = init->barrier_r_motor.max_relative_angle;//init->barrier_r_motor.relative_angle;
    init->barrier_r_motor.motor_speed_set = init->barrier_r_motor.motor_speed;
	
	//
    init->saver_l_motor.relative_angle_set = init->saver_l_motor.max_relative_angle;
    init->saver_l_motor.motor_speed_set = init->saver_l_motor.motor_speed;

    init->saver_r_motor.relative_angle_set = init->saver_r_motor.min_relative_angle;
    init->saver_r_motor.motor_speed_set = init->saver_r_motor.motor_speed;
	
	//б�º���
	ramp_init(&init->barrier_l_ramp, USER_CONTROL_TIME_MS, init->barrier_l_motor.max_relative_angle,init->barrier_l_motor.min_relative_angle);
	ramp_init(&init->barrier_r_ramp, USER_CONTROL_TIME_MS, init->barrier_r_motor.max_relative_angle,init->barrier_r_motor.min_relative_angle);
	ramp_init(&init->saver_l_ramp, USER_CONTROL_TIME_MS, init->saver_l_motor.max_relative_angle,init->saver_l_motor.min_relative_angle);
	ramp_init(&init->saver_r_ramp, USER_CONTROL_TIME_MS, init->saver_r_motor.max_relative_angle,init->saver_r_motor.min_relative_angle);

}

static void user_feedback_update(user_control_t *feedback_update)//motor_gyro:���ٶ� rad/s
{
    if (feedback_update == NULL)
    {
        return;
    }
	//��̨������motor_gyroԭ���Ƕ�ȡ�����ǽ��ٶȣ������õ��������rpm�������ٶ�
	
	//���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 19Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
	if (feedback_update->barrier_l_motor.user_motor_measure->ecd - feedback_update->barrier_l_motor.user_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		feedback_update->barrier_l_motor.ecd_count--;	
	}
	else if (feedback_update->barrier_l_motor.user_motor_measure->ecd - feedback_update->barrier_l_motor.user_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		feedback_update->barrier_l_motor.ecd_count++;
	}
	if (feedback_update->barrier_r_motor.user_motor_measure->ecd - feedback_update->barrier_r_motor.user_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		feedback_update->barrier_r_motor.ecd_count--;	
	}
	else if (feedback_update->barrier_r_motor.user_motor_measure->ecd - feedback_update->barrier_r_motor.user_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		feedback_update->barrier_r_motor.ecd_count++;
	}
	
	if (feedback_update->saver_l_motor.user_motor_measure->ecd - feedback_update->saver_l_motor.user_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		feedback_update->saver_l_motor.ecd_count--;	
	}
	else if (feedback_update->saver_l_motor.user_motor_measure->ecd - feedback_update->saver_l_motor.user_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		feedback_update->saver_l_motor.ecd_count++;
	}
	
	if (feedback_update->saver_r_motor.user_motor_measure->ecd - feedback_update->saver_r_motor.user_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		feedback_update->saver_r_motor.ecd_count--;	
	}
	else if (feedback_update->saver_r_motor.user_motor_measure->ecd - feedback_update->saver_r_motor.user_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		feedback_update->saver_r_motor.ecd_count++;
	}
    //��̨���ݸ���

	//�ϰ���pitch
    feedback_update->barrier_l_motor.relative_angle = (feedback_update->barrier_l_motor.ecd_count * ECD_RANGE 
																+ feedback_update->barrier_l_motor.user_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
	feedback_update->barrier_r_motor.relative_angle = (feedback_update->barrier_r_motor.ecd_count * ECD_RANGE 
																+ feedback_update->barrier_r_motor.user_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->saver_l_motor.relative_angle = (feedback_update->saver_l_motor.ecd_count * ECD_RANGE 
																+ feedback_update->saver_l_motor.user_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->saver_r_motor.relative_angle = (feedback_update->saver_r_motor.ecd_count * ECD_RANGE 
																+ feedback_update->saver_r_motor.user_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;	
	
	feedback_update->barrier_l_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->barrier_l_motor.user_motor_measure->speed_rpm;
    feedback_update->barrier_r_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->barrier_r_motor.user_motor_measure->speed_rpm;
	feedback_update->saver_l_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->saver_l_motor.user_motor_measure->speed_rpm;
    feedback_update->saver_r_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->saver_r_motor.user_motor_measure->speed_rpm;
}

static void user_set_control(user_control_t *set_control)
{
	if (set_control == NULL)
    {
        return;
    }
	
	//ң��������
	fp32 add_barrier_angle = 0.0f;	//�ϰ���
	fp32 add_saver_angle = 0.0f;	//��Ԯ
	
	if( switch_is_mid(set_control->user_RC->rc.s[USER_MODE_CHANNEL]) )//�ϰ���;�Ԯ
	{
		rc_deadband_limit(set_control->user_RC->rc.ch[BARRIER_PITCH_CHANNEL], add_barrier_angle, RC_DEADBAND);
		add_barrier_angle = add_barrier_angle * BARRIER_PITCH_RC_SEN;	// + set_control->user_RC->mouse.y * PITCH_MOUSE_SEN;
		
		rc_deadband_limit(set_control->user_RC->rc.ch[SAVER_CHANNEL], add_saver_angle, RC_DEADBAND);
		add_saver_angle = add_saver_angle * SAVER_RC_SEN;	// + set_control->user_RC->mouse.y * PITCH_MOUSE_SEN;	
		
		//encondeģʽ�£��������Ƕȿ���
		//���λ�øı����Ҫ�޸�add
		user_relative_angle_limit(&set_control->barrier_l_motor, add_barrier_angle);
		user_relative_angle_limit(&set_control->barrier_r_motor, -add_barrier_angle);
		user_relative_angle_limit(&set_control->saver_l_motor, -add_saver_angle);
		user_relative_angle_limit(&set_control->saver_r_motor, add_saver_angle);
	}
	
	//���̿���
	static uint8_t barrier_flag = 0;	//Ĭ�Ϸ����ϰ���
//	static uint8_t saver_flag = 0;		//Ĭ�Ϸ��¾�Ԯ
	static uint16_t time = 0;			//״̬�ı� ��ʱ
	static uint8_t time_flag = 0;		//״̬�ı� ��ʱflag
	
	if( time == 500 )	//	1sʱ�䵽
	{
		time = 0;
		time_flag = 0;
	}
	if( time_flag == 1 )
	{
		time++;
		return ;
	}
	
	//���λ�øı����Ҫ�޸�
	//�ϰ���
	if( (set_control->user_RC->key.v & BARRIER_KEY) && (barrier_flag == 0))
	{
		barrier_flag = 1;
		time_flag = 1;
	}
	else if( (set_control->user_RC->key.v & BARRIER_KEY) && (barrier_flag == 1))
	{
		barrier_flag = 0;
		time_flag = 1;
	}
	if( barrier_flag == 1 )
	{
		ramp_calc(&set_control->barrier_l_ramp,BARRIER_L_ADD_VALUE);
		ramp_calc(&set_control->barrier_r_ramp,-BARRIER_R_ADD_VALUE);
		set_control->barrier_l_motor.relative_angle_set = set_control->barrier_l_ramp.out;
		set_control->barrier_r_motor.relative_angle_set = set_control->barrier_r_ramp.out;
	}
	else if( barrier_flag == 0 )
	{
		ramp_calc(&set_control->barrier_l_ramp,-BARRIER_L_ADD_VALUE);
		ramp_calc(&set_control->barrier_r_ramp,BARRIER_R_ADD_VALUE);
		set_control->barrier_l_motor.relative_angle_set = set_control->barrier_l_ramp.out;
		set_control->barrier_r_motor.relative_angle_set = set_control->barrier_r_ramp.out;
	}
	
	//��Ԯ
//	if( (set_control->user_RC->key.v & SAVER_KEY) && (saver_flag == 0))
//	{
//		saver_flag = 1;
//		time_flag = 1;
//	}
//	else if( (set_control->user_RC->key.v & SAVER_KEY) && (saver_flag == 1))
//	{
//		saver_flag = 0;
//		time_flag = 1;
//	}
//	if( saver_flag == 1 )
//	{
//		ramp_calc(&set_control->saver_l_ramp, -SAVER_L_ADD_VALUE);
//		ramp_calc(&set_control->saver_r_ramp, SAVER_R_ADD_VALUE);
//		set_control->saver_l_motor.relative_angle_set = set_control->saver_l_ramp.out;
//		set_control->saver_r_motor.relative_angle_set = set_control->saver_r_ramp.out;			
//	}
//	else if( saver_flag == 0 )
//	{
//		ramp_calc(&set_control->saver_l_ramp, SAVER_L_ADD_VALUE);
//		ramp_calc(&set_control->saver_r_ramp, -SAVER_R_ADD_VALUE);
//		set_control->saver_l_motor.relative_angle_set = set_control->saver_l_ramp.out;
//		set_control->saver_r_motor.relative_angle_set = set_control->saver_r_ramp.out;
//	}

	if( (set_control->user_RC->key.v & SAVER_KEY) && (set_control->user_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)==0 )
	{
		ramp_calc(&set_control->saver_l_ramp, -SAVER_L_ADD_VALUE);
		ramp_calc(&set_control->saver_r_ramp, SAVER_R_ADD_VALUE);
		set_control->saver_l_motor.relative_angle_set = set_control->saver_l_ramp.out;
		set_control->saver_r_motor.relative_angle_set = set_control->saver_r_ramp.out;	
	}
	else if( (set_control->user_RC->key.v & SAVER_KEY) && (set_control->user_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) )
	{
		ramp_calc(&set_control->saver_l_ramp, SAVER_L_ADD_VALUE);
		ramp_calc(&set_control->saver_r_ramp, -SAVER_R_ADD_VALUE);
		set_control->saver_l_motor.relative_angle_set = set_control->saver_l_ramp.out;
		set_control->saver_r_motor.relative_angle_set = set_control->saver_r_ramp.out;

	}
}

static void user_control_loop(user_control_t *control_loop)
{
	if (control_loop == NULL)
    {
        return;
    }
	
	user_motor_relative_angle_control(&control_loop->barrier_l_motor);
	user_motor_relative_angle_control(&control_loop->barrier_r_motor);
	user_motor_relative_angle_control(&control_loop->saver_l_motor);
	user_motor_relative_angle_control(&control_loop->saver_r_motor);
}

static void user_motor_relative_angle_control(user_motor_t *user_motor)
{
    if (user_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
    user_motor->motor_speed_set = user_PID_calc(&user_motor->user_motor_relative_angle_pid, user_motor->relative_angle, user_motor->relative_angle_set, user_motor->motor_speed);
    user_motor->current_set = PID_calc(&user_motor->user_motor_speed_pid, user_motor->motor_speed, user_motor->motor_speed_set);
    //����ֵ��ֵ
    user_motor->given_current = (int16_t)(user_motor->current_set);
}

static void user_relative_angle_limit(user_motor_t *user_motor, fp32 add)
{
    if (user_motor == NULL)
    {
        return;
    }
    user_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
    if (user_motor->relative_angle_set > user_motor->max_relative_angle)
    {
        user_motor->relative_angle_set = user_motor->max_relative_angle;
    }
    else if (user_motor->relative_angle_set < user_motor->min_relative_angle)
    {
        user_motor->relative_angle_set = user_motor->min_relative_angle;
    }
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, user motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void user_PID_init(user_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 user_PID_calc(user_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
static void user_PID_clear(user_PID_t *user_pid_clear)
{
    if (user_pid_clear == NULL)
    {
        return;
    }
    user_pid_clear->err = user_pid_clear->set = user_pid_clear->get = 0.0f;
    user_pid_clear->out = user_pid_clear->Pout = user_pid_clear->Iout = user_pid_clear->Dout = 0.0f;
}
