/*
	修改自barrier_task
*/
#include "catch_task.h"

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

#define catch_total_pid_clear(catch_clear)                                                   \
    {                                                                                          \
        catch_PID_clear(&(catch_clear)->catch_pitch_l_motor.catch_motor_relative_angle_pid);   \
        PID_clear(&(catch_clear)->catch_pitch_l_motor.catch_motor_speed_pid);                    \
                                                                                               \
        catch_PID_clear(&(catch_clear)->catch_pitch_r_motor.catch_motor_relative_angle_pid); \
        PID_clear(&(catch_clear)->catch_pitch_r_motor.catch_motor_speed_pid);                  \
																								\
        catch_PID_clear(&(catch_clear)->catch_rotation_motor.catch_motor_relative_angle_pid);   \
        PID_clear(&(catch_clear)->catch_rotation_motor.catch_motor_speed_pid);                    \
																								\
        catch_PID_clear(&(catch_clear)->catch_up_l_motor.catch_motor_relative_angle_pid);   \
        PID_clear(&(catch_clear)->catch_up_l_motor.catch_motor_speed_pid);                    \
																								\
        catch_PID_clear(&(catch_clear)->catch_up_r_motor.catch_motor_relative_angle_pid);   \
        PID_clear(&(catch_clear)->catch_up_r_motor.catch_motor_speed_pid);                    \
    }

static void catch_init(catch_control_t *init);
static void catch_feedback_update(catch_control_t *feedback_update);
static void catch_set_control(catch_control_t *set_control);
static void catch_relative_angle_limit(catch_motor_t *user_motor, fp32 add);
static void catch_control_loop(catch_control_t *control_loop);
static void catch_motor_relative_angle_control(catch_motor_t *user_motor);

static void catch_PID_init(catch_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 catch_PID_calc(catch_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void catch_PID_clear(catch_PID_t *user_pid_clear);

static void gas_set_control(void);
static void radio_set_control(void);//设置倒车雷达		
	
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t catch_high_water;
#endif

//抓手3508运动数据
catch_control_t catch_control;

//遥控器指针
const RC_ctrl_t *gas_rc;

extern ExpandBoard_t   expand_tx;	//拓展板数据

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void catch_task(void const *pvParameters)
{
    //wait a time
    //空闲一段时间
    vTaskDelay(CATCH_TASK_INIT_TIME);
    catch_init(&catch_control);

	gas_rc = get_remote_control_point();
	
    while (1)
    {
        catch_feedback_update(&catch_control);
        catch_set_control(&catch_control);                 //设置控制量
        catch_control_loop(&catch_control);
		
		gas_set_control();
		radio_set_control(); //设置倒车雷达
		
//        if (toe_is_error(DBUS_TOE))
//        {
//            CAN_cmd_catch(0, 0, 0, 0);
//        }
//        else
        {
            CAN_cmd_catch(catch_control.catch_pitch_l_motor.given_current,
                          catch_control.catch_pitch_r_motor.given_current,
                          catch_control.catch_rotation_motor.given_current,
                          0);
            CAN_cmd_up(catch_control.catch_up_l_motor.given_current,
						catch_control.catch_up_r_motor.given_current,
							0,0
                       );
        }

        //os delay
        //系统延时
        vTaskDelay(CATCH_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        catch_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void catch_init(catch_control_t *init)
{
    //速度PID，都是3508，不用改
    static const fp32 Speed_pid[3] = {CATCH_SPEED_PID_KP, CATCH_SPEED_PID_KI, CATCH_SPEED_PID_KD};

    //电机数据指针获取
    init->catch_pitch_l_motor.catch_motor_measure = get_catch_pitch_l_motor_measure_point();
    init->catch_pitch_r_motor.catch_motor_measure = get_catch_pitch_r_motor_measure_point();
    init->catch_rotation_motor.catch_motor_measure = get_rotation_motor_measure_point();
    init->catch_up_l_motor.catch_motor_measure = get_up_l_motor_measure_point();
    init->catch_up_r_motor.catch_motor_measure = get_up_r_motor_measure_point();

    //遥控器数据指针获取
    init->catch_RC = get_remote_control_point();

//    //初始化电机模式
//    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化抓手pitch电机pid
    catch_PID_init(&init->catch_pitch_l_motor.catch_motor_relative_angle_pid, CATCH_ENCODE_RELATIVE_PID_MAX_OUT, CATCH_ENCODE_RELATIVE_PID_MAX_IOUT, CATCH_ENCODE_RELATIVE_PID_KP, CATCH_ENCODE_RELATIVE_PID_KI, CATCH_ENCODE_RELATIVE_PID_KD);
    catch_PID_init(&init->catch_pitch_r_motor.catch_motor_relative_angle_pid, CATCH_ENCODE_RELATIVE_PID_MAX_OUT, CATCH_ENCODE_RELATIVE_PID_MAX_IOUT, CATCH_ENCODE_RELATIVE_PID_KP, CATCH_ENCODE_RELATIVE_PID_KI, CATCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->catch_pitch_l_motor.catch_motor_speed_pid, PID_POSITION, Speed_pid, CATCH_SPEED_PID_MAX_OUT, CATCH_SPEED_PID_MAX_IOUT);
    PID_init(&init->catch_pitch_r_motor.catch_motor_speed_pid, PID_POSITION, Speed_pid, CATCH_SPEED_PID_MAX_OUT, CATCH_SPEED_PID_MAX_IOUT);

    //初始化旋转PID
    catch_PID_init(&init->catch_rotation_motor.catch_motor_relative_angle_pid, ROTATION_ENCODE_RELATIVE_PID_MAX_OUT, ROTATION_ENCODE_RELATIVE_PID_MAX_IOUT, ROTATION_ENCODE_RELATIVE_PID_KP, CATCH_ENCODE_RELATIVE_PID_KI, CATCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->catch_rotation_motor.catch_motor_speed_pid, PID_POSITION, Speed_pid, CATCH_SPEED_PID_MAX_OUT, CATCH_SPEED_PID_MAX_IOUT);

    //初始化升降PID
    catch_PID_init(&init->catch_up_l_motor.catch_motor_relative_angle_pid, UP_ENCODE_RELATIVE_PID_MAX_OUT, UP_ENCODE_RELATIVE_PID_MAX_IOUT, UP_ENCODE_RELATIVE_PID_KP, UP_ENCODE_RELATIVE_PID_KI, UP_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->catch_up_l_motor.catch_motor_speed_pid, PID_POSITION, Speed_pid, CATCH_SPEED_PID_MAX_OUT, CATCH_SPEED_PID_MAX_IOUT);
    catch_PID_init(&init->catch_up_r_motor.catch_motor_relative_angle_pid, UP_ENCODE_RELATIVE_PID_MAX_OUT, UP_ENCODE_RELATIVE_PID_MAX_IOUT, UP_ENCODE_RELATIVE_PID_KP, UP_ENCODE_RELATIVE_PID_KI, UP_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->catch_up_r_motor.catch_motor_speed_pid, PID_POSITION, Speed_pid, CATCH_SPEED_PID_MAX_OUT, CATCH_SPEED_PID_MAX_IOUT);

    //清除所有PID
    catch_total_pid_clear(init);

    //清零
    init->catch_pitch_l_motor.ecd_count = 0;
    init->catch_pitch_r_motor.ecd_count = 0;
    init->catch_rotation_motor.ecd_count = 0;
    init->catch_up_l_motor.ecd_count = 0;
    init->catch_up_r_motor.ecd_count = 0;

    catch_feedback_update(init);

    //角度 限幅
    init->catch_pitch_l_motor.max_relative_angle = init->catch_pitch_l_motor.relative_angle + CATCH_L_MAX_ANGLE;
    init->catch_pitch_l_motor.min_relative_angle = init->catch_pitch_l_motor.relative_angle + CATCH_L_MIN_ANGLE;
    init->catch_pitch_r_motor.max_relative_angle = init->catch_pitch_r_motor.relative_angle + CATCH_R_MAX_ANGLE;
    init->catch_pitch_r_motor.min_relative_angle = init->catch_pitch_r_motor.relative_angle + CATCH_R_MIN_ANGLE;
    //
    init->catch_rotation_motor.max_relative_angle = init->catch_rotation_motor.relative_angle + ROTATION_MAX_ANGLE;
    init->catch_rotation_motor.min_relative_angle = init->catch_rotation_motor.relative_angle + ROTATION_MIN_ANGLE;
    //
    init->catch_up_l_motor.max_relative_angle = init->catch_up_l_motor.relative_angle + UP_L_MAX_ANGLE;
    init->catch_up_l_motor.min_relative_angle = init->catch_up_l_motor.relative_angle + UP_L_MIN_ANGLE;
    init->catch_up_r_motor.max_relative_angle = init->catch_up_r_motor.relative_angle + UP_R_MAX_ANGLE;
    init->catch_up_r_motor.min_relative_angle = init->catch_up_r_motor.relative_angle + UP_R_MIN_ANGLE;

    //初始 速度、角度 设置
    init->catch_pitch_l_motor.relative_angle_set = init->catch_pitch_l_motor.relative_angle;
    init->catch_pitch_l_motor.motor_speed_set = init->catch_pitch_l_motor.motor_speed;
    init->catch_pitch_r_motor.relative_angle_set = init->catch_pitch_r_motor.relative_angle;
    init->catch_pitch_r_motor.motor_speed_set = init->catch_pitch_r_motor.motor_speed;

    //
    init->catch_rotation_motor.relative_angle_set = init->catch_rotation_motor.relative_angle;
    init->catch_rotation_motor.motor_speed_set = init->catch_rotation_motor.motor_speed;

    //
    init->catch_up_l_motor.relative_angle_set = init->catch_up_l_motor.relative_angle;
    init->catch_up_l_motor.motor_speed_set = init->catch_up_l_motor.motor_speed;
    init->catch_up_r_motor.relative_angle_set = init->catch_up_r_motor.relative_angle;
    init->catch_up_r_motor.motor_speed_set = init->catch_up_r_motor.motor_speed;
	
	//斜坡
	ramp_init(&init->catch_pitch_l_ramp, CATCH_CONTROL_TIME_MS , init->catch_pitch_l_motor.max_relative_angle,init->catch_pitch_l_motor.min_relative_angle);
	ramp_init(&init->catch_pitch_r_ramp, CATCH_CONTROL_TIME_MS, init->catch_pitch_r_motor.max_relative_angle,init->catch_pitch_r_motor.min_relative_angle);
	ramp_init(&init->catch_rotation_ramp, CATCH_CONTROL_TIME_MS , init->catch_rotation_motor.max_relative_angle,init->catch_rotation_motor.min_relative_angle);
	ramp_init(&init->catch_up_l_ramp, CATCH_CONTROL_TIME_MS, init->catch_up_l_motor.max_relative_angle,init->catch_up_l_motor.min_relative_angle);
	ramp_init(&init->catch_up_r_ramp, CATCH_CONTROL_TIME_MS, init->catch_up_r_motor.max_relative_angle,init->catch_up_r_motor.min_relative_angle);
	
}

static void catch_feedback_update(catch_control_t *feedback_update)//motor_gyro:角速度 rad/s
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台任务中motor_gyro原本是读取陀螺仪角速度，现在用电机反馈的rpm来计算速度

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 19圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (feedback_update->catch_pitch_l_motor.catch_motor_measure->ecd - feedback_update->catch_pitch_l_motor.catch_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        feedback_update->catch_pitch_l_motor.ecd_count--;
    }
    else if (feedback_update->catch_pitch_l_motor.catch_motor_measure->ecd - feedback_update->catch_pitch_l_motor.catch_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        feedback_update->catch_pitch_l_motor.ecd_count++;
    }
    if (feedback_update->catch_pitch_r_motor.catch_motor_measure->ecd - feedback_update->catch_pitch_r_motor.catch_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        feedback_update->catch_pitch_r_motor.ecd_count--;
    }
    else if (feedback_update->catch_pitch_r_motor.catch_motor_measure->ecd - feedback_update->catch_pitch_r_motor.catch_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        feedback_update->catch_pitch_r_motor.ecd_count++;
    }

    if (feedback_update->catch_rotation_motor.catch_motor_measure->ecd - feedback_update->catch_rotation_motor.catch_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        feedback_update->catch_rotation_motor.ecd_count--;
    }
    else if (feedback_update->catch_rotation_motor.catch_motor_measure->ecd - feedback_update->catch_rotation_motor.catch_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        feedback_update->catch_rotation_motor.ecd_count++;
    }

    if (feedback_update->catch_up_l_motor.catch_motor_measure->ecd - feedback_update->catch_up_l_motor.catch_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        feedback_update->catch_up_l_motor.ecd_count--;
    }
    else if (feedback_update->catch_up_l_motor.catch_motor_measure->ecd - feedback_update->catch_up_l_motor.catch_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        feedback_update->catch_up_l_motor.ecd_count++;
    }
    if (feedback_update->catch_up_r_motor.catch_motor_measure->ecd - feedback_update->catch_up_r_motor.catch_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        feedback_update->catch_up_r_motor.ecd_count--;
    }
    else if (feedback_update->catch_up_r_motor.catch_motor_measure->ecd - feedback_update->catch_up_r_motor.catch_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        feedback_update->catch_up_r_motor.ecd_count++;
    }
    //云台数据更新

    //障碍块pitch
    feedback_update->catch_pitch_l_motor.relative_angle = (feedback_update->catch_pitch_l_motor.ecd_count * ECD_RANGE
            + feedback_update->catch_pitch_l_motor.catch_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->catch_pitch_r_motor.relative_angle = (feedback_update->catch_pitch_r_motor.ecd_count * ECD_RANGE
            + feedback_update->catch_pitch_r_motor.catch_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->catch_rotation_motor.relative_angle = (feedback_update->catch_rotation_motor.ecd_count * ECD_RANGE
            + feedback_update->catch_rotation_motor.catch_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->catch_up_l_motor.relative_angle = (feedback_update->catch_up_l_motor.ecd_count * ECD_RANGE
            + feedback_update->catch_up_l_motor.catch_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    feedback_update->catch_up_r_motor.relative_angle = (feedback_update->catch_up_r_motor.ecd_count * ECD_RANGE
            + feedback_update->catch_up_r_motor.catch_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    feedback_update->catch_pitch_l_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->catch_pitch_l_motor.catch_motor_measure->speed_rpm;
    feedback_update->catch_pitch_r_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->catch_pitch_r_motor.catch_motor_measure->speed_rpm;
    feedback_update->catch_rotation_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->catch_rotation_motor.catch_motor_measure->speed_rpm;
    feedback_update->catch_up_l_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->catch_up_l_motor.catch_motor_measure->speed_rpm;
    feedback_update->catch_up_r_motor.motor_speed = M3508_MOTOR_RPM_TO_VECTOR * feedback_update->catch_up_r_motor.catch_motor_measure->speed_rpm;
}

static void catch_set_control(catch_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    //遥控器控制
    fp32 add_up_angle = 0.0f;			//升降
    fp32 add_catch_angle = 0.0f;		//抓手
    fp32 add_rotation_angle = 0.0f;		//矿石旋转

	static uint8_t catch_pitch_flag = 0;
	uint8_t rotation_flag = 2;
	
	
    if( switch_is_up(set_control->catch_RC->rc.s[CATCH_MODE_CHANNEL]) )
    {
        rc_deadband_limit(set_control->catch_RC->rc.ch[CATCH_UP_CHANNEL], add_up_angle, RC_DEADBAND);
        add_up_angle += add_up_angle * CATCH_UP_RC_SEN;	// + set_control->catch_RC->mouse.y * PITCH_MOUSE_SEN;

        rc_deadband_limit(set_control->catch_RC->rc.ch[CATCH_PITCH_CHANNEL], add_catch_angle, RC_DEADBAND);
        add_catch_angle += add_catch_angle * CATCH_PITCH_RC_SEN;	// + set_control->catch_RC->mouse.y * PITCH_MOUSE_SEN;

        rc_deadband_limit(set_control->catch_RC->rc.ch[ROTATION_CHANNEL], add_rotation_angle, RC_DEADBAND);
        add_rotation_angle += add_rotation_angle * ROTATION_RC_SEN;	// + set_control->catch_RC->mouse.y * PITCH_MOUSE_SEN;
    
		//enconde模式下，电机编码角度控制
		//电机位置改变后，需要修改add
		catch_relative_angle_limit(&set_control->catch_pitch_l_motor, -add_catch_angle);
		catch_relative_angle_limit(&set_control->catch_pitch_r_motor, add_catch_angle);
		catch_relative_angle_limit(&set_control->catch_rotation_motor, add_rotation_angle);

		catch_relative_angle_limit(&set_control->catch_up_l_motor, -add_up_angle);
		catch_relative_angle_limit(&set_control->catch_up_r_motor, add_up_angle);
	}

    //键盘控制
    //升降
    if( set_control->catch_RC->key.v & UP_DOWN_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)==0)
    {
		ramp_calc(&set_control->catch_up_l_ramp, -CATCH_UP_L_ADD_VALUE);
		ramp_calc(&set_control->catch_up_r_ramp, CATCH_UP_R_ADD_VALUE);
		set_control->catch_up_l_motor.relative_angle_set = set_control->catch_up_l_ramp.out;
		set_control->catch_up_r_motor.relative_angle_set = set_control->catch_up_r_ramp.out;
	}
    else if( set_control->catch_RC->key.v & UP_DOWN_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) )
	{
		ramp_calc(&set_control->catch_up_l_ramp, CATCH_UP_L_ADD_VALUE);
		ramp_calc(&set_control->catch_up_r_ramp, -CATCH_UP_R_ADD_VALUE);
		set_control->catch_up_l_motor.relative_angle_set = set_control->catch_up_l_ramp.out;
		set_control->catch_up_r_motor.relative_angle_set = set_control->catch_up_r_ramp.out;		
	}
	
    //抓手pitch
    if( set_control->catch_RC->key.v & CATCH_PITCH_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)==0)
	{
		catch_pitch_flag = 1;
		
		ramp_calc(&set_control->catch_pitch_l_ramp, -CATCH_PITCH_L_ADD_VALUE);
		ramp_calc(&set_control->catch_pitch_r_ramp, CATCH_PITCH_R_ADD_VALUE);
		set_control->catch_pitch_l_motor.relative_angle_set = set_control->catch_pitch_l_ramp.out;
		set_control->catch_pitch_r_motor.relative_angle_set = set_control->catch_pitch_r_ramp.out;			

	}
    else if( set_control->catch_RC->key.v & CATCH_PITCH_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) )
	{
		catch_pitch_flag = 0;

		ramp_calc(&set_control->catch_pitch_l_ramp, CATCH_PITCH_L_ADD_VALUE);
		ramp_calc(&set_control->catch_pitch_r_ramp, -CATCH_PITCH_R_ADD_VALUE);
		set_control->catch_pitch_l_motor.relative_angle_set = set_control->catch_pitch_l_ramp.out;
		set_control->catch_pitch_r_motor.relative_angle_set = set_control->catch_pitch_r_ramp.out;					
	}
//	if( catch_pitch_flag == 1 ) 
//	{
//		ramp_calc(&set_control->catch_pitch_l_ramp, -CATCH_PITCH_L_ADD_VALUE);
//		ramp_calc(&set_control->catch_pitch_r_ramp, CATCH_PITCH_R_ADD_VALUE);
//		set_control->catch_pitch_l_motor.relative_angle_set = set_control->catch_pitch_l_ramp.out;
//		set_control->catch_pitch_r_motor.relative_angle_set = set_control->catch_pitch_r_ramp.out;			
//	}
//	else if( catch_pitch_flag == 0 )
//	{
//		ramp_calc(&set_control->catch_pitch_l_ramp, CATCH_PITCH_L_ADD_VALUE);
//		ramp_calc(&set_control->catch_pitch_r_ramp, -CATCH_PITCH_R_ADD_VALUE);
//		set_control->catch_pitch_l_motor.relative_angle_set = set_control->catch_pitch_l_ramp.out;
//		set_control->catch_pitch_r_motor.relative_angle_set = set_control->catch_pitch_r_ramp.out;			
//	}
	
    //矿石旋转
    if( set_control->catch_RC->key.v & CATCH_ROTATION_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)==0)
	{
		rotation_flag = 1;
	}
    else if( set_control->catch_RC->key.v & CATCH_ROTATION_KEY &&(set_control->catch_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) )
	{
		rotation_flag = 0;
	}
	if( rotation_flag == 1 )
	{
		ramp_calc(&set_control->catch_rotation_ramp, CATCH_ROTATION_ADD_VALUE);
		set_control->catch_rotation_motor.relative_angle_set = set_control->catch_rotation_ramp.out;
	}
	else if( rotation_flag == 0 )
	{
		ramp_calc(&set_control->catch_rotation_ramp, -CATCH_ROTATION_ADD_VALUE);
		set_control->catch_rotation_motor.relative_angle_set = set_control->catch_rotation_ramp.out;
	}
//    //enconde模式下，电机编码角度控制
//    //电机位置改变后，需要修改add
//    catch_relative_angle_limit(&set_control->catch_pitch_l_motor, -add_catch_angle);
//    catch_relative_angle_limit(&set_control->catch_pitch_r_motor, add_catch_angle);
//    catch_relative_angle_limit(&set_control->catch_rotation_motor, add_rotation_angle);
//	
//	  catch_relative_angle_limit(&set_control->catch_up_l_motor, add_up_angle);
//    catch_relative_angle_limit(&set_control->catch_up_r_motor, -add_up_angle);
}

static void catch_control_loop(catch_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    catch_motor_relative_angle_control(&control_loop->catch_pitch_l_motor);
    catch_motor_relative_angle_control(&control_loop->catch_pitch_r_motor);
    catch_motor_relative_angle_control(&control_loop->catch_rotation_motor);
    catch_motor_relative_angle_control(&control_loop->catch_up_l_motor);
    catch_motor_relative_angle_control(&control_loop->catch_up_r_motor);
}

static void catch_motor_relative_angle_control(catch_motor_t *user_motor)
{
    if (user_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    user_motor->motor_speed_set = catch_PID_calc(&user_motor->catch_motor_relative_angle_pid, user_motor->relative_angle, user_motor->relative_angle_set, user_motor->motor_speed);
    user_motor->current_set = PID_calc(&user_motor->catch_motor_speed_pid, user_motor->motor_speed, user_motor->motor_speed_set);
//    user_motor->current_set = catch_PID_calc(&user_motor->catch_motor_relative_angle_pid, user_motor->relative_angle, user_motor->relative_angle_set, user_motor->motor_speed);
	//控制值赋值
    user_motor->given_current = (int16_t)(user_motor->current_set);
}

static void catch_relative_angle_limit(catch_motor_t *catch_motor, fp32 add)
{
    if (catch_motor == NULL)
    {
        return;
    }
    catch_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (catch_motor->relative_angle_set > catch_motor->max_relative_angle)
    {
        catch_motor->relative_angle_set = catch_motor->max_relative_angle;
    }
    else if (catch_motor->relative_angle_set < catch_motor->min_relative_angle)
    {
        catch_motor->relative_angle_set = catch_motor->min_relative_angle;
    }
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, user motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void catch_PID_init(catch_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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

static fp32 catch_PID_calc(catch_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void catch_PID_clear(catch_PID_t *user_pid_clear)
{
    if (user_pid_clear == NULL)
    {
        return;
    }
    user_pid_clear->err = user_pid_clear->set = user_pid_clear->get = 0.0f;
    user_pid_clear->out = user_pid_clear->Pout = user_pid_clear->Iout = user_pid_clear->Dout = 0.0f;
}

static void gas_set_control(void)
{
	static uint8_t gas_forward_flag = 0;	//默认前伸气缸不伸出
	static uint8_t gas_catch_flag = 0;		//默认松开抓手
	static uint8_t gas_up_flag = 0;		//矿石升降flag，默认下降
	static uint16_t time = 0;			//状态改变 延时
	static uint8_t time_flag = 0;		//状态改变 延时flag
	
	
	// 状态改变延时
	if( time == 100 )	//	200ms时间到
	{
		time = 0;
		time_flag = 0;
	}
	if( time_flag == 1 )
	{
		time++;
		return ;	// 延时中，气动状态不可变，直接返回
	}
	
	//矿石升降
	if( (gas_rc->key.v & GAS_UP_KEY) && (gas_up_flag == 0) )
		
	{
		gas_up_flag = 1;
		time_flag = 1;
		expand_tx.gas_sta |= 0x01;
	}
	else if( (gas_rc->key.v & GAS_UP_KEY) && (gas_up_flag == 1) )
	{
		gas_up_flag = 0;
		time_flag = 1;
		expand_tx.gas_sta &= 0xfe;	//1111 1110
	}
	
	//抓手前伸
//	if( (gas_rc->key.v & KEY_PRESSED_OFFSET_R) && (gas_forward_flag == 0) )
	if( (gas_rc->mouse.press_l == 1) && (gas_forward_flag == 0) )
	{
		gas_forward_flag = 1;
		time_flag = 1;
		expand_tx.gas_sta |= 0x01<<1;
	}
//	else if( (gas_rc->key.v & KEY_PRESSED_OFFSET_R) && (gas_forward_flag == 1) )
	else if( (gas_rc->mouse.press_l == 1) && (gas_forward_flag == 1) )
	{
		gas_forward_flag = 0;
		time_flag = 1;
		expand_tx.gas_sta &= 0xfd;	//1111 1101
	}
	//抓手 抓取
//	if( (gas_rc->key.v & KEY_PRESSED_OFFSET_F) && (gas_catch_flag == 0) )
	if( (gas_rc->mouse.press_r == 1) && (gas_catch_flag == 0) )
	{
		gas_catch_flag = 1;
		time_flag = 1;
		expand_tx.gas_sta |= 0x01<<2;
	}
//	else if( (gas_rc->key.v & KEY_PRESSED_OFFSET_F) && (gas_catch_flag == 1) )
	else if( (gas_rc->mouse.press_r == 1) && (gas_catch_flag == 1) )
	{
		gas_catch_flag = 0;
		time_flag = 1;
		expand_tx.gas_sta &= 0xfb;	//1111 1011
	}

	//输出继电器信号，高电平控制
	if( gas_up_flag == 1)
		HAL_GPIO_WritePin(GAS_UP_GPIO_Port,GAS_UP_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_UP_GPIO_Port,GAS_UP_Pin,GPIO_PIN_RESET);
	
	if( gas_forward_flag == 1 )
		HAL_GPIO_WritePin(GAS_FORWARD_GPIO_Port,GAS_FORWARD_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_FORWARD_GPIO_Port,GAS_FORWARD_Pin,GPIO_PIN_RESET);
	
	if( gas_catch_flag == 1 )
		HAL_GPIO_WritePin(GAS_CATCH_GPIO_Port,GAS_CATCH_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_CATCH_GPIO_Port,GAS_CATCH_Pin,GPIO_PIN_RESET);
	
}

extern fp32 gimbal_yaw_angle;

static void radio_set_control(void)	//设置倒车雷达
{
	static uint8_t radio_forward_flag = 0;	//前摄像头，默认不开
	static uint8_t radio_backward_flag = 0;	//后摄像头，默认不开
	
	if( gas_rc->key.v & RADIO_KEY )	//复位图传
	{
		gimbal_yaw_angle = 77;
		
		radio_forward_flag = radio_backward_flag = 0;
		expand_tx.gas_sta &= 0x3f;	//0011 1111
		
		return ;
	}
	
	if( (gas_rc->key.v & RADIO_KEY) && (gas_rc->key.v & KEY_PRESSED_OFFSET_SHIFT) && (radio_forward_flag == 0) )
	{
		gimbal_yaw_angle = 180;
		
		radio_forward_flag = 1;
		radio_backward_flag = 0;
		expand_tx.gas_sta |= 0x01 << 7;
	}
	else if( (gas_rc->key.v & RADIO_KEY) && (gas_rc->key.v & KEY_PRESSED_OFFSET_SHIFT) && (radio_forward_flag == 1) )
	{
		gimbal_yaw_angle = 77;
		
		radio_forward_flag = 0;
		expand_tx.gas_sta &= 0xef;	//0111 1111
	}
	
	if( (gas_rc->key.v & RADIO_KEY) && (gas_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (radio_backward_flag == 0) )
	{
		gimbal_yaw_angle = 180;
		
		radio_backward_flag = 1;
		radio_forward_flag = 0;
		expand_tx.gas_sta |= 0x01 << 6; 
	}
	else if( (gas_rc->key.v & RADIO_KEY) && (gas_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (radio_backward_flag == 1) )
	{
		gimbal_yaw_angle = 77;
		
		radio_backward_flag = 0;
		expand_tx.gas_sta &= 0xbf;	//1011 1111
	}
	
}
