#ifndef USER_TASK_H
#define USER_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "AHRS_middleware.h"

#include "user_lib.h"

//遥控器开关
#define USER_MODE_CHANNEL		1
//遥控器摇杆
#define BARRIER_PITCH_CHANNEL 	1
#define SAVER_CHANNEL			3

//障碍块、救援 按键
#define BARRIER_KEY		KEY_PRESSED_OFFSET_Z
#define SAVER_KEY		KEY_PRESSED_OFFSET_X

#define USER_TASK_INIT_TIME	357
//user task control time  2ms
//障碍块任务控制间隔 2ms
#define USER_CONTROL_TIME_MS	2.5f
#define USER_CONTROL_TIME 		USER_CONTROL_TIME_MS/1000.0f

#define BARRIER_PITCH_RC_SEN	0.000005f
#define SAVER_RC_SEN			0.000005f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define USER_SPEED_PID_KP        15000.0f
#define USER_SPEED_PID_KI        10.0f
#define USER_SPEED_PID_KD        0.0f
#define USER_SPEED_PID_MAX_OUT   16000.0f
#define USER_SPEED_PID_MAX_IOUT  2000.0f

//障碍块pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define BARRIER_ENCODE_RELATIVE_PID_KP 5.0f
#define BARRIER_ENCODE_RELATIVE_PID_KI 0.0f
#define BARRIER_ENCODE_RELATIVE_PID_KD 0.0f
#define BARRIER_ENCODE_RELATIVE_PID_MAX_OUT 2.0f
#define BARRIER_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//救援 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define SAVER_ENCODE_RELATIVE_PID_KP 4.0f
#define SAVER_ENCODE_RELATIVE_PID_KI 0.0f
#define SAVER_ENCODE_RELATIVE_PID_KD 0.0f
#define SAVER_ENCODE_RELATIVE_PID_MAX_OUT 2.0f	//3.0f
#define SAVER_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f


//电机位置改变后需要修改
#define BARRIER_L_MAX_ANGLE		70.0f *ANGLE_TO_RAD
#define BARRIER_L_MIN_ANGLE		10.0f *ANGLE_TO_RAD
#define BARRIER_R_MAX_ANGLE		-10.0f *ANGLE_TO_RAD
#define BARRIER_R_MIN_ANGLE		-70.0f *ANGLE_TO_RAD

#define SAVER_L_MAX_ANGLE		-10.0f *ANGLE_TO_RAD
#define SAVER_L_MIN_ANGLE		-80.0f *ANGLE_TO_RAD
#define SAVER_R_MAX_ANGLE		80.0f *ANGLE_TO_RAD
#define SAVER_R_MIN_ANGLE		10.0f *ANGLE_TO_RAD

//
#define BARRIER_L_ADD_VALUE		0.1f *ANGLE_TO_RAD
#define BARRIER_R_ADD_VALUE		0.1f *ANGLE_TO_RAD
#define SAVER_L_ADD_VALUE		0.1f *ANGLE_TO_RAD
#define SAVER_R_ADD_VALUE		0.1f *ANGLE_TO_RAD
//typedef enum
//{
//	USER_MOTOR_RAW = 0,	
//} user_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} user_PID_t;

typedef struct
{
    const motor_measure_t *user_motor_measure;
    user_PID_t user_motor_relative_angle_pid;
    pid_type_def  user_motor_speed_pid;
//    user_motor_mode_e user_motor_mode;
//    user_motor_mode_e last_user_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
//    fp32 motor_gyro;         //rad/s
//    fp32 motor_gyro_set;
    fp32 motor_speed;
	fp32 motor_speed_set;
//    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	
	int32_t ecd_count;
} user_motor_t;

typedef struct
{
	const RC_ctrl_t *user_RC;               //底盘使用的遥控器指针, the point to remote control
    
	user_motor_t barrier_l_motor;
    user_motor_t barrier_r_motor;
    user_motor_t saver_l_motor;
    user_motor_t saver_r_motor;
	
	ramp_function_source_t barrier_l_ramp;	//障碍块 角度控制 斜坡
	ramp_function_source_t barrier_r_ramp;
	ramp_function_source_t saver_l_ramp;
	ramp_function_source_t saver_r_ramp;
} user_control_t;



void user_task(void const *pvParameters);

#endif /* user_TASK_H */
