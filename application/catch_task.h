#ifndef CATCH_TASK_H
#define CATCH_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "AHRS_middleware.h"

#include "user_lib.h"

//ң��������ģʽ
#define CATCH_MODE_CHANNEL  1
//ң����ҡ��
#define CATCH_UP_CHANNEL	1
#define CATCH_PITCH_CHANNEL 3
#define ROTATION_CHANNEL	4

//������ץ�� ����
#define UP_DOWN_KEY			KEY_PRESSED_OFFSET_C
#define CATCH_PITCH_KEY		KEY_PRESSED_OFFSET_Q
#define CATCH_ROTATION_KEY	KEY_PRESSED_OFFSET_E	

//���װ���
#define GAS_UP_KEY		KEY_PRESSED_OFFSET_V
//�����״�
#define RADIO_KEY		KEY_PRESSED_OFFSET_F

#define CATCH_TASK_INIT_TIME	357
//catch task control time  2ms
//ץ��������Ƽ�� 2ms
#define CATCH_CONTROL_TIME_MS	2
#define CATCH_CONTROL_TIME		CATCH_CONTROL_TIME_MS/1000.0f

//ң�������ж�
#define CATCH_UP_RC_SEN				0.00000005f
#define CATCH_PITCH_RC_SEN			0.000005f
#define ROTATION_RC_SEN				0.000005f

//�������ж�
#define CATCH_UP_KEY_SEN			0.000005f
#define CATCH_PITCH_KEY_SEN			0.000005f
#define ROTATION_KEY_SEN			0.000005f

//3508 �ٶȻ� PID�����Լ� PID���������������
#define CATCH_SPEED_PID_KP        15000.0f
#define CATCH_SPEED_PID_KI        10.0f
#define CATCH_SPEED_PID_KD        0.0f
#define CATCH_SPEED_PID_MAX_OUT   16000.0f
#define CATCH_SPEED_PID_MAX_IOUT  2000.0f

//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define CATCH_ENCODE_RELATIVE_PID_KP 2.5f
#define CATCH_ENCODE_RELATIVE_PID_KI 0.0f
#define CATCH_ENCODE_RELATIVE_PID_KD 0.0f
#define CATCH_ENCODE_RELATIVE_PID_MAX_OUT 2.0f
#define CATCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//��ת �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define ROTATION_ENCODE_RELATIVE_PID_KP 1.5f
#define ROTATION_ENCODE_RELATIVE_PID_KI 0.0f
#define ROTATION_ENCODE_RELATIVE_PID_KD 0.0f
#define ROTATION_ENCODE_RELATIVE_PID_MAX_OUT 2.0f	//3.0f
#define ROTATION_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//���� �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define UP_ENCODE_RELATIVE_PID_KP 1.5f
#define UP_ENCODE_RELATIVE_PID_KI 0.0f
#define UP_ENCODE_RELATIVE_PID_KD 0.0f
#define UP_ENCODE_RELATIVE_PID_MAX_OUT 2.0f	//3.0f
#define UP_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f


//���λ�øı����Ҫ�޸�
#define CATCH_L_MAX_ANGLE	0.0f *ANGLE_TO_RAD
#define CATCH_L_MIN_ANGLE	-75.0f *ANGLE_TO_RAD
#define CATCH_R_MAX_ANGLE	75.0f *ANGLE_TO_RAD
#define CATCH_R_MIN_ANGLE	0.0f *ANGLE_TO_RAD

#define ROTATION_MAX_ANGLE		180.0f *ANGLE_TO_RAD
#define ROTATION_MIN_ANGLE		-180.0f *ANGLE_TO_RAD

#define UP_L_MAX_ANGLE		0.0f *ANGLE_TO_RAD
#define UP_L_MIN_ANGLE		-400.0f *ANGLE_TO_RAD
#define UP_R_MAX_ANGLE		400.0f *ANGLE_TO_RAD
#define UP_R_MIN_ANGLE		0.0f *ANGLE_TO_RAD

#define CATCH_UP_L_ADD_VALUE		0.2f *ANGLE_TO_RAD
#define CATCH_UP_R_ADD_VALUE		0.2f *ANGLE_TO_RAD
#define CATCH_PITCH_L_ADD_VALUE		0.05f *ANGLE_TO_RAD
#define CATCH_PITCH_R_ADD_VALUE		0.05f *ANGLE_TO_RAD
#define CATCH_ROTATION_ADD_VALUE	0.05f *ANGLE_TO_RAD

//typedef enum
//{
//	CATCH_MOTOR_RAW = 0,	
//} catch_motor_mode_e;

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
} catch_PID_t;

typedef struct
{
    const motor_measure_t *catch_motor_measure;
    catch_PID_t catch_motor_relative_angle_pid;
    pid_type_def  catch_motor_speed_pid;
//    barrier_motor_mode_e barrier_motor_mode;
//    barrier_motor_mode_e last_barrier_motor_mode;
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
} catch_motor_t;

typedef struct
{
	const RC_ctrl_t *catch_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
    catch_motor_t catch_pitch_l_motor;
    catch_motor_t catch_pitch_r_motor;
    catch_motor_t catch_rotation_motor;
    catch_motor_t catch_up_l_motor;
    catch_motor_t catch_up_r_motor;	

    ramp_function_source_t catch_pitch_l_ramp;
    ramp_function_source_t catch_pitch_r_ramp;
    ramp_function_source_t catch_rotation_ramp;
    ramp_function_source_t catch_up_l_ramp;
    ramp_function_source_t catch_up_r_ramp;	
	
} catch_control_t;

void catch_task(void const *pvParameters);

#endif /* CATCH_TASK_H */
