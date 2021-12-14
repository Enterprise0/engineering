#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

#define SERVO_YAW_CHANNEL 	2
#define SERVO_PITCH_CHANNEL 3

#define SERVO_RC_DEADBAND   10

#define SERVO_YAW_RC_SEN    	-0.005f
#define SERVO_PITCH_RC_SEN  	-0.006f //0.005

#define SERVO_YAW_MOUSE_SEN 	0.00005//0.00005f
#define SERVO_PITCH_MOUSE_SEN 	0.00015//0.00015f

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500
#define ANGLE_MIN		0.0f
#define ANGLE_MAX		180.0f			//270



extern void servo_task(void const * argument);

#endif
