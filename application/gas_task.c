#include "gas_task.h"

#include "cmsis_os.h"

#include "main.h"

#include "cmsis_os.h"

#include "remote_control.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gas_high_water;
#endif

static void gas_set_control(void);
	
const RC_ctrl_t *gas_control_rc;

void gas_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(357);
	
	gas_control_rc = get_remote_control_point();
	
    while (1)
    {
		
		gas_set_control();
		
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(GAS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gas_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }	
}

static void gas_set_control(void)
{
	static uint8_t gas_forward_flag = 0;	//Ĭ��ǰ�����ײ����
	static uint8_t gas_catch_flag = 0;		//Ĭ���ɿ�ץ��
	static uint8_t gas_up_flag = 0;		//��ʯ����flag��Ĭ���½�
	static uint16_t time = 0;			//״̬�ı� ��ʱ
	static uint8_t time_flag = 0;		//״̬�ı� ��ʱflag
	
	//����̵����źţ��ߵ�ƽ����
	if( gas_forward_flag == 1 )
		HAL_GPIO_WritePin(GAS_FORWARD_GPIO_Port,GAS_FORWARD_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_FORWARD_GPIO_Port,GAS_FORWARD_Pin,GPIO_PIN_RESET);
	if( gas_catch_flag == 1 )
		HAL_GPIO_WritePin(GAS_CATCH_GPIO_Port,GAS_CATCH_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_CATCH_GPIO_Port,GAS_CATCH_Pin,GPIO_PIN_RESET);
	if( gas_up_flag == 1)
		HAL_GPIO_WritePin(GAS_UP_GPIO_Port,GAS_UP_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GAS_UP_GPIO_Port,GAS_UP_Pin,GPIO_PIN_RESET);
	
	// ״̬�ı���ʱ
	if( time == 500 )	//	1sʱ�䵽
	{
		time = 0;
		time_flag = 0;
	}
	if( time_flag == 1 )
	{
		time++;
		return ;	// ��ʱ�У�����״̬���ɱ䣬ֱ�ӷ���
	}

//	//ץ��ǰ��
	if( gas_control_rc->rc.ch[0] >300 && (gas_forward_flag == 0) )
//	if( (gas_control_rc->mouse.press_r == 1) && (gas_forward_flag == 0) )
	{
		gas_forward_flag = 1;
		time_flag = 1;
	}
	else if( gas_control_rc->rc.ch[0] < -300 && (gas_forward_flag == 1) )
//	else if( (gas_control_rc->mouse.press_r == 0) && (gas_forward_flag == 1) )
	{
		gas_forward_flag = 0;
		time_flag = 1;
	}
//	//ץ�� ץȡ
	if( gas_control_rc->rc.ch[1] >300 && (gas_catch_flag == 0) )
//	if( (gas_control_rc->mouse.press_l) == 1 && (gas_catch_flag == 0) )
	{
		gas_catch_flag = 1;
		time_flag = 1;
	}
	else if( gas_control_rc->rc.ch[1] < -300 && (gas_catch_flag == 1) )
//	else if( (gas_control_rc->mouse.press_l) == 0 && (gas_catch_flag == 1) )
	{
		gas_catch_flag = 0;
		time_flag = 1;
	}
//	//��ʯ����
	if( gas_control_rc->rc.ch[2] >300 && (gas_up_flag == 0) )
//	if( (gas_control_rc->key.v & GAS_UP_KEY) && (gas_up_flag == 0) )
	{
		gas_up_flag = 1;
		time_flag = 1;
	}
	else if( gas_control_rc->rc.ch[2] < -300 && (gas_up_flag == 1) )
//	else if( (gas_control_rc->key.v & GAS_UP_KEY) && (gas_up_flag == 1) )
	{
		gas_up_flag = 0;
		time_flag = 1;
	}
}

