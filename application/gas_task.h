#ifndef GAS_TASK_H
#define GAS_TASK_H

#define GAS_TASK_INIT_TIME	357
//catch task control time  2ms
//����������Ƽ�� 2ms
#define GAS_CONTROL_TIME_MS		2
#define GAS_CONTROL_TIME		GAS_CONTROL_TIME_MS/1000.0f


//���װ���
#define GAS_UP_KEY		KEY_PRESSED_OFFSET_V


void gas_task(void const *pvParameters);

#endif /* GAS_TASK_H */
