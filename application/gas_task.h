#ifndef GAS_TASK_H
#define GAS_TASK_H

#define GAS_TASK_INIT_TIME	357
//catch task control time  2ms
//气缸任务控制间隔 2ms
#define GAS_CONTROL_TIME_MS		2
#define GAS_CONTROL_TIME		GAS_CONTROL_TIME_MS/1000.0f


//气缸按键
#define GAS_UP_KEY		KEY_PRESSED_OFFSET_V


void gas_task(void const *pvParameters);

#endif /* GAS_TASK_H */
