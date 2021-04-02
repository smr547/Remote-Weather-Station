

#ifndef SRC_POWER_TASK_H_
#define SRC_POWER_TASK_H_

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>

// Power management queue and task handle

static TaskHandle_t th_PowerTask;
static QueueHandle_t q_PowerTask;

#endif /* #ifndef SRC_POWER_TASK_H_ */
