#ifndef __DATA_GLOBAL_H__
#define __DATA_GLOBAL_H__

#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"

// 定义任务优先级
#define PRIORITY_HIGH (configMAX_PRIORITIES - 1)
#define PRIORITY_MEDIUM (configMAX_PRIORITIES - 2)
#define PRIORITY_LOW (configMAX_PRIORITIES - 3)

// 定义任务栈大小
#define STACK_SIZE 256

// 定义事件组事件位
#define EVENT_TOUCHA_SWITCH (1 << 0)
#define EVENT_TOUCHB_SWITCH (1 << 1)
#define EVENT_HEARTBEAT_RISE (1 << 2)
#define EVENT_HEARTBEAT_DOWN (1 << 3)

// 定义队列长度
#define QUEUE_LENGTH 10

// 定义 LED 闪烁间隔（毫秒）
#define LED_DELAY_MS 500

// 板载 LED 引脚
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define TOUCHA_PIN 16
#define TOUCHB_PIN 17
#define HEARTBEAT_INT_PIN 6

#endif // __DATA_GLOBAL_H__