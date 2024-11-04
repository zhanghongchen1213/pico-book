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
#define EVENT_TOUCHA_SWITCH_RISE (1 << 6)
#define EVENT_TOUCHB_SWITCH (1 << 1)
#define EVENT_TOUCHB_SWITCH_RISE (1 << 7)
#define EVENT_HEARTBEAT (1 << 2)
#define EVENT_HEARTCANCLE (1 << 3)
#define EVENT_ADC_MONITOR (1 << 4)
#define EVENT_EFFECT_CANCEL (1 << 5)

// 定义队列长度
#define QUEUE_LENGTH 10

// 定义 LED 闪烁间隔（毫秒）
#define LED_DELAY_MS 200

// 板载 LED 引脚
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define REG_LED_SUM 60

#define BRIGHTNESS 1500     // 开灯亮度阈值
#define CLOSEBTIGHTNESS 100 // 关灯亮度阈值
#endif                      // __DATA_GLOBAL_H__