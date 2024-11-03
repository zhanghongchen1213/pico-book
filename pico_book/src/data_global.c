#include "data_global.h"

// 事件组句柄
EventGroupHandle_t xEventGroup;

// 数据队列句柄
QueueHandle_t xHeartbeatQueue;

// 任务句柄
TaskHandle_t xTask_TouchSwitchMonitor_Handle;
TaskHandle_t xTask_HeartbeatMonitor_Handle;
TaskHandle_t xTask_WS2812BControl_Handle;
TaskHandle_t xTask_VoiceBroadcastControl_Handle;
TaskHandle_t xTask_OLEDDisplay_Handle;
TaskHandle_t xTask_Blink_Handle;

uint32_t data[REG_LED_SUM]; // WS2812B 数据缓冲区