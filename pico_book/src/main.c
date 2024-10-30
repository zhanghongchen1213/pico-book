#include <stdio.h>
#include "pico/stdlib.h"
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
#define EVENT_TOUCH_SWITCH (1 << 0)
#define EVENT_HEARTBEAT (1 << 1)

// 定义队列长度
#define QUEUE_LENGTH 10

// 定义 LED 闪烁间隔（毫秒）
#define LED_DELAY_MS 500

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

// 板载 LED 引脚
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

// 触摸开关监测任务
void Task_TouchSwitchMonitor(void *pvParameters)
{
    while (1)
    {
        // 检测触摸开关状态
        bool touchDetected = false; // 这里需要替换为实际的触摸检测逻辑

        if (touchDetected)
        {
            // 控制5个IO口连接的LED灯
            // 控制单IO口连接的LED灯带
            // 上报MCU（如需要）

            // 设置事件，通知其他任务
            xEventGroupSetBits(xEventGroup, EVENT_TOUCH_SWITCH);
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 心跳检测任务
void Task_HeartbeatMonitor(void *pvParameters)
{
    while (1)
    {
        // 检测心跳传感器
        bool heartbeatDetected = false; // 这里需要替换为实际的心跳检测逻辑

        if (heartbeatDetected)
        {
            // 获取心跳数据
            int heartbeatData = 0; // 这里需要替换为实际的心跳数据

            // 发送心跳数据到队列
            xQueueSend(xHeartbeatQueue, &heartbeatData, 0);

            // 设置事件，通知其他任务
            xEventGroupSetBits(xEventGroup, EVENT_HEARTBEAT);
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// WS2812B 氛围灯控制任务
void Task_WS2812BControl(void *pvParameters)
{
    EventBits_t uxBits;

    while (1)
    {
        // 等待事件组的事件
        uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_TOUCH_SWITCH | EVENT_HEARTBEAT,
            pdTRUE,       // 清除已设置的事件位
            pdFALSE,      // 任意一个事件位被设置就返回
            portMAX_DELAY // 一直等待
        );

        if (uxBits & EVENT_TOUCH_SWITCH)
        {
            // 处理触摸开关触发的氛围灯变化
        }

        if (uxBits & EVENT_HEARTBEAT)
        {
            // 处理心跳检测触发的氛围灯变化
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 语音播报模块控制任务
void Task_VoiceBroadcastControl(void *pvParameters)
{
    EventBits_t uxBits;

    // 开机后播放默认音乐
    // 播放默认音乐的逻辑

    while (1)
    {
        // 等待事件组的事件
        uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_TOUCH_SWITCH | EVENT_HEARTBEAT,
            pdTRUE,       // 清除已设置的事件位
            pdFALSE,      // 任意一个事件位被设置就返回
            portMAX_DELAY // 一直等待
        );

        if (uxBits & EVENT_TOUCH_SWITCH)
        {
            // 播放触摸开关触发的音频
        }

        if (uxBits & EVENT_HEARTBEAT)
        {
            // 播放心跳检测触发的音频
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// OLED 显示任务
void Task_OLEDDisplay(void *pvParameters)
{
    int heartbeatData;

    // 开机后显示默认图像
    // 显示默认图像的逻辑

    while (1)
    {
        // 从队列中接收心跳数据
        if (xQueueReceive(xHeartbeatQueue, &heartbeatData, portMAX_DELAY) == pdPASS)
        {
            // 更新OLED显示内容，显示心跳数据
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// LED 闪烁任务
void Task_Blink(void *pvParameters)
{
    // 初始化板载 LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool led_state = false;

    while (1)
    {
        // 切换 LED 状态
        led_state = !led_state;
        gpio_put(PICO_DEFAULT_LED_PIN, led_state);

        // 延时
        vTaskDelay(pdMS_TO_TICKS(LED_DELAY_MS));
    }
}

void vLaunch(void)
{
    // 创建事件组
    xEventGroup = xEventGroupCreate();

    // 创建数据队列
    xHeartbeatQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));

    // 创建触摸开关监测任务
    xTaskCreate(Task_TouchSwitchMonitor, "TouchSwitchMonitor", STACK_SIZE, NULL, PRIORITY_HIGH, &xTask_TouchSwitchMonitor_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_TouchSwitchMonitor_Handle, (1 << 0)); // 绑定到核心0
#endif

    // 创建心跳检测任务
    xTaskCreate(Task_HeartbeatMonitor, "HeartbeatMonitor", STACK_SIZE, NULL, PRIORITY_HIGH, &xTask_HeartbeatMonitor_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_HeartbeatMonitor_Handle, (1 << 0)); // 绑定到核心0
#endif

    // 创建 WS2812B 氛围灯控制任务
    xTaskCreate(Task_WS2812BControl, "WS2812BControl", STACK_SIZE, NULL, PRIORITY_MEDIUM, &xTask_WS2812BControl_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_WS2812BControl_Handle, (1 << 1)); // 绑定到核心1
#endif

    // 创建语音播报模块控制任务
    xTaskCreate(Task_VoiceBroadcastControl, "VoiceBroadcastControl", STACK_SIZE, NULL, PRIORITY_MEDIUM, &xTask_VoiceBroadcastControl_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_VoiceBroadcastControl_Handle, (1 << 1)); // 绑定到核心1
#endif

    // 创建 OLED 显示任务
    xTaskCreate(Task_OLEDDisplay, "OLEDDisplay", STACK_SIZE, NULL, PRIORITY_MEDIUM, &xTask_OLEDDisplay_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_OLEDDisplay_Handle, (1 << 1)); // 绑定到核心1
#endif

    // 创建 LED 闪烁任务
    xTaskCreate(Task_Blink, "BlinkTask", STACK_SIZE, NULL, PRIORITY_LOW, &xTask_Blink_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    // 将 LED 闪烁任务绑定到核心1，以免影响高优先级任务
    vTaskCoreAffinitySet(xTask_Blink_Handle, (1 << 1));
#endif

    // 启动调度器
    vTaskStartScheduler();
}

int main(void)
{
    // 初始化标准库
    stdio_init_all();

    // 启动任务
    vLaunch();

    // 主循环不应该运行到这里
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
