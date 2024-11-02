#include <stdio.h>
#include "pico/stdlib.h"
#include "Mp3.h"
#include "max30102.h"
#include "debug.h"
#include "data_global.h"

// 全局变量
const uint8_t RATE_SIZE = 4; // 定义心率数组大小，用于计算心率的移动平均值
uint8_t rates[RATE_SIZE];    // 存储多个心率的数组，用于平滑心率波动
uint8_t rateSpot = 0;        // 当前存储心率的数组索引
absolute_time_t lastBeat;    // 更新 lastBeat 的类型

float beatsPerMinute;   // 实时心率（单位：每分钟）
int beatAvg = 0;        // 心率的平均值
bool heartBeat = false; // 心跳检测标志

extern int beatAvg;
extern bool heartBeat;

// 事件组句柄
extern EventGroupHandle_t xEventGroup;

// 数据队列句柄
extern QueueHandle_t xHeartbeatQueue;

// 任务句柄
extern TaskHandle_t xTask_TouchSwitchMonitor_Handle;
extern TaskHandle_t xTask_HeartbeatMonitor_Handle;
extern TaskHandle_t xTask_WS2812BControl_Handle;
extern TaskHandle_t xTask_VoiceBroadcastControl_Handle;
extern TaskHandle_t xTask_OLEDDisplay_Handle;
extern TaskHandle_t xTask_Blink_Handle;

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
            xEventGroupSetBits(xEventGroup, EVENT_TOUCHA_SWITCH);
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
        /*
        // 阻塞自身等待任务通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DEBUG_PRINT("HeartbeatMonitor task notified\n");
        // 执行心跳检测逻辑
        uint32_t irValue = particleSensor.getIR(); // 读取 IR 数据
        if (checkForBeat(irValue))
        {
            absolute_time_t currentTime = get_absolute_time();               // 获取当前时间
            int64_t delta_us = absolute_time_diff_us(lastBeat, currentTime); // 计算时间差
            lastBeat = currentTime;

            float delta = delta_us / 1000.0;        // 将微秒转换为毫秒
            beatsPerMinute = 60 / (delta / 1000.0); // 将时间差换算为每分钟心率

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (uint8_t)beatsPerMinute;
                rateSpot %= RATE_SIZE;

                // 计算平均心率
                beatAvg = 0;
                for (uint8_t x = 0; x < RATE_SIZE; x++)
                {
                    beatAvg += rates[x];
                }
                beatAvg /= RATE_SIZE;

                DEBUG_PRINT("Heartbeat detected! BPM: %d\n", beatAvg);
            }
        }*/

        long irValue = particleSensor.getIR(); // 获取红外 LED 的光强度值
        // DEBUG_PRINT("IR Value: %ld\n", irValue);

        if (checkForBeat(irValue) == true) // 检测是否检测到心跳
        {
            DEBUG_PRINT("Heartbeat detected!\n");
            // 检测到心跳！
            absolute_time_t currentTime = get_absolute_time();               // 获取当前时间
            int64_t delta_us = absolute_time_diff_us(lastBeat, currentTime); // 计算时间差，单位为微秒
            lastBeat = currentTime;                                          // 更新最后心跳时间

            float delta = delta_us / 1000.0;        // 将微秒转换为毫秒
            beatsPerMinute = 60 / (delta / 1000.0); // 将时间差换算为每分钟心率

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {                                                // 心率在合理范围内
                rates[rateSpot++] = (uint8_t)beatsPerMinute; // 将当前心率存储到数组中
                rateSpot %= RATE_SIZE;                       // 将数组索引限制在 RATE_SIZE 范围内

                // 计算心率的平均值
                beatAvg = 0;
                for (uint8_t x = 0; x < RATE_SIZE; x++)
                {
                    beatAvg += rates[x];
                }
                beatAvg /= RATE_SIZE;
            }
        }
        DEBUG_PRINT("Heart rate: %d\n", beatAvg);
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
            EVENT_TOUCHA_SWITCH | EVENT_TOUCHB_SWITCH | EVENT_HEARTBEAT_RISE | EVENT_HEARTBEAT_DOWN,
            pdTRUE,       // 清除已设置的事件位
            pdFALSE,      // 任意一个事件位被设置就返回
            portMAX_DELAY // 一直等待
        );

        if (uxBits & EVENT_TOUCHA_SWITCH)
        {
            // 处理触摸开关触发的氛围灯变化
        }

        if (uxBits & EVENT_TOUCHB_SWITCH)
        {
            // 处理心跳检测触发的氛围灯变化
        }

        if (uxBits & EVENT_HEARTBEAT_RISE)
        {
            // 处理心跳检测上升沿触发的氛围灯变化
        }

        if (uxBits & EVENT_HEARTBEAT_DOWN)
        {
            // 处理心跳检测下降沿触发的氛围灯变化
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 语音播报模块控制任务
void Task_VoiceBroadcastControl(void *pvParameters)
{

    while (1)
    {
        mp3.randomAll();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /*
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
        */
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

void gpio_interrupt_handler(uint gpio, uint32_t events)
{
    // 根据触发的GPIO处理不同的任务
    if (gpio == TOUCHA_PIN)
    {
        // 通知任务1
        xTaskNotify(xTask_TouchSwitchMonitor_Handle, 0, eSetBits);
        xEventGroupSetBits(xEventGroup, EVENT_TOUCHA_SWITCH);
    }
    else if (gpio == TOUCHB_PIN)
    {
        // 通知任务2
        xTaskNotify(xTask_TouchSwitchMonitor_Handle, 0, eSetBits);
        xEventGroupSetBits(xEventGroup, EVENT_TOUCHB_SWITCH);
    }
    // 处理第三个GPIO的上下沿变化
    else if (gpio == HEARTBEAT_INT_PIN)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            // xTaskNotify(xTask_HeartbeatMonitor_Handle, 0, eSetBits);
            // xEventGroupSetBits(xEventGroup, EVENT_HEARTBEAT_RISE);
            DEBUG_PRINT("Heartbeat rising edge detected\n");
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            // xEventGroupSetBits(xEventGroup, EVENT_HEARTBEAT_DOWN);
            DEBUG_PRINT("Heartbeat falling edge detected\n");
        }
    }
}

void hardware_init()
{
    /*mp3硬件初始化*/
    mp3.init();

    /*中断配置*/
    // 设置GPIO引脚为输入
    gpio_init(TOUCHA_PIN);
    gpio_set_dir(TOUCHB_PIN, GPIO_IN);
    gpio_pull_up(TOUCHB_PIN);

    gpio_init(TOUCHB_PIN);
    gpio_set_dir(TOUCHB_PIN, GPIO_IN);
    gpio_pull_up(TOUCHB_PIN);

    gpio_init(HEARTBEAT_INT_PIN);
    gpio_set_dir(HEARTBEAT_INT_PIN, GPIO_IN);

    // 设置中断回调
    gpio_set_irq_enabled_with_callback(TOUCHA_PIN, GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);
    gpio_set_irq_enabled_with_callback(TOUCHB_PIN, GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);
    gpio_set_irq_enabled_with_callback(HEARTBEAT_INT_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);

    /*心跳检测硬件初始化*/
    if (!particleSensor.begin(i2c0)) // 使用默认 I2C 端口，速率为 400kHz
    {
        DEBUG_PRINT("MAX30105 was not found. Please check wiring/power.");
        while (1)
            ; // 如果传感器初始化失败，程序停在此处
    }
    DEBUG_PRINT("Place your index finger on the sensor with steady pressure.");
    particleSensor.setup();                    // 配置传感器为默认设置
    particleSensor.enableDATARDY();            // 开启数据准备中断
    particleSensor.setPulseAmplitudeRed(0x0A); // 设置红光的幅度
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
}

int main(void)
{
    // 初始化标准库
    stdio_init_all();

    // 初始化硬件
    hardware_init();

    // 启动任务
    vLaunch();

    // 主循环不应该运行到这里
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
