#include <stdio.h>
#include "pico/stdlib.h"
#include "Mp3.h"
#include "max30102.h"
#include "debug.h"
#include "data_global.h"
#include "ssd1306_i2c.h"
#include "irq.h"
#include "ws2812.h"
#include "led.h"
#include "lightR.h"

// 全局变量
const uint8_t RATE_SIZE = 4; // 定义心率数组大小，用于计算心率的移动平均值
uint8_t rates[RATE_SIZE];    // 存储多个心率的数组，用于平滑心率波动
uint8_t rateSpot = 0;        // 当前存储心率的数组索引
absolute_time_t lastBeat;    // 更新 lastBeat 的类型

float beatsPerMinute;   // 实时心率（单位：每分钟）
int beatAvg = 0;        // 心率的平均值
bool heartBeat = false; // 心跳检测标志

/*外部变量*/
extern uint32_t data[REG_LED_SUM];
// 事件组句柄
extern EventGroupHandle_t xEventGroup;
// 数据队列句柄
extern QueueHandle_t xHeartbeatQueue;
// 任务句柄
extern TaskHandle_t xTask_ADCMonitor_Handle;
extern TaskHandle_t xTask_HeartbeatMonitor_Handle;
extern TaskHandle_t xTask_WS2812BControl_Handle;
extern TaskHandle_t xTask_VoiceBroadcastControl_Handle;
extern TaskHandle_t xTask_OLEDDisplay_Handle;
extern TaskHandle_t xTask_Blink_Handle;

// 光敏监测任务
void Task_ADCMonitor(void *pvParameters)
{
    bool eventReported = false; // 标志位，用于标记事件是否已上报
    while (1)
    {
        uint16_t lightValue = adc.read();
        DEBUG_PRINT("Light value: %d\n", lightValue);

        if (lightValue >= BRIGHTNESS && !eventReported) // 达到亮度阈值且事件尚未触发
        {
            // 打开最后一页，开启流水灯
            led_group.led_on();
            // 上报MCU更新RGB和语音事件
            xEventGroupSetBits(xEventGroup, EVENT_ADC_MONITOR);
            eventReported = true; // 标记事件已上报
        }
        else if (lightValue <= CLOSEBTIGHTNESS && eventReported) // 达到关闭阈值且事件已触发
        {
            // 关闭流水灯
            led_group.led_off();
            eventReported = false; // 重置标志位，允许重新触发事件
        }

        // 适当延时，避免任务占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 心跳检测任务
void Task_HeartbeatMonitor(void *pvParameters)
{
    bool heart_detected = false;     // 标记心跳是否正在进行
    bool finger_present = false;     // 标记手指是否在传感器上
    bool heartbeat_reported = false; // 标记心跳事件是否已上报

    while (1)
    {
        long irValue = particleSensor.getIR(); // 获取红外 LED 的光强度值
        // DEBUG_PRINT("IR value: %ld\n", irValue);

        if (irValue > 100000) // 检测手指是否放上
        {
            if (!finger_present)
            {
                // 首次检测到手指放上
                finger_present = true;
                DEBUG_PRINT("Finger detected on sensor.\n");
                heartbeat_reported = false; // 允许触发心跳事件
            }

            // 始终检测心跳，不受 heart_detected 限制
            if (checkForBeat(irValue))
            {
                if (!heart_detected)
                {
                    heart_detected = true;

                    // 仅触发一次心跳事件
                    if (!heartbeat_reported)
                    {
                        DEBUG_PRINT("Heartbeat detected!\n");
                        xEventGroupSetBits(xEventGroup, EVENT_HEARTBEAT); // 上报心跳事件
                        heartbeat_reported = true;                        // 标记为已触发
                    }
                }

                // 计算心率
                absolute_time_t currentTime = get_absolute_time();               // 获取当前时间
                int64_t delta_us = absolute_time_diff_us(lastBeat, currentTime); // 计算时间差
                lastBeat = currentTime;                                          // 更新最后心跳时间

                float delta = delta_us / 1000.0;        // 将微秒转换为毫秒
                beatsPerMinute = 60 / (delta / 1000.0); // 换算为每分钟心率

                if (beatsPerMinute < 255 && beatsPerMinute > 20) // 心率在合理范围内
                {
                    rates[rateSpot++] = (uint8_t)beatsPerMinute;
                    rateSpot %= RATE_SIZE;

                    // 计算心率平均值
                    beatAvg = 0;
                    for (uint8_t x = 0; x < RATE_SIZE; x++)
                    {
                        beatAvg += rates[x];
                    }
                    beatAvg /= RATE_SIZE;
                }
            }
        }
        else if (finger_present && heart_detected) // 手指从传感器上移开
        {
            // 手指拿开，停止心跳监测
            finger_present = false;
            heart_detected = false;
            beatAvg = 0;
            DEBUG_PRINT("Finger removed, heart monitoring stopped.\n");
            xEventGroupSetBits(xEventGroup, EVENT_HEARTCANCLE); // 上报心跳取消事件
        }
        DEBUG_PRINT("Heart rate: %d\n", beatAvg);
        // 将心跳数据发送到队列中
        if (beatAvg > 0)
        {
            xQueueSend(xHeartbeatQueue, &beatAvg, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// WS2812B 氛围灯控制任务
void Task_WS2812BControl(void *pvParameters)
{
    EventBits_t uxBits;
    rgb.start_effect(convertToGRB(0xFF0000), rainbow_cycle_task); // 彩虹效果

    while (1)
    {
        // 等待事件组的事件
        uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_TOUCHA_SWITCH | EVENT_TOUCHB_SWITCH | EVENT_HEARTBEAT | EVENT_ADC_MONITOR | EVENT_HEARTCANCLE |
                EVENT_TOUCHA_SWITCH_RISE | EVENT_TOUCHB_SWITCH_RISE,
            pdTRUE,       // 清除已设置的事件位
            pdFALSE,      // 任意一个事件位被设置就返回
            portMAX_DELAY // 一直等待
        );

        if (uxBits & EVENT_TOUCHA_SWITCH)
        {
            // 处理触摸开关触发的氛围灯变化
            rgb.start_effect(convertToGRB(0x6495ED), blink_task); // 绿色闪烁效果
        }

        if (uxBits & EVENT_TOUCHB_SWITCH)
        {
            // 处理触摸开关触发的氛围灯变化
            rgb.start_effect(convertToGRB(0xFF0000), rainbow_cycle_task); // 彩虹效果
        }

        if (uxBits & EVENT_HEARTBEAT)
        {
            rgb.start_effect(convertToGRB(0xFF0000), heartbeat_task); // 红色心跳效果
        }

        if (uxBits & EVENT_HEARTCANCLE)
        {
            // 切换到彩虹循环效果
            rgb.start_effect(convertToGRB(0xFF69B4), breathing_light_task); // 粉色呼吸灯效果
        }

        if (uxBits & EVENT_ADC_MONITOR)
        {
            // 切换到彩虹循环效果
            rgb.start_effect(convertToGRB(0xFFA500), breathing_light_task); // 橙色呼吸灯效果
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
            EVENT_TOUCHA_SWITCH | EVENT_TOUCHB_SWITCH | EVENT_HEARTBEAT | EVENT_ADC_MONITOR | EVENT_HEARTCANCLE |
                EVENT_TOUCHA_SWITCH_RISE | EVENT_TOUCHB_SWITCH_RISE,
            pdTRUE,       // 清除已设置的事件位
            pdFALSE,      // 任意一个事件位被设置就返回
            portMAX_DELAY // 一直等待
        );

        if (uxBits & EVENT_TOUCHA_SWITCH)
        {
            // 播放触摸开关触发的音频
        }

        if (uxBits & EVENT_TOUCHB_SWITCH)
        {
            // 播放触摸开关触发的音频
        }

        if (uxBits & EVENT_HEARTBEAT)
        {
            // 播放心跳检测触发的音频
        }

        if (uxBits & EVENT_ADC_MONITOR)
        {
            // 播放 ADC 监测触发的音频
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
    xTaskCreate(Task_ADCMonitor, "ADCMonitor", STACK_SIZE, NULL, PRIORITY_MEDIUM, &xTask_ADCMonitor_Handle);
#if (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(xTask_ADCMonitor_Handle, (1 << 0)); // 绑定到核心0
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

void hardware_init()
{
    /*mp3硬件初始化*/
    mp3.init();

    /*中断配置*/
    pico_irq_init();

    /*心跳检测硬件初始化*/
    if (!particleSensor.begin()) // 使用默认 I2C 端口，速率为 400kHz
    {
        DEBUG_PRINT("MAX30105 was not found. Please check wiring/power.");
        while (1)
            ; // 如果传感器初始化失败，程序停在此处
    }
    particleSensor.setup();                    // 配置传感器为默认设置
    particleSensor.enableDATARDY();            // 开启数据准备中断
    particleSensor.setPulseAmplitudeRed(0x0A); // 设置红光的幅度
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

    // DEBUG_PRINT("SSD1306 OLED init begin....\r\n");
    // ssd1306_begin();

    rgb.init_pio();

    // rgb.rainbow_cycle(20);

    led_group.led_init();

    adc.init();
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
