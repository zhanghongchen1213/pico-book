#include "ws2812.h"

static int dma_channel;
static TaskHandle_t currentEffectTask = NULL; // 当前灯效任务句柄

extern EventGroupHandle_t xEventGroup;
extern uint32_t data[REG_LED_SUM];

WS2812 rgb;

// 初始化 PIO（可编程输入/输出）的函数
void WS2812::init_pio()
{
    // 将 WS2812 程序添加到 PIO 并获取偏移量
    uint offset = pio_add_program(pio0, &my_ws2812_program);

    // 初始化 WS2812 的 GPIO 引脚
    pio_gpio_init(pio0, WS2812_PIN);
    // 将引脚方向设置为输出
    pio_sm_set_consecutive_pindirs(pio0, 0, WS2812_PIN, 1, true);

    // 配置状态机
    auto c = my_ws2812_program_get_default_config(offset);
    // 设置状态机的引脚
    sm_config_set_set_pins(&c, WS2812_PIN, 1);
    // 配置移位寄存器
    sm_config_set_out_shift(&c, false, false, 32);
    // 使用配置初始化状态机
    pio_sm_init(pio0, 0, offset, &c);

    // 启用状态机
    pio_sm_set_enabled(pio0, 0, true);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq(&dma_config, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_channel,
        &dma_config,
        &pio0->txf[0], // 目标地址：PIO 的 TX FIFO
        data,          // 源地址：data 数组
        REG_LED_SUM,
        false // 不自动启动传输
    );
}

// 发送数据的函数，将更新 data 数组的数据发送到 DMA
void WS2812::send_data()
{
    dma_channel_set_read_addr(dma_channel, data, true);
    dma_channel_wait_for_finish_blocking(dma_channel); // 确保每次更新完成
}

void WS2812::set_all_leds_color(uint32_t color)
{
    for (uint i = 0; i < REG_LED_SUM; i++)
    {
        data[i] = color; // 设置所有灯的颜色
    }
    send_data(); // 更新灯带显示
}

void WS2812::set_single_led_color(uint index, uint32_t color)
{
    if (index < REG_LED_SUM)
    {
        data[index] = color; // 设置指定灯的颜色
        send_data();         // 更新灯带显示
    }
}

void WS2812::running_light(uint32_t color, uint delay_ms)
{
    for (uint i = 0; i < REG_LED_SUM; i++)
    {
        // 先清空所有灯
        for (uint j = 0; j < REG_LED_SUM; j++)
        {
            data[j] = 0x000000; // 关灯
        }
        data[i] = color;                     // 当前灯点亮指定颜色
        send_data();                         // 更新灯带显示
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 延时，控制流水速度
    }
}

void WS2812::breathing_light(uint32_t color, uint delay_ms)
{
    while (1)
    {

        for (int brightness = 0; brightness <= 255; brightness++)
        {
            uint32_t dimmed_color = ((color & 0xFF0000) * brightness / 255) & 0xFF0000 |
                                    ((color & 0x00FF00) * brightness / 255) & 0x00FF00 |
                                    ((color & 0x0000FF) * brightness / 255) & 0x0000FF;
            set_all_leds_color(dimmed_color);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        for (int brightness = 255; brightness >= 0; brightness--)
        {
            uint32_t dimmed_color = ((color & 0xFF0000) * brightness / 255) & 0xFF0000 |
                                    ((color & 0x00FF00) * brightness / 255) & 0x00FF00 |
                                    ((color & 0x0000FF) * brightness / 255) & 0x0000FF;
            set_all_leds_color(dimmed_color);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

void WS2812::rainbow_cycle(uint delay_ms)
{
    while (1)
    {

        for (int j = 0; j < 256; j++)
        { // 彩虹循环一次
            for (int i = 0; i < REG_LED_SUM; i++)
            {
                uint8_t wheel_pos = (i * 256 / REG_LED_SUM + j) & 255;
                if (wheel_pos < 85)
                {
                    data[i] = ((255 - wheel_pos * 3) << 16) | (wheel_pos * 3 << 8);
                }
                else if (wheel_pos < 170)
                {
                    wheel_pos -= 85;
                    data[i] = ((255 - wheel_pos * 3) << 8) | (wheel_pos * 3);
                }
                else
                {
                    wheel_pos -= 170;
                    data[i] = (wheel_pos * 3 << 16) | ((255 - wheel_pos * 3) << 8);
                }
            }
            send_data();
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

void WS2812::blink(uint32_t color, uint times, uint delay_ms)
{
    while (1)
    {
        for (uint i = 0; i < times; i++)
        {
            set_all_leds_color(color); // 设置颜色
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            set_all_leds_color(0x000000); // 关灯
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

void WS2812::heartbeat_effect(uint32_t color, uint peak_brightness, uint min_brightness, uint speed_up, uint speed_down)
{
    while (1)
    {
        // 快速跳跃：增加亮度到峰值
        for (uint brightness = min_brightness; brightness <= peak_brightness; brightness += 20) // 每次增加20步，快速增亮
        {
            uint8_t red = ((color >> 16) & 0xFF) * brightness / 255;
            uint8_t green = ((color >> 8) & 0xFF) * brightness / 255;
            uint8_t blue = (color & 0xFF) * brightness / 255;

            uint32_t dimmed_color = (red << 16) | (green << 8) | blue;

            for (uint i = 0; i < REG_LED_SUM; i++)
            {
                data[i] = dimmed_color;
            }

            send_data();
            vTaskDelay(pdMS_TO_TICKS(speed_up)); // 快速增亮
        }

        // 短暂停留在峰值亮度，模拟心脏的顶峰跳跃
        vTaskDelay(pdMS_TO_TICKS(speed_up * 2));

        // 缓慢减暗：从峰值亮度回到最低亮度
        for (uint brightness = peak_brightness; brightness >= min_brightness; brightness -= 10) // 每次减小3步，缓慢减暗
        {
            uint8_t red = ((color >> 16) & 0xFF) * brightness / 255;
            uint8_t green = ((color >> 8) & 0xFF) * brightness / 255;
            uint8_t blue = (color & 0xFF) * brightness / 255;

            uint32_t dimmed_color = (red << 16) | (green << 8) | blue;

            for (uint i = 0; i < REG_LED_SUM; i++)
            {
                data[i] = dimmed_color;
            }

            send_data();
            vTaskDelay(pdMS_TO_TICKS(speed_down)); // 缓慢减暗
        }

        // 短暂休息，模拟心脏跳动的间隔
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void WS2812::start_heartbeat_effect(uint32_t color)
{
    EventBits_t uxBits;
    while (1)
    {
        uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_HEARTCANCLE,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(0));

        if (uxBits & EVENT_HEARTCANCLE)
        {
            return;
        }

        heartbeat_effect(color, 200, 50, 5, 10);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// 心跳效果任务
void heartbeat_task(void *color)
{
    WS2812 *ws = &rgb;
    uint32_t heartbeat_color = *(uint32_t *)color;

    while (1)
    {
        ws->heartbeat_effect(heartbeat_color, 200, 50, 5, 15);

        // 检查是否收到取消信号
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup, EVENT_HEARTCANCLE, pdTRUE, pdFALSE, pdMS_TO_TICKS(0));
        if (uxBits & EVENT_HEARTCANCLE)
        {
            delete (uint32_t *)color; // 释放分配的内存
            vTaskDelete(NULL);        // 终止任务
        }
    }
}

// 呼吸灯效果任务
void breathing_light_task(void *color)
{
    WS2812 *ws = &rgb;
    uint32_t breath_color = *(uint32_t *)color;

    while (1)
    {
        ws->breathing_light(breath_color, 5);

        // 检查是否收到取消信号
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup, EVENT_EFFECT_CANCEL, pdTRUE, pdFALSE, pdMS_TO_TICKS(0));
        if (uxBits & EVENT_EFFECT_CANCEL)
        {
            delete (uint32_t *)color; // 释放分配的内存
            vTaskDelete(NULL);        // 终止任务
        }
    }
}

void rainbow_cycle_task(void *color)
{
    WS2812 *ws = &rgb;
    uint32_t rainbow_color = *(uint32_t *)color;

    while (1)
    {
        ws->rainbow_cycle(20);

        // 检查是否收到取消信号
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup, EVENT_EFFECT_CANCEL, pdTRUE, pdFALSE, pdMS_TO_TICKS(0));
        if (uxBits & EVENT_EFFECT_CANCEL)
        {
            delete (uint32_t *)color; // 释放分配的内存
            vTaskDelete(NULL);        // 终止任务
        }
    }
}

void blink_task(void *color)
{
    WS2812 *ws = &rgb;
    uint32_t blink_color = *(uint32_t *)color;

    while (1)
    {
        ws->blink(blink_color, 3, 200);

        // 检查是否收到取消信号
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup, EVENT_EFFECT_CANCEL, pdTRUE, pdFALSE, pdMS_TO_TICKS(0));
        if (uxBits & EVENT_EFFECT_CANCEL)
        {
            delete (uint32_t *)color; // 释放分配的内存
            vTaskDelete(NULL);        // 终止任务
        }
    }
}

// 启动指定灯效
void WS2812::start_effect(uint32_t color, void (*effect_func)(void *))
{
    // 如果已有任务在运行，发送取消事件并删除任务
    if (currentEffectTask != NULL)
    {
        xEventGroupSetBits(xEventGroup, EVENT_EFFECT_CANCEL);
        vTaskDelete(currentEffectTask);
        currentEffectTask = NULL; // 确保任务句柄为空
    }

    // 为颜色参数创建局部副本，并启动新的灯效任务
    uint32_t *color_ptr = new uint32_t(color);
    xTaskCreate(effect_func, "EffectTask", 1024, (void *)color_ptr, tskIDLE_PRIORITY + 1, &currentEffectTask);
}

uint32_t convertToGRB(uint32_t color)
{
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;
    return (green << 16) | (red << 8) | blue;
}