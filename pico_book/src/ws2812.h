#ifndef __WS2812_H
#define __WS2812_H

#include "pico/stdlib.h"

#include "hardware/dma.h"

#include "ws2812.pio.h"

#include "debug.h"

#include "data_global.h"

#define WS2812_PIN 22

class WS2812
{
private:
    // 发送数据
    void send_data();

public:
    // 初始化 PIO
    void init_pio();

    // 设置所有 LED 的颜色
    void set_all_leds_color(uint32_t color);

    // 设置单个 LED 的颜色
    void set_single_led_color(uint index, uint32_t color);

    // 跑马灯效果
    void running_light(uint32_t color, uint delay_ms);

    // 呼吸灯效果
    void breathing_light(uint32_t color, uint delay_ms);

    // 彩虹循环效果
    void rainbow_cycle(uint delay_ms);

    // 闪烁效果
    void blink(uint32_t color, uint times, uint delay_ms);

    // 心跳效果
    void heartbeat_effect(uint32_t color, uint peak_brightness, uint min_brightness, uint speed_up, uint speed_down);

    void start_heartbeat_effect(uint32_t color);

    void start_effect(uint32_t color, void (*effect_func)(void *));
};

void heartbeat_task(void *color);
void breathing_light_task(void *color);
void rainbow_cycle_task(void *color);
void blink_task(void *color);
uint32_t convertToGRB(uint32_t color);

extern WS2812 rgb;
#endif
