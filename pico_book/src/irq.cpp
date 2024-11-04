#include "irq.h"

// 事件组句柄
extern EventGroupHandle_t xEventGroup;

// 任务句柄
extern TaskHandle_t xTask_ADCMonitor_Handle;
extern TaskHandle_t xTask_HeartbeatMonitor_Handle;
extern TaskHandle_t xTask_WS2812BControl_Handle;
extern TaskHandle_t xTask_VoiceBroadcastControl_Handle;

void pico_irq_init(void)
{
    gpio_init(TOUCHA_PIN);
    gpio_set_dir(TOUCHA_PIN, GPIO_IN);
    gpio_pull_up(TOUCHA_PIN);

    gpio_init(TOUCHB_PIN);
    gpio_set_dir(TOUCHB_PIN, GPIO_IN);
    gpio_pull_up(TOUCHB_PIN);

    gpio_init(HEARTBEAT_INT_PIN);
    gpio_set_dir(HEARTBEAT_INT_PIN, GPIO_IN);

    // 设置中断回调
    gpio_set_irq_enabled_with_callback(TOUCHA_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);
    gpio_set_irq_enabled_with_callback(TOUCHB_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);
    gpio_set_irq_enabled_with_callback(HEARTBEAT_INT_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_interrupt_handler);
}

void gpio_interrupt_handler(uint gpio, uint32_t events)
{
    // 根据触发的GPIO处理不同的任务
    if (gpio == TOUCHA_PIN)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            led_group.led_alone_off();
            xEventGroupSetBits(xEventGroup, EVENT_TOUCHA_SWITCH);
            // DEBUG_PRINT("TouchA rising edge detected\n");
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            led_group.led_alone_on();
            xEventGroupSetBits(xEventGroup, EVENT_TOUCHA_SWITCH_RISE);
            // DEBUG_PRINT("TouchA falling edge detected\n");
        }
    }
    else if (gpio == TOUCHB_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            xEventGroupSetBits(xEventGroup, EVENT_TOUCHB_SWITCH);
            // DEBUG_PRINT("TouchB falling edge detected\n");
        }
        else if (events & GPIO_IRQ_EDGE_RISE)
        {
            xEventGroupSetBits(xEventGroup, EVENT_TOUCHB_SWITCH_RISE);
            // DEBUG_PRINT("TouchB rising edge detected\n");
        }
    }
}
#if 0
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
#endif