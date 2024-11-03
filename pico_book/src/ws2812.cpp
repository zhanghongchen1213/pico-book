#include "ws2812.h"

static int dma_channel;

extern uint32_t data[REG_LED_SUM];

// 初始化 PIO（可编程输入/输出）的函数
void init_pio()
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
        count_of(data),
        false // 不自动启动传输
    );
}

// 发送数据的函数，将更新 data 数组的数据发送到 DMA
void ws2812_send_data()
{
    dma_channel_set_read_addr(dma_channel, data, true); // 设置 DMA 读取地址并启动传输
}