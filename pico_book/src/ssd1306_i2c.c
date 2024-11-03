#include "ssd1306_i2c.h"

void calc_render_area_buflen(struct render_area *area)
{
    // 计算渲染区域的缓冲区长度
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
}

void SSD1306_send_cmd(uint8_t cmd)
{
    // I2C 写入过程需要一个控制字节后跟数据
    // 这个“数据”可以是命令或后续命令的数据
    // Co = 1, D/C = 0 => 驱动程序期望一个命令
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(i2c1, SSD1306_I2C_ADDR, buf, 2, false);
}

void SSD1306_send_cmd_list(uint8_t *buf, int num)
{
    for (int i = 0; i < num; i++)
    {
        SSD1306_send_cmd(buf[i]);
    }
}

void SSD1306_send_buf(uint8_t buf[], int buflen)
{
    // 在水平寻址模式下，列地址指针自动递增
    // 然后换到下一页，所以我们可以一次性发送整个帧缓冲区

    // 将我们的帧缓冲区复制到一个新缓冲区，因为我们需要在开头添加控制字节

    uint8_t *temp_buf = malloc(buflen + 1);

    temp_buf[0] = 0x40;
    memcpy(temp_buf + 1, buf, buflen);

    i2c_write_blocking(i2c1, SSD1306_I2C_ADDR, temp_buf, buflen + 1, false);

    free(temp_buf);
}

void SSD1306_init()
{
    // 其中一些命令不是严格必要的，因为复位过程默认使用其中一些命令，但它们在这里显示
    // 以演示初始化序列的样子
    // 一些配置值由板制造商推荐

    uint8_t cmds[] = {
        SSD1306_SET_DISP, // 设置显示关闭
        /* 内存映射 */
        SSD1306_SET_MEM_MODE, // 设置内存地址模式 0 = 水平，1 = 垂直，2 = 页面
        0x00,                 // 水平寻址模式
        /* 分辨率和布局 */
        SSD1306_SET_DISP_START_LINE,    // 设置显示起始行为 0
        SSD1306_SET_SEG_REMAP | 0x01,   // 设置段重映射，列地址 127 映射到 SEG0
        SSD1306_SET_MUX_RATIO,          // 设置多路复用比率
        SSD1306_HEIGHT - 1,             // 显示高度 - 1
        SSD1306_SET_COM_OUT_DIR | 0x08, // 设置 COM（公共）输出扫描方向。从下到上扫描，COM[N-1] 到 COM0
        SSD1306_SET_DISP_OFFSET,        // 设置显示偏移
        0x00,                           // 无偏移
        SSD1306_SET_COM_PIN_CFG,        // 设置 COM（公共）引脚硬件配置。板特定的魔术数字。
                                        // 0x02 适用于 128x32，0x12 可能适用于 128x64。其他选项 0x22, 0x32
#if ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 32))
        0x02,
#elif ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 64))
        0x12,
#else
        0x02,
#endif
        /* 定时和驱动方案 */
        SSD1306_SET_DISP_CLK_DIV, // 设置显示时钟分频比
        0x80,                     // 分频比为 1，标准频率
        SSD1306_SET_PRECHARGE,    // 设置预充电周期
        0xF1,                     // Vcc 在我们的板上内部生成
        SSD1306_SET_VCOM_DESEL,   // 设置 VCOMH 取消选择电平
        0x30,                     // 0.83xVcc
        /* 显示 */
        SSD1306_SET_CONTRAST, // 设置对比度控制
        0xFF,
        SSD1306_SET_ENTIRE_ON,     // 设置整个显示器跟随 RAM 内容
        SSD1306_SET_NORM_DISP,     // 设置正常（非反转）显示
        SSD1306_SET_CHARGE_PUMP,   // 设置充电泵
        0x14,                      // Vcc 在我们的板上内部生成
        SSD1306_SET_SCROLL | 0x00, // 禁用水平滚动。如果启用滚动，这是必要的，因为内存写入会被破坏
        SSD1306_SET_DISP | 0x01,   // 打开显示
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void SSD1306_scroll(bool on)
{
    // 配置水平滚动
    uint8_t cmds[] = {
        SSD1306_SET_HORIZ_SCROLL | 0x00,
        0x00,                                // 虚拟字节
        0x00,                                // 起始页 0
        0x00,                                // 时间间隔
        0x03,                                // 结束页 3 SSD1306_NUM_PAGES ??
        0x00,                                // 虚拟字节
        0xFF,                                // 虚拟字节
        SSD1306_SET_SCROLL | (on ? 0x01 : 0) // 启动/停止滚动
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void render(uint8_t *buf, struct render_area *area)
{
    // 使用渲染区域更新显示的一部分
    uint8_t cmds[] = {
        SSD1306_SET_COL_ADDR,
        area->start_col,
        area->end_col,
        SSD1306_SET_PAGE_ADDR,
        area->start_page,
        area->end_page};

    SSD1306_send_cmd_list(cmds, count_of(cmds));
    SSD1306_send_buf(buf, area->buflen);
}

void SetPixel(uint8_t *buf, int x, int y, bool on)
{
    assert(x >= 0 && x < SSD1306_WIDTH && y >= 0 && y < SSD1306_HEIGHT);

    // 确定要设置的正确位的计算取决于我们处于哪个地址模式。此代码假定为水平

    // SSD1306 上的视频 RAM 分为 8 行，每像素一位。
    // 每行长 128，每行高 8 像素，每字节垂直排列，因此字节 0 是 x=0, y=0->7，
    // 字节 1 是 x = 1, y=0->7 等

    // 这段代码可以优化，但为了清晰起见是这样的。编译器
    // 应该会做一个不错的优化。

    const int BytesPerRow = SSD1306_WIDTH; // x 像素，1bpp，但每行高 8 像素，所以 (x / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow + x;
    uint8_t byte = buf[byte_idx];

    if (on)
        byte |= 1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    buf[byte_idx] = byte;
}
// 基本的 Bresenhams 算法。
void DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on)
{

    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;

    while (true)
    {
        SetPixel(buf, x0, y0, on);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

inline int GetFontIndex(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'Z')
    {
        return ch - 'A' + 1;
    }
    else if (ch >= '0' && ch <= '9')
    {
        return ch - '0' + 27;
    }
    else
        return 0; // 没有该字符则为空格。
}

void WriteChar(uint8_t *buf, int16_t x, int16_t y, uint8_t ch)
{
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    // 目前，只在 Y 行边界（每 8 个垂直像素）上写入
    y = y / 8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i = 0; i < 8; i++)
    {
        buf[fb_idx++] = font[idx * 8 + i];
    }
}

void WriteString(uint8_t *buf, int16_t x, int16_t y, char *str)
{
    // 剔除屏幕外的任何字符串
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    while (*str)
    {
        WriteChar(buf, x, y, *str++);
        x += 8;
    }
}

void ssd1306_begin()
{
    i2c_init(i2c1, SSD1306_I2C_CLK * 1000);
    gpio_set_function(SSD1306_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SSD1306_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SSD1306_SDA_PIN);
    gpio_pull_up(SSD1306_SCL_PIN);
    // DEBUG_PRINT("SSD1306 I2C PIN init done\r\n");

    SSD1306_init();

    struct render_area frame_area = {
        start_col : 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
    };

    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t buf[SSD1306_BUF_LEN];
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++)
    {
        SSD1306_send_cmd(SSD1306_SET_ALL_ON); // Set all pixels on
        sleep_ms(500);
        SSD1306_send_cmd(SSD1306_SET_ENTIRE_ON); // go back to following RAM for pixel state
        sleep_ms(500);
    }

    // render 3 cute little raspberries
    struct render_area area = {
        start_page : 0,
        end_page : (IMG_HEIGHT / SSD1306_PAGE_HEIGHT) - 1
    };

restart:

    area.start_col = 0;
    area.end_col = IMG_WIDTH - 1;

    calc_render_area_buflen(&area);

    uint8_t offset = 5 + IMG_WIDTH; // 5px padding

    for (int i = 0; i < 3; i++)
    {
        render(raspberry26x32, &area);
        area.start_col += offset;
        area.end_col += offset;
    }

    SSD1306_scroll(true);
    sleep_ms(5000);
    SSD1306_scroll(false);

    char *text[] = {
        "A long time ago",
        "  on an OLED ",
        "   display",
        " far far away",
        "Lived a small",
        "red raspberry",
        "by the name of",
        "    PICO"};

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        WriteString(buf, 5, y, text[i]);
        y += 8;
    }
    render(buf, &frame_area);

    // Test the display invert function
    sleep_ms(3000);
    SSD1306_send_cmd(SSD1306_SET_INV_DISP);
    sleep_ms(3000);
    SSD1306_send_cmd(SSD1306_SET_NORM_DISP);

    bool pix = true;
    for (int i = 0; i < 2; i++)
    {
        for (int x = 0; x < SSD1306_WIDTH; x++)
        {
            DrawLine(buf, x, 0, SSD1306_WIDTH - 1 - x, SSD1306_HEIGHT - 1, pix);
            render(buf, &frame_area);
        }

        for (int y = SSD1306_HEIGHT - 1; y >= 0; y--)
        {
            DrawLine(buf, 0, y, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1 - y, pix);
            render(buf, &frame_area);
        }
        pix = false;
    }

    goto restart;
}
