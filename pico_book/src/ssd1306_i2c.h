#ifndef __SSD1306_I2C_H
#define __SSD1306_I2C_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "raspberry26x32.h"
#include "ssd1306_font.h"
#include "debug.h"

#define SSD1306_SDA_PIN 2
#define SSD1306_SCL_PIN 3

#define SSD1306_HEIGHT 32
#define SSD1306_WIDTH 128

#define SSD1306_I2C_ADDR _u(0x3C)

// 400 是常见的，但通常可以超频以提高显示响应。
// 在 32 和 84 像素高度设备上测试过，工作正常。
#define SSD1306_I2C_CLK 400
// #define SSD1306_I2C_CLK             1000

// 命令（参见数据手册）
#define SSD1306_SET_MEM_MODE _u(0x20)
#define SSD1306_SET_COL_ADDR _u(0x21)
#define SSD1306_SET_PAGE_ADDR _u(0x22)
#define SSD1306_SET_HORIZ_SCROLL _u(0x26)
#define SSD1306_SET_SCROLL _u(0x2E)

#define SSD1306_SET_DISP_START_LINE _u(0x40)

#define SSD1306_SET_CONTRAST _u(0x81)
#define SSD1306_SET_CHARGE_PUMP _u(0x8D)

#define SSD1306_SET_SEG_REMAP _u(0xA0)
#define SSD1306_SET_ENTIRE_ON _u(0xA4)
#define SSD1306_SET_ALL_ON _u(0xA5)
#define SSD1306_SET_NORM_DISP _u(0xA6)
#define SSD1306_SET_INV_DISP _u(0xA7)
#define SSD1306_SET_MUX_RATIO _u(0xA8)
#define SSD1306_SET_DISP _u(0xAE)
#define SSD1306_SET_COM_OUT_DIR _u(0xC0)
#define SSD1306_SET_COM_OUT_DIR_FLIP _u(0xC0)

#define SSD1306_SET_DISP_OFFSET _u(0xD3)
#define SSD1306_SET_DISP_CLK_DIV _u(0xD5)
#define SSD1306_SET_PRECHARGE _u(0xD9)
#define SSD1306_SET_COM_PIN_CFG _u(0xDA)
#define SSD1306_SET_VCOM_DESEL _u(0xDB)

#define SSD1306_PAGE_HEIGHT _u(8)
#define SSD1306_NUM_PAGES (SSD1306_HEIGHT / SSD1306_PAGE_HEIGHT)
#define SSD1306_BUF_LEN (SSD1306_NUM_PAGES * SSD1306_WIDTH)

#define SSD1306_WRITE_MODE _u(0xFE)
#define SSD1306_READ_MODE _u(0xFF)

struct render_area
{
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_page;
    uint8_t end_page;
    int buflen;
};

#ifdef __cplusplus
extern "C"
{
#endif

    // 函数声明
    // 初始化SSD1306显示屏
    void SSD1306_init();

    // 发送命令到SSD1306显示屏
    // 参数: cmd - 要发送的命令
    void SSD1306_send_cmd(uint8_t cmd);

    // 发送命令列表到SSD1306显示屏
    // 参数: buf - 命令列表的缓冲区
    //       num - 命令的数量
    void SSD1306_send_cmd_list(uint8_t *buf, int num);

    // 发送数据缓冲区到SSD1306显示屏
    // 参数: buf - 数据缓冲区
    //       buflen - 缓冲区长度
    void SSD1306_send_buf(uint8_t buf[], int buflen);

    // 启动或停止滚动显示
    // 参数: on - 启动滚动为true，停止滚动为false
    void SSD1306_scroll(bool on);

    // 渲染指定区域的缓冲区
    // 参数: buf - 数据缓冲区
    //       area - 渲染区域
    void render(uint8_t *buf, struct render_area *area);

    // 设置指定像素点的状态
    // 参数: buf - 数据缓冲区
    //       x - 像素点的x坐标
    //       y - 像素点的y坐标
    //       on - 像素点状态，true为点亮，false为熄灭
    void SetPixel(uint8_t *buf, int x, int y, bool on);

    // 画一条直线
    // 参数: buf - 数据缓冲区
    //       x0 - 起点的x坐标
    //       y0 - 起点的y坐标
    //       x1 - 终点的x坐标
    //       y1 - 终点的y坐标
    //       on - 线条状态，true为点亮，false为熄灭
    void DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on);

    // 在指定位置写入一个字符
    // 参数: buf - 数据缓冲区
    //       x - 字符的x坐标
    //       y - 字符的y坐标
    //       ch - 要写入的字符
    void WriteChar(uint8_t *buf, int16_t x, int16_t y, uint8_t ch);

    // 在指定位置写入一个字符串
    // 参数: buf - 数据缓冲区
    //       x - 字符串的x坐标
    //       y - 字符串的y坐标
    //       str - 要写入的字符串
    void WriteString(uint8_t *buf, int16_t x, int16_t y, char *str);

    // 计算渲染区域的缓冲区长度
    // 参数: area - 渲染区域
    void calc_render_area_buflen(struct render_area *area);

    // 初始化SSD1306显示屏
    void ssd1306_begin();

#ifdef __cplusplus
}
#endif

#endif