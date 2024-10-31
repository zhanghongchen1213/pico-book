// MAX30105.cpp

#include "max30102.h"
#include <string.h> // 用于 memcpy

// Status Registers
static const uint8_t MAX30105_INTSTAT1 = 0x00;
static const uint8_t MAX30105_INTSTAT2 = 0x01;
static const uint8_t MAX30105_INTENABLE1 = 0x02;
static const uint8_t MAX30105_INTENABLE2 = 0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 0x05;
static const uint8_t MAX30105_FIFOREADPTR = 0x06;
static const uint8_t MAX30105_FIFODATA = 0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 0x08;
static const uint8_t MAX30105_MODECONFIG = 0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 0x0A; // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 0xFE;
static const uint8_t MAX30105_PARTID = 0xFF; // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK = (unsigned int)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK = (unsigned int)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK = (unsigned int)~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK = (unsigned int)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (unsigned int)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK = (unsigned int)~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK = 0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 0x7F;
static const uint8_t MAX30105_SHUTDOWN = 0x80;
static const uint8_t MAX30105_WAKEUP = 0x00;

static const uint8_t MAX30105_RESET_MASK = 0xBF;
static const uint8_t MAX30105_RESET = 0x40;

static const uint8_t MAX30105_MODE_MASK = 0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 0x03;
static const uint8_t MAX30105_MODE_MULTILED = 0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 0x03;

// Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 0x8F;

static const uint8_t SLOT_NONE = 0x00;
static const uint8_t SLOT_RED_LED = 0x01;
static const uint8_t SLOT_IR_LED = 0x02;
static const uint8_t SLOT_GREEN_LED = 0x03;
static const uint8_t SLOT_NONE_PILOT = 0x04;
static const uint8_t SLOT_RED_PILOT = 0x05;
static const uint8_t SLOT_IR_PILOT = 0x06;
static const uint8_t SLOT_GREEN_PILOT = 0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

MAX30105 heart;
// 构造函数
MAX30105::MAX30105()
{
    // 初始化成员变量
    _i2c = i2c0;
    _i2caddr = MAX30105_ADDRESS;
    _sda_pin = 4;
    _scl_pin = 5;
    activeLEDs = 0;
    memset(&sense, 0, sizeof(sense));
}

// 初始化函数
bool MAX30105::begin(i2c_inst_t *i2c, uint8_t sda_pin, uint8_t scl_pin, uint32_t i2c_speed, uint8_t i2caddr)
{
    _i2c = i2c;
    _sda_pin = sda_pin;
    _scl_pin = scl_pin;
    _i2caddr = i2caddr;

    // 初始化 I2C 接口和引脚
    i2c_init(_i2c, i2c_speed);
    gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_sda_pin);
    gpio_pull_up(_scl_pin);

    // 检查传感器是否连接
    uint8_t partID = readRegister8(MAX30105_PARTID); // 读取 Part ID 寄存器
    if (partID != MAX_30105_EXPECTEDPARTID)
    { // MAX30105 的 Part ID 应为 0x15
        return false;
    }
    printf("Part ID: 0x%02X\n", partID);

    readRegister8(MAX30105_REVISIONID); // 读取 Revision ID 寄存器

    printf("Revision ID: 0x%02X\n", revisionID);

    return true;
}

// 读取 8 位寄存器
uint8_t MAX30105::readRegister8(uint8_t reg)
{
    uint8_t data;
    int ret;

    // 发送寄存器地址
    ret = i2c_write_blocking(_i2c, _i2caddr, &reg, 1, true); // true 表示保持总线（重复开始条件）
    if (ret != 1)
    {
        return 0; // 失败
    }

    // 读取数据
    ret = i2c_read_blocking(_i2c, _i2caddr, &data, 1, false); // false 表示停止总线
    if (ret != 1)
    {
        return 0; // 失败
    }

    return data;
}

// 写入 8 位寄存器
void MAX30105::writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;

    i2c_write_blocking(_i2c, _i2caddr, buffer, 2, false);
}

// 位掩码操作
void MAX30105::bitMask(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t original = readRegister8(reg);
    original = (original & mask) | value;
    writeRegister8(reg, original);
}

// 设置 LED 模式
void MAX30105::setLEDMode(uint8_t mode)
{
    // 设置 LED 模式寄存器，假设寄存器地址为 0x09
    writeRegister8(0x09, mode);
    activeLEDs = mode;
}

// 设置 ADC 范围
void MAX30105::setADCRange(uint8_t adcRange)
{
    // 设置 ADC 范围寄存器，假设寄存器地址为 0x0A
    bitMask(0x0A, 0x9F, adcRange);
}

// 设置采样率
void MAX30105::setSampleRate(uint8_t sampleRate)
{
    // 设置采样率寄存器，假设寄存器地址为 0x0A
    bitMask(0x0A, 0xE3, sampleRate);
}

// 设置脉冲宽度
void MAX30105::setPulseWidth(uint8_t pulseWidth)
{
    // 设置脉冲宽度寄存器，假设寄存器地址为 0x0A
    bitMask(0x0A, 0xFC, pulseWidth);
}

// 设置红光 LED 电流
void MAX30105::setPulseAmplitudeRed(uint8_t value)
{
    // 假设寄存器地址为 0x0C
    writeRegister8(0x0C, value);
}

// 设置红外光 LED 电流
void MAX30105::setPulseAmplitudeIR(uint8_t value)
{
    // 假设寄存器地址为 0x0D
    writeRegister8(0x0D, value);
}

// 设置绿光 LED 电流
void MAX30105::setPulseAmplitudeGreen(uint8_t value)
{
    // 假设寄存器地址为 0x0E
    writeRegister8(0x0E, value);
}

// 其他函数实现...
// 例如 softReset(), shutDown(), wakeUp(), getRed(), getIR(), getGreen(), safeCheck(), check(), 等。

// 由于篇幅限制，我将在下面提供关键的函数实现。

// 获取红光值
uint32_t MAX30105::getRed(void)
{
    // 检查新数据
    if (safeCheck(250))
    {
        return sense.red[sense.head];
    }
    else
    {
        return 0;
    }
}

// 获取红外光值
uint32_t MAX30105::getIR(void)
{
    // 检查新数据
    if (safeCheck(250))
    {
        return sense.IR[sense.head];
    }
    else
    {
        return 0;
    }
}

// 获取绿光值
uint32_t MAX30105::getGreen(void)
{
    // 检查新数据
    if (safeCheck(250))
    {
        return sense.green[sense.head];
    }
    else
    {
        return 0;
    }
}

// 安全检查新数据
bool MAX30105::safeCheck(uint8_t maxTimeToCheck)
{
    uint64_t startTime = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - startTime < maxTimeToCheck)
    {
        if (check())
        {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

// 检查新数据
uint16_t MAX30105::check(void)
{
    uint8_t readPointer = getReadPointer();
    uint8_t writePointer = getWritePointer();

    int numberOfSamples = 0;

    if (readPointer != writePointer)
    {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0)
        {
            numberOfSamples += 32; // 环绕条件
        }

        // 读取 FIFO 数据
        // 根据 activeLEDs 和 numberOfSamples 计算需要读取的字节数
        int bytesToRead = numberOfSamples * activeLEDs * 3;

        uint8_t fifoData[bytesToRead];

        // 读取 FIFO 数据，假设 FIFO 数据寄存器地址为 0x07
        uint8_t reg = 0x07;
        int ret = i2c_write_blocking(_i2c, _i2caddr, &reg, 1, true);
        if (ret != 1)
        {
            return 0;
        }

        ret = i2c_read_blocking(_i2c, _i2caddr, fifoData, bytesToRead, false);
        if (ret != bytesToRead)
        {
            return 0;
        }

        // 解析 FIFO 数据
        for (int i = 0; i < numberOfSamples; i++)
        {
            sense.head++;
            sense.head %= STORAGE_SIZE;

            uint32_t red = 0;
            uint32_t ir = 0;
            uint32_t green = 0;

            int index = i * activeLEDs * 3;

            // 读取红光数据
            red = ((uint32_t)fifoData[index] << 16) | ((uint32_t)fifoData[index + 1] << 8) | fifoData[index + 2];
            red &= 0x03FFFF; // 18 位有效数据

            sense.red[sense.head] = red;

            if (activeLEDs > 1)
            {
                // 读取红外光数据
                index += 3;
                ir = ((uint32_t)fifoData[index] << 16) | ((uint32_t)fifoData[index + 1] << 8) | fifoData[index + 2];
                ir &= 0x03FFFF;

                sense.IR[sense.head] = ir;
            }

            if (activeLEDs > 2)
            {
                // 读取绿光数据
                index += 3;
                green = ((uint32_t)fifoData[index] << 16) | ((uint32_t)fifoData[index + 1] << 8) | fifoData[index + 2];
                green &= 0x03FFFF;

                sense.green[sense.head] = green;
            }
        }

        return numberOfSamples;
    }

    return 0;
}

// 获取 FIFO 读指针
uint8_t MAX30105::getReadPointer(void)
{
    // 假设 FIFO 读指针寄存器地址为 0x06
    return readRegister8(0x06);
}

// 获取 FIFO 写指针
uint8_t MAX30105::getWritePointer(void)
{
    // 假设 FIFO 写指针寄存器地址为 0x04
    return readRegister8(0x04);
}

// 其他必要的函数实现...
