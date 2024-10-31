// MAX30105.h

#ifndef _MAX30105_H_
#define _MAX30105_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MAX30105 I2C 地址
#define MAX30105_ADDRESS 0x57 // 7-bit I2C Address

class MAX30105
{
public:
    MAX30105();

    bool begin(i2c_inst_t *i2c, uint8_t sda_pin, uint8_t scl_pin, uint32_t i2c_speed = 100000, uint8_t i2caddr = MAX30105_ADDRESS);

    uint32_t getRed(void);   // 返回红光值
    uint32_t getIR(void);    // 返回红外光值
    uint32_t getGreen(void); // 返回绿光值

    bool safeCheck(uint8_t maxTimeToCheck); // 在指定时间内检查新数据

    // 配置函数
    void softReset();
    void shutDown();
    void wakeUp();

    void setLEDMode(uint8_t mode);

    void setADCRange(uint8_t adcRange);
    void setSampleRate(uint8_t sampleRate);
    void setPulseWidth(uint8_t pulseWidth);

    void setPulseAmplitudeRed(uint8_t value);
    void setPulseAmplitudeIR(uint8_t value);
    void setPulseAmplitudeGreen(uint8_t value);

    void enableSlot(uint8_t slotNumber, uint8_t device);
    void disableSlots(void);

    // FIFO 配置
    void setFIFOAverage(uint8_t samples);
    void enableFIFORollover();
    void disableFIFORollover();
    void setFIFOAlmostFull(uint8_t samples);

    void clearFIFO(void); // 重置 FIFO

private:
    i2c_inst_t *_i2c; // I2C 接口
    uint8_t _i2caddr; // I2C 地址
    uint8_t _sda_pin; // SDA 引脚
    uint8_t _scl_pin; // SCL 引脚

    uint8_t activeLEDs; // 活动的 LED 数量

    uint8_t revisionID;

    // 读写寄存器
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    // 位掩码操作
    void bitMask(uint8_t reg, uint8_t mask, uint8_t value);

    // 检查新数据
    uint16_t check(void);

    // 内部数据缓冲区
    static const int STORAGE_SIZE = 4;
    struct Record
    {
        uint32_t red[STORAGE_SIZE];
        uint32_t IR[STORAGE_SIZE];
        uint32_t green[STORAGE_SIZE];
        uint8_t head;
        uint8_t tail;
    } sense;

    uint8_t getReadPointer(void);
    uint8_t getWritePointer(void);
};

extern MAX30105 heart;
#endif // _MAX30105_H_
