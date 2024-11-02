#ifndef __MAX30102_H
#define __MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "debug.h"
#include "heartRate.h"

// MAX30102 7位I2C地址
#define MAX30105_ADDRESS 0x57

// 定义I2C速度
#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

// 定义I2C缓冲区长度（根据树莓派Pico的I2C缓冲区大小调整）
#define I2C_BUFFER_LENGTH 32
// 状态寄存器
#define MAX30105_INTSTAT1 0x00
#define MAX30105_INTSTAT2 0x01
#define MAX30105_INTENABLE1 0x02
#define MAX30105_INTENABLE2 0x03

// FIFO寄存器
#define MAX30105_FIFOWRITEPTR 0x04
#define MAX30105_FIFOOVERFLOW 0x05
#define MAX30105_FIFOREADPTR 0x06
#define MAX30105_FIFODATA 0x07

// 配置寄存器
#define MAX30105_FIFOCONFIG 0x08
#define MAX30105_MODECONFIG 0x09
#define MAX30105_PARTICLECONFIG 0x0A // 注意，有时在数据手册中列为“SPO2”配置（第11页）
#define MAX30105_LED1_PULSEAMP 0x0C
#define MAX30105_LED2_PULSEAMP 0x0D
#define MAX30105_LED3_PULSEAMP 0x0E
#define MAX30105_LED_PROX_AMP 0x10
#define MAX30105_MULTILEDCONFIG1 0x11
#define MAX30105_MULTILEDCONFIG2 0x12

// 芯片温度寄存器
#define MAX30105_DIETEMPINT 0x1F
#define MAX30105_DIETEMPFRAC 0x20
#define MAX30105_DIETEMPCONFIG 0x21

// 接近功能寄存器
#define MAX30105_PROXINTTHRESH 0x30

// 部件ID寄存器
#define MAX30105_REVISIONID 0xFE
#define MAX30105_PARTID 0xFF // 应始终为0x15。与MAX30102相同。

// MAX30105命令
// 中断配置（第13,14页）
#define MAX30105_INT_A_FULL_MASK (uint8_t) ~0b10000000
#define MAX30105_INT_A_FULL_ENABLE 0x80
#define MAX30105_INT_A_FULL_DISABLE 0x00

#define MAX30105_INT_DATA_RDY_MASK (uint8_t) ~0b01000000
#define MAX30105_INT_DATA_RDY_ENABLE 0x40
#define MAX30105_INT_DATA_RDY_DISABLE 0x00

#define MAX30105_INT_ALC_OVF_MASK (uint8_t) ~0b00100000
#define MAX30105_INT_ALC_OVF_ENABLE 0x20
#define MAX30105_INT_ALC_OVF_DISABLE 0x00

#define MAX30105_INT_PROX_INT_MASK (uint8_t) ~0b00010000
#define MAX30105_INT_PROX_INT_ENABLE 0x10
#define MAX30105_INT_PROX_INT_DISABLE 0x00

#define MAX30105_INT_DIE_TEMP_RDY_MASK (uint8_t) ~0b00000010
#define MAX30105_INT_DIE_TEMP_RDY_ENABLE 0x02
#define MAX30105_INT_DIE_TEMP_RDY_DISABLE 0x00

#define MAX30105_SAMPLEAVG_MASK (uint8_t) ~0b11100000
#define MAX30105_SAMPLEAVG_1 0x00
#define MAX30105_SAMPLEAVG_2 0x20
#define MAX30105_SAMPLEAVG_4 0x40
#define MAX30105_SAMPLEAVG_8 0x60
#define MAX30105_SAMPLEAVG_16 0x80
#define MAX30105_SAMPLEAVG_32 0xA0

#define MAX30105_ROLLOVER_MASK 0xEF
#define MAX30105_ROLLOVER_ENABLE 0x10
#define MAX30105_ROLLOVER_DISABLE 0x00

#define MAX30105_A_FULL_MASK 0xF0

// 模式配置命令（第19页）
#define MAX30105_SHUTDOWN_MASK 0x7F
#define MAX30105_SHUTDOWN 0x80
#define MAX30105_WAKEUP 0x00

#define MAX30105_RESET_MASK 0xBF
#define MAX30105_RESET 0x40

#define MAX30105_MODE_MASK 0xF8
#define MAX30105_MODE_REDONLY 0x02
#define MAX30105_MODE_REDIRONLY 0x03
#define MAX30105_MODE_MULTILED 0x07

// 粒子传感配置命令（第19-20页）
#define MAX30105_ADCRANGE_MASK 0x9F
#define MAX30105_ADCRANGE_2048 0x00
#define MAX30105_ADCRANGE_4096 0x20
#define MAX30105_ADCRANGE_8192 0x40
#define MAX30105_ADCRANGE_16384 0x60

#define MAX30105_SAMPLERATE_MASK 0xE3
#define MAX30105_SAMPLERATE_50 0x00
#define MAX30105_SAMPLERATE_100 0x04
#define MAX30105_SAMPLERATE_200 0x08
#define MAX30105_SAMPLERATE_400 0x0C
#define MAX30105_SAMPLERATE_800 0x10
#define MAX30105_SAMPLERATE_1000 0x14
#define MAX30105_SAMPLERATE_1600 0x18
#define MAX30105_SAMPLERATE_3200 0x1C

#define MAX30105_PULSEWIDTH_MASK 0xFC
#define MAX30105_PULSEWIDTH_69 0x00
#define MAX30105_PULSEWIDTH_118 0x01
#define MAX30105_PULSEWIDTH_215 0x02
#define MAX30105_PULSEWIDTH_411 0x03

// 多LED模式配置（第22页）
#define MAX30105_SLOT1_MASK 0xF8
#define MAX30105_SLOT2_MASK 0x8F
#define MAX30105_SLOT3_MASK 0xF8
#define MAX30105_SLOT4_MASK 0x8F

#define SLOT_NONE 0x00
#define SLOT_RED_LED 0x01
#define SLOT_IR_LED 0x02
#define SLOT_GREEN_LED 0x03
#define SLOT_NONE_PILOT 0x04
#define SLOT_RED_PILOT 0x05
#define SLOT_IR_PILOT 0x06
#define SLOT_GREEN_PILOT 0x07

#define MAX_30105_EXPECTEDPARTID 0x15

// MAX30105在IC上最多存储32个样本
// 这是微控制器的额外本地存储
#define STORAGE_SIZE 4 // 每个long是4个字节，因此限制此值以适应您的微控制器

// MAX30102类定义
class MAX30105
{
public:
    MAX30105(void);

    bool begin(i2c_inst_t *i2c, uint8_t sda_pin = PICO_DEFAULT_I2C_SDA_PIN, uint8_t scl_pin = PICO_DEFAULT_I2C_SCL_PIN, uint32_t i2cSpeed = I2C_SPEED_FAST, uint8_t i2caddr = MAX30105_ADDRESS);

    uint32_t getRed(void);                  // 返回即时红色值
    uint32_t getIR(void);                   // 返回即时红外值
    uint32_t getGreen(void);                // 返回即时绿色值
    bool safeCheck(uint8_t maxTimeToCheck); // 给定最大检查时间，检查新数据

    // 配置
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
    void setPulseAmplitudeProximity(uint8_t value);

    void setProximityThreshold(uint8_t threshMSB);

    // 多LED配置模式（第22页）
    void enableSlot(uint8_t slotNumber, uint8_t device); // 给定槽号，分配设备到槽
    void disableSlots(void);

    // 数据采集

    // 中断（第13,14页）
    uint8_t getINT1(void);  // 返回主中断组
    uint8_t getINT2(void);  // 返回温度准备中断
    void enableAFULL(void); // 启用/禁用单个中断
    void disableAFULL(void);
    void enableDATARDY(void);
    void disableDATARDY(void);
    void enableALCOVF(void);
    void disableALCOVF(void);
    void enablePROXINT(void);
    void disablePROXINT(void);
    void enableDIETEMPRDY(void);
    void disableDIETEMPRDY(void);

    // FIFO配置（第18页）
    void setFIFOAverage(uint8_t samples);
    void enableFIFORollover();
    void disableFIFORollover();
    void setFIFOAlmostFull(uint8_t samples);

    // FIFO读取
    uint16_t check(void);        // 检查新数据并填充FIFO
    uint8_t available(void);     // 告诉调用者有多少新样本可用（头 - 尾）
    void nextSample(void);       // 前进感应数组的尾部
    uint32_t getFIFORed(void);   // 返回FIFO样本指向的红色值
    uint32_t getFIFOIR(void);    // 返回FIFO样本指向的红外值
    uint32_t getFIFOGreen(void); // 返回FIFO样本指向的绿色值

    uint8_t getWritePointer(void);
    uint8_t getReadPointer(void);
    void clearFIFO(void); // 将读/写指针设置为零

    // 接近模式中断阈值
    void setPROXINTTHRESH(uint8_t val);

    // 芯片温度
    float readTemperature();
    float readTemperatureF();

    // 检测ID/修订版
    uint8_t getRevisionID();
    uint8_t readPartID();

    // 使用用户可选设置设置IC
    void setup(uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

    // 低级I2C通信
    uint8_t readRegister8(uint8_t address, uint8_t reg);
    void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

private:
    i2c_inst_t *_i2cPort; // I2C端口实例
    uint8_t _i2caddr;
    uint8_t _sda_pin; // SDA 引脚
    uint8_t _scl_pin; // SCL 引脚

    // activeLEDs是开启的通道数，可以是1到3。红+红外常见为2。
    uint8_t activeLEDs; // 在设置期间设置。允许check()计算从FIFO读取的字节数

    uint8_t revisionID;

    void readRevisionID();

    void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
};

extern MAX30105 particleSensor;

#endif
