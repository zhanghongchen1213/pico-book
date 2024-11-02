#include "max30102.h"

// 数据存储结构体
struct Record
{
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
} sense;

MAX30105 particleSensor;

MAX30105::MAX30105()
{
    // Constructor
}

bool MAX30105::begin(i2c_inst_t *i2c, uint8_t sda_pin, uint8_t scl_pin, uint32_t i2cSpeed, uint8_t i2caddr)
{
    // 设置 I2C 端口和地址
    _i2cPort = i2c;
    _i2caddr = i2caddr;
    _sda_pin = sda_pin;
    _scl_pin = scl_pin;

    // 初始化 I2C 端口并设置波特率
    i2c_init(_i2cPort, i2cSpeed);
    gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_sda_pin);
    gpio_pull_up(_scl_pin);

    // 检查是否连接了 MAX30105
    if (readPartID() != MAX_30105_EXPECTEDPARTID)
    {
        // 错误：读取的 Part ID 与预期的不符，可能是连接问题
        return false;
    }

    // 获取修订版本 ID
    readRevisionID();

    return true;
}

//
// 配置
//

// 开始中断配置
uint8_t MAX30105::getINT1(void)
{
    return (readRegister8(_i2caddr, MAX30105_INTSTAT1));
}
uint8_t MAX30105::getINT2(void)
{
    return (readRegister8(_i2caddr, MAX30105_INTSTAT2));
}

void MAX30105::enableAFULL(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE);
}
void MAX30105::disableAFULL(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE);
}

void MAX30105::enableDATARDY(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE);
}
void MAX30105::disableDATARDY(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE);
}

void MAX30105::enableALCOVF(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE);
}
void MAX30105::disableALCOVF(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE);
}

void MAX30105::enablePROXINT(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE);
}
void MAX30105::disablePROXINT(void)
{
    bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE);
}

void MAX30105::enableDIETEMPRDY(void)
{
    bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30105::disableDIETEMPRDY(void)
{
    bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE);
}

// 结束中断配置

void MAX30105::softReset(void)
{
    bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);
    absolute_time_t startTime = get_absolute_time();
    while (absolute_time_diff_us(startTime, get_absolute_time()) < 100000) // 超时100ms
    {
        uint8_t response = readRegister8(_i2caddr, MAX30105_MODECONFIG);
        if ((response & MAX30105_RESET) == 0)
            break;   // 复位完成
        sleep_ms(1); // 延时1ms，避免过度占用I2C总线
    }
}

void MAX30105::shutDown(void)
{
    // 将IC置于低功耗模式（数据手册第19页）
    // 在关机期间，IC将继续响应I2C命令，但不会更新或进行新的读数（如温度）
    bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void MAX30105::wakeUp(void)
{
    // 将IC从低功耗模式中拉出（数据手册第19页）
    bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void MAX30105::setLEDMode(uint8_t mode)
{
    // 设置用于采样的LED -- 仅红色，红色+红外，或自定义。
    // 参见数据手册第19页
    bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void MAX30105::setADCRange(uint8_t adcRange)
{
    // adcRange: 其中之一 MAX30105_ADCRANGE_2048, _4096, _8192, _16384
    bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void MAX30105::setSampleRate(uint8_t sampleRate)
{
    // sampleRate: 其中之一 MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
    bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void MAX30105::setPulseWidth(uint8_t pulseWidth)
{
    // pulseWidth: 其中之一 MAX30105_PULSEWIDTH_69, _188, _215, _411
    bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// 注意: 幅度值: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (典型值)
// 参见数据手册第21页
void MAX30105::setPulseAmplitudeRed(uint8_t amplitude)
{
    writeRegister8(_i2caddr, MAX30105_LED1_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeIR(uint8_t amplitude)
{
    writeRegister8(_i2caddr, MAX30105_LED2_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeGreen(uint8_t amplitude)
{
    writeRegister8(_i2caddr, MAX30105_LED3_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeProximity(uint8_t amplitude)
{
    writeRegister8(_i2caddr, MAX30105_LED_PROX_AMP, amplitude);
}

void MAX30105::setProximityThreshold(uint8_t threshMSB)
{
    // 设置将触发粒子感应模式开始的IR ADC计数。
    // threshMSB仅表示ADC计数的8个最高有效位。
    // 参见数据手册第24页。
    writeRegister8(_i2caddr, MAX30105_PROXINTTHRESH, threshMSB);
}
// 根据槽号分配设备
// 设备可以是 SLOT_RED_LED 或 SLOT_RED_PILOT（接近传感器）
// 分配 SLOT_RED_LED 将脉冲 LED
// 分配 SLOT_RED_PILOT 将 ??
void MAX30105::enableSlot(uint8_t slotNumber, uint8_t device)
{
    switch (slotNumber)
    {
    case (1):
        bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
        break;
    case (2):
        bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
        break;
    case (3):
        bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
        break;
    case (4):
        bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
        break;
    default:
        // 不应该到这里！
        break;
    }
}

// 清除所有槽分配
void MAX30105::disableSlots(void)
{
    writeRegister8(_i2caddr, MAX30105_MULTILEDCONFIG1, 0);
    writeRegister8(_i2caddr, MAX30105_MULTILEDCONFIG2, 0);
}

//
// FIFO 配置
//

// 设置采样平均值（表3，第18页）
void MAX30105::setFIFOAverage(uint8_t numberOfSamples)
{
    bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

// 重置所有点以在已知状态下开始
// 第15页建议在开始读取之前清除FIFO
void MAX30105::clearFIFO(void)
{
    writeRegister8(_i2caddr, MAX30105_FIFOWRITEPTR, 0);
    writeRegister8(_i2caddr, MAX30105_FIFOOVERFLOW, 0);
    writeRegister8(_i2caddr, MAX30105_FIFOREADPTR, 0);
}

// 如果FIFO溢出，启用滚动
void MAX30105::enableFIFORollover(void)
{
    bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

// 如果FIFO溢出，禁用滚动
void MAX30105::disableFIFORollover(void)
{
    bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

// 设置触发几乎满中断的样本数（第18页）
// 上电默认值为32个样本
// 注意这是反向的：0x00是32个样本，0x0F是17个样本
void MAX30105::setFIFOAlmostFull(uint8_t numberOfSamples)
{
    bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

// 读取FIFO写指针
uint8_t MAX30105::getWritePointer(void)
{
    return (readRegister8(_i2caddr, MAX30105_FIFOWRITEPTR));
}

// 读取FIFO读指针
uint8_t MAX30105::getReadPointer(void)
{
    return (readRegister8(_i2caddr, MAX30105_FIFOREADPTR));
}

// 芯片温度
// 返回温度（摄氏度）
float MAX30105::readTemperature()
{
    // 第一步：配置芯片温度寄存器以获取1个温度样本
    writeRegister8(_i2caddr, MAX30105_DIETEMPCONFIG, 0x01);

    // 轮询位以清除，读取完成
    // 超时100ms
    // 设置超时时间
    absolute_time_t startTime = get_absolute_time();
    while (absolute_time_diff_us(startTime, get_absolute_time()) < 100000) // 超时100ms
    {
        uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG);
        if ((response & 0x01) == 0)
            break;   // 完成！
        sleep_ms(1); // 避免过度占用I2C总线
    }
    // TODO 如何处理失败？使用什么类型的错误？
    //? if(millis() - startTime >= 100) return(-999.0);

    // 第二步：读取芯片温度寄存器（整数）
    int8_t tempInt = readRegister8(_i2caddr, MAX30105_DIETEMPINT);
    uint8_t tempFrac = readRegister8(_i2caddr, MAX30105_DIETEMPFRAC);

    // 第三步：计算温度（数据手册第23页）
    return (float)tempInt + ((float)tempFrac * 0.0625);
}

// 返回芯片温度（华氏度）
float MAX30105::readTemperatureF()
{
    float temp = readTemperature();

    if (temp != -999.0)
        temp = temp * 1.8 + 32.0;

    return (temp);
}

// 设置接近中断阈值
void MAX30105::setPROXINTTHRESH(uint8_t val)
{
    writeRegister8(_i2caddr, MAX30105_PROXINTTHRESH, val);
}

//
// 设备ID和修订版本
//
uint8_t MAX30105::readPartID()
{
    return readRegister8(_i2caddr, MAX30105_PARTID);
}

void MAX30105::readRevisionID()
{
    revisionID = readRegister8(_i2caddr, MAX30105_REVISIONID);
}

uint8_t MAX30105::getRevisionID()
{
    return revisionID;
}

// 传感器设置
// MAX30105有许多设置。默认选择：
//  样本平均值 = 4
//  模式 = 多LED
//  ADC范围 = 16384（每LSB 62.5pA）
//  采样率 = 50
// 如果你刚开始使用MAX30105传感器，请使用默认设置
void MAX30105::setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
    softReset(); // 重置所有配置、阈值和数据寄存器到上电复位值

    // FIFO配置
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // 如果你希望，芯片将平均多个相同类型的样本
    if (sampleAverage == 1)
        setFIFOAverage(MAX30105_SAMPLEAVG_1); // 每个FIFO记录不进行平均
    else if (sampleAverage == 2)
        setFIFOAverage(MAX30105_SAMPLEAVG_2);
    else if (sampleAverage == 4)
        setFIFOAverage(MAX30105_SAMPLEAVG_4);
    else if (sampleAverage == 8)
        setFIFOAverage(MAX30105_SAMPLEAVG_8);
    else if (sampleAverage == 16)
        setFIFOAverage(MAX30105_SAMPLEAVG_16);
    else if (sampleAverage == 32)
        setFIFOAverage(MAX30105_SAMPLEAVG_32);
    else
        setFIFOAverage(MAX30105_SAMPLEAVG_4);

    // setFIFOAlmostFull(2); // 设置为30个样本以触发“几乎满”中断
    enableFIFORollover(); // 允许FIFO环绕/滚动
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // 模式配置
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (ledMode == 3)
        setLEDMode(MAX30105_MODE_MULTILED); // 监视所有三个LED通道
    else if (ledMode == 2)
        setLEDMode(MAX30105_MODE_REDIRONLY); // 红色和红外
    else
        setLEDMode(MAX30105_MODE_REDONLY); // 仅红色
    activeLEDs = ledMode;                  // 用于控制从FIFO缓冲区读取的字节数
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // 粒子感应配置
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (adcRange < 4096)
        setADCRange(MAX30105_ADCRANGE_2048); // 每LSB 7.81pA
    else if (adcRange < 8192)
        setADCRange(MAX30105_ADCRANGE_4096); // 每LSB 15.63pA
    else if (adcRange < 16384)
        setADCRange(MAX30105_ADCRANGE_8192); // 每LSB 31.25pA
    else if (adcRange == 16384)
        setADCRange(MAX30105_ADCRANGE_16384); // 每LSB 62.5pA
    else
        setADCRange(MAX30105_ADCRANGE_2048);

    if (sampleRate < 100)
        setSampleRate(MAX30105_SAMPLERATE_50); // 每秒采样50次
    else if (sampleRate < 200)
        setSampleRate(MAX30105_SAMPLERATE_100);
    else if (sampleRate < 400)
        setSampleRate(MAX30105_SAMPLERATE_200);
    else if (sampleRate < 800)
        setSampleRate(MAX30105_SAMPLERATE_400);
    else if (sampleRate < 1000)
        setSampleRate(MAX30105_SAMPLERATE_800);
    else if (sampleRate < 1600)
        setSampleRate(MAX30105_SAMPLERATE_1000);
    else if (sampleRate < 3200)
        setSampleRate(MAX30105_SAMPLERATE_1600);
    else if (sampleRate == 3200)
        setSampleRate(MAX30105_SAMPLERATE_3200);
    else
        setSampleRate(MAX30105_SAMPLERATE_50);

    // 脉冲宽度越长，检测范围越长
    // 在69us和0.4mA时约2英寸
    // 在411us和0.4mA时约6英寸
    if (pulseWidth < 118)
        setPulseWidth(MAX30105_PULSEWIDTH_69); // 第26页，获得15位分辨率
    else if (pulseWidth < 215)
        setPulseWidth(MAX30105_PULSEWIDTH_118); // 16位分辨率
    else if (pulseWidth < 411)
        setPulseWidth(MAX30105_PULSEWIDTH_215); // 17位分辨率
    else if (pulseWidth == 411)
        setPulseWidth(MAX30105_PULSEWIDTH_411); // 18位分辨率
    else
        setPulseWidth(MAX30105_PULSEWIDTH_69);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // LED脉冲幅度配置
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // 默认值为0x1F，获得6.4mA
    // powerLevel = 0x02, 0.4mA - 检测范围约4英寸
    // powerLevel = 0x1F, 6.4mA - 检测范围约8英寸
    // powerLevel = 0x7F, 25.4mA - 检测范围约8英寸
    // powerLevel = 0xFF, 50.0mA - 检测范围约12英寸

    setPulseAmplitudeRed(powerLevel);
    setPulseAmplitudeIR(powerLevel);
    setPulseAmplitudeGreen(powerLevel);
    setPulseAmplitudeProximity(powerLevel);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // 多LED模式配置，启用三个LED的读取
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    enableSlot(1, SLOT_RED_LED);
    if (ledMode > 1)
        enableSlot(2, SLOT_IR_LED);
    if (ledMode > 2)
        enableSlot(3, SLOT_GREEN_LED);
    // enableSlot(1, SLOT_RED_PILOT);
    // enableSlot(2, SLOT_IR_PILOT);
    // enableSlot(3, SLOT_GREEN_PILOT);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    clearFIFO(); // 在开始检查传感器之前重置FIFO
}

//
// 数据采集
//

// 告诉调用者有多少样本可用
uint8_t MAX30105::available(void)
{
    int8_t numberOfSamples = sense.head - sense.tail;
    if (numberOfSamples < 0)
        numberOfSamples += STORAGE_SIZE;

    return (numberOfSamples);
}

// 报告最新的红色值
uint32_t MAX30105::getRed(void)
{
    // 检查传感器是否有新的数据，等待250ms
    if (safeCheck(250))
        return (sense.red[sense.head]);
    else
        return (0); // 传感器未找到新数据
}

// 报告最新的红外值
uint32_t MAX30105::getIR(void)
{
    // 检查传感器是否有新的数据，等待250ms
    if (safeCheck(250))
        return (sense.IR[sense.head]);
    else
        return (0); // 传感器未找到新数据
}

// 报告最新的绿色值
uint32_t MAX30105::getGreen(void)
{
    // 检查传感器是否有新的数据，等待250ms
    if (safeCheck(250))
        return (sense.green[sense.head]);
    else
        return (0); // 传感器未找到新数据
}

// 报告FIFO中的下一个红色值
uint32_t MAX30105::getFIFORed(void)
{
    return (sense.red[sense.tail]);
}

// 报告FIFO中的下一个红外值
uint32_t MAX30105::getFIFOIR(void)
{
    return (sense.IR[sense.tail]);
}

// 报告FIFO中的下一个绿色值
uint32_t MAX30105::getFIFOGreen(void)
{
    return (sense.green[sense.tail]);
}

// 前进尾部指针
void MAX30105::nextSample(void)
{
    if (available()) // 仅在有新数据时前进尾部指针
    {
        sense.tail++;
        sense.tail %= STORAGE_SIZE; // 环绕条件
    }
}

// 轮询传感器以获取新数据
// 定期调用
// 如果有新数据可用，它会更新主结构中的头部和尾部
// 返回获取的新样本数
uint16_t MAX30105::check(void)
{
    uint8_t readPointer = getReadPointer();
    uint8_t writePointer = getWritePointer();

    int numberOfSamples = 0;

    // 检查是否有新数据
    if (readPointer != writePointer)
    {
        // 计算需要从传感器获取的读数数目
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0)
        {
            numberOfSamples += 32; // 环绕情况
        }

        // 根据需要读取的样本数量计算字节数
        int bytesLeftToRead = numberOfSamples * activeLEDs * 3;
        uint8_t fifo_data_reg = MAX30105_FIFODATA;

        while (bytesLeftToRead > 0)
        {
            int toGet = bytesLeftToRead;
            if (toGet > I2C_BUFFER_LENGTH)
            {
                toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); // 调整为样本的整数倍
            }

            bytesLeftToRead -= toGet;

            // 发送 FIFO 数据寄存器地址
            int result = i2c_write_blocking(_i2cPort, MAX30105_ADDRESS, &fifo_data_reg, 1, true);
            if (result < 0)
            {
                DEBUG_PRINT("I2C write error: %d\n", result);
                return 0;
            }

            // 读取数据到缓冲区
            uint8_t data[toGet];
            result = i2c_read_blocking(_i2cPort, MAX30105_ADDRESS, data, toGet, false);
            if (result < 0)
            {
                DEBUG_PRINT("I2C read error: %d\n", result);
                return 0;
            }

            int index = 0;
            while (toGet > 0)
            {
                sense.head++;               // 更新存储结构的头部指针
                sense.head %= STORAGE_SIZE; // 环绕条件

                uint32_t tempLong = 0;

                // 读取红光数据（3字节）
                tempLong = ((uint32_t)data[index] << 16) | ((uint32_t)data[index + 1] << 8) | data[index + 2];
                tempLong &= 0x3FFFF; // 保留 18 位数据
                sense.red[sense.head] = tempLong;
                index += 3;
                toGet -= 3;

                if (activeLEDs > 1)
                {
                    // 读取红外数据（3字节）
                    tempLong = ((uint32_t)data[index] << 16) | ((uint32_t)data[index + 1] << 8) | data[index + 2];
                    tempLong &= 0x3FFFF;
                    sense.IR[sense.head] = tempLong;
                    index += 3;
                    toGet -= 3;
                }

                if (activeLEDs > 2)
                {
                    // 读取绿光数据（3字节）
                    tempLong = ((uint32_t)data[index] << 16) | ((uint32_t)data[index + 1] << 8) | data[index + 2];
                    tempLong &= 0x3FFFF;
                    sense.green[sense.head] = tempLong;
                    index += 3;
                    toGet -= 3;
                }
            }
        }
    }

    return numberOfSamples; // 返回新数据的样本数量
}

// 检查新数据，但在一定时间后放弃
// 如果找到新数据，则返回true
// 如果未找到新数据，则返回false
bool MAX30105::safeCheck(uint8_t maxTimeToCheck)
{
    absolute_time_t markTime = get_absolute_time();

    while (1)
    {
        // 将 maxTimeToCheck 从毫秒转换为微秒进行比较
        if (absolute_time_diff_us(markTime, get_absolute_time()) > maxTimeToCheck * 1000)
        {
            return (false);
        }

        if (check() == true) // 找到新数据
        {
            return (true);
        }

        sleep_ms(1); // 暂停 1 毫秒
    }
}

// 给定一个寄存器，读取它，掩码它，然后设置该值
void MAX30105::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
    // 获取当前寄存器内容
    uint8_t originalContents = readRegister8(_i2caddr, reg);

    // 清零我们感兴趣的寄存器部分
    originalContents = originalContents & mask;

    // 更改内容
    writeRegister8(_i2caddr, reg, originalContents | thing);
}

// writeRegister8 函数
void MAX30105::writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(_i2cPort, address, buffer, 2, false);
}

// readRegister8 函数
uint8_t MAX30105::readRegister8(uint8_t address, uint8_t reg)
{
    uint8_t value = 0;
    // 发送寄存器地址
    i2c_write_blocking(_i2cPort, address, &reg, 1, true);
    // 读取数据
    i2c_read_blocking(_i2cPort, address, &value, 1, false);
    return value;
}
