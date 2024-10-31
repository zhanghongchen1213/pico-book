#include "Mp3.h"
#include <cstring>

Mp3 mp3;
// UART 接口函数的封装
static void mp3_uart_write(const uint8_t *data, size_t length)
{
    uart_write_blocking(MP3_UART_ID, data, length);
}

static int mp3_uart_read(uint8_t *buffer, size_t length, unsigned long timeout_ms)
{
    size_t received = 0;
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);
    while (received < length && absolute_time_diff_us(get_absolute_time(), timeout) > 0)
    {
        int c = uart_getc(MP3_UART_ID);
        if (c != PICO_ERROR_TIMEOUT)
        {
            buffer[received++] = (uint8_t)c;
        }
    }
    return received;
}

unsigned int Mp3::sendCommandWithUnsignedIntResponse(uint8_t command)
{
    char buffer[6];
    memset(buffer, 0, sizeof(buffer));
    sendCommand(command, 0, 0, buffer, sizeof(buffer));

#if MP3_DEBUG
    printf("Received buffer: ");
    for (int i = 0; i < 6; i++)
    {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
#endif

    if ((uint8_t)buffer[0] == 0xAA && (uint8_t)buffer[2] == command)
    {
        unsigned int value = ((uint8_t)buffer[3] << 8) | (uint8_t)buffer[4];
#if MP3_DEBUG
        printf("Received value: %u\n", value);
#endif
        return value;
    }
    else
    {
#if MP3_DEBUG
        printf("Invalid response format\n");
#endif
        return 0;
    }
}

bool Mp3::waitUntilAvailable(unsigned long timeout)
{
    absolute_time_t end_time = make_timeout_time_ms(timeout);
    while (absolute_time_diff_us(end_time, get_absolute_time()) > 0)
    {
        if (uart_is_readable(MP3_UART_ID))
        {
            return true; // 数据可用，返回 true
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 延时 1 毫秒，让出 CPU
    }
    return false; // 超时，返回 false
}

void Mp3::sendCommand(uint8_t command, uint8_t arg1, uint8_t arg2, char *responseBuffer, unsigned int bufferLength)
{
    uint8_t args = (arg1 != 0) ? 1 : 0;
    if ((command == 0x41) || (command == 0x42))
    {
        args = 2;
    }

#if MP3_DEBUG
    printf("\nSending command: 7E %02X %02X", 2 + args, command);
    if (args >= 1)
        printf(" %02X", arg1);
    if (args >= 2)
        printf(" %02X", arg2);
    printf(" EF\n");
#endif

    uint8_t cmdBuffer[6];
    size_t cmdLength = 0;
    cmdBuffer[cmdLength++] = 0x7E;
    cmdBuffer[cmdLength++] = 2 + args;
    cmdBuffer[cmdLength++] = command;
    if (args >= 1)
        cmdBuffer[cmdLength++] = arg1;
    if (args == 2)
        cmdBuffer[cmdLength++] = arg2;
    cmdBuffer[cmdLength++] = 0xEF;

    mp3_uart_write(cmdBuffer, cmdLength);
    sleep_ms(200);
    /*
            if (responseBuffer && bufferLength)
            {
                memset(responseBuffer, 0, bufferLength);
            }

            waitUntilAvailable(1000);

        #if MP3_DEBUG
            printf("Received response: ");
        #endif

            unsigned int i = 0;
            uint8_t byte = 0;
            while (waitUntilAvailable(150))
            {
                byte = uart_getc(MP3_UART_ID);

        #if MP3_DEBUG
                printf("%02X ", byte);
        #endif

                if (responseBuffer && (i < bufferLength - 1))
                {
                    responseBuffer[i++] = byte;
                }
            }

        #if MP3_DEBUG
            printf("\n");
        #endif

        */
}

void Mp3::sendCommand(uint8_t command)
{
    this->sendCommand(command, 0, 0, 0, 0);
}

void Mp3::sendCommand(uint8_t command, uint8_t arg1)
{
    this->sendCommand(command, arg1, 0, 0, 0);
}

void Mp3::sendCommand(uint8_t command, uint8_t arg1, uint8_t arg2)
{
    this->sendCommand(command, arg1, arg2, 0, 0);
}

void Mp3::init()
{
    // 初始化 UART
    uart_init(MP3_UART_ID, MP3_UART_BAUDRATE);
    uart_set_format(MP3_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(MP3_UART_ID, false);

    gpio_set_function(MP3_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MP3_UART_RX_PIN, GPIO_FUNC_UART);

    sleep_ms(2000);

    setVolume(30);
    uint8_t volume = getVolume();
#if MP3_DEBUG
    printf("Volume: %u\n", volume);
#endif
    setLoopMode(4); // 单曲播放
    count = countFiles();
#if MP3_DEBUG
    printf("MP3 file count: %u\n", count);
#endif
}

void Mp3::play()
{
    sendCommand(0x01);
}

void Mp3::restart()
{
    uint8_t oldVolume = getVolume();
    setVolume(0);
    next();
    pause();
    setVolume(oldVolume);
    prev();
}

void Mp3::pause()
{
    sendCommand(0x02);
}

void Mp3::next()
{
    sendCommand(0x03);
}

void Mp3::prev()
{
    sendCommand(0x04);
}

void Mp3::fastReverse()
{
    sendCommand(0x0B);
}

void Mp3::fastForward()
{
    sendCommand(0x0A);
}

void Mp3::playFileByIndexNumber(unsigned int fileNumber)
{
    sendCommand(0x41, (fileNumber >> 8) & 0xFF, fileNumber & 0xFF);
}

void Mp3::volumeUp()
{
    sendCommand(0x05);
}

void Mp3::volumeDn()
{
    sendCommand(0x06);
}

void Mp3::setVolume(uint8_t volumeFrom0To30)
{
    sendCommand(0x31, volumeFrom0To30);
}

void Mp3::setEqualizer(uint8_t equalizerMode)
{
    sendCommand(0x32, equalizerMode);
}

void Mp3::setLoopMode(uint8_t loopMode)
{
    sendCommand(0x33, loopMode);
}

void Mp3::playFileNumber(uint8_t file, uint8_t fileNumber)
{
    sendCommand(0x42, file, fileNumber);
}

uint8_t Mp3::getStatus()
{
    uint8_t statTotal = 0;
    uint8_t stat = 0;
    do
    {
        statTotal = 0;
        for (uint8_t x = 0; x < MP3_STATUS_CHECKS_IN_AGREEMENT; x++)
        {
            stat = this->sendCommandWithUnsignedIntResponse(0x42);
            if (stat == 0)
                return 0;
            statTotal += stat;
        }

    } while (statTotal != 1 * MP3_STATUS_CHECKS_IN_AGREEMENT && statTotal != 2 * MP3_STATUS_CHECKS_IN_AGREEMENT);

    return statTotal / MP3_STATUS_CHECKS_IN_AGREEMENT;
}

uint8_t Mp3::getVolume()
{
    return sendCommandWithUnsignedIntResponse(0x11);
}

uint8_t Mp3::getEqualizer()
{
    return sendCommandWithUnsignedIntResponse(0x44);
}

uint8_t Mp3::getLoopMode()
{
    return sendCommandWithUnsignedIntResponse(0x13);
}

unsigned int Mp3::countFiles()
{
    return sendCommandWithUnsignedIntResponse(0x14);
}

unsigned int Mp3::currentFileIndexNumber()
{
    return sendCommandWithUnsignedIntResponse(0x1A);
}

void Mp3::randomAll()
{
    const unsigned int minFileNumber = 1;
    const unsigned int maxFileNumber = 80;

    // 播放当前文件
    playFileNumber(3, currentFileNumber);

    // 更新文件编号，循环回到起始值
    currentFileNumber++;
    if (currentFileNumber > maxFileNumber)
    {
        currentFileNumber = minFileNumber;
    }
}
