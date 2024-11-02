#ifndef __MP3_H
#define __MP3_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

// 定义 UART 实例和引脚
#define MP3_UART_ID uart1
#define MP3_UART_BAUDRATE 9600
#define MP3_UART_TX_PIN 8 // 根据您的硬件连接进行修改
#define MP3_UART_RX_PIN 9 // 根据您的硬件连接进行修改

// 定义其他宏和常量
#define MP3_EQ_NORMAL 0
#define MP3_EQ_POP 1
#define MP3_EQ_ROCK 2
#define MP3_EQ_JAZZ 3
#define MP3_EQ_CLASSIC 4
#define MP3_EQ_BASS 5

#define MP3_LOOP_ALL 0
#define MP3_LOOP_FOLDER 1
#define MP3_LOOP_ONE 2
#define MP3_LOOP_RAM 3

#define MP3_STATUS_STOPPED 0
#define MP3_STATUS_PLAYING 1
#define MP3_STATUS_PAUSED 2
#define MP3_STATUS_FF 3
#define MP3_STATUS_FR 4

#define MP3_STATUS_CHECKS_IN_AGREEMENT 4

class Mp3
{
private:
    unsigned int count = 0;
    unsigned int currentFileNumber = 1;

    unsigned int sendCommandWithUnsignedIntResponse(uint8_t command);
    bool waitUntilAvailable(unsigned long timeout = 1000);
    void sendCommand(uint8_t command, uint8_t arg1, uint8_t arg2, char *responseBuffer, unsigned int bufferLength);
    void sendCommand(uint8_t command);
    void sendCommand(uint8_t command, uint8_t arg1);
    void sendCommand(uint8_t command, uint8_t arg1, uint8_t arg2);

public:
    void init();
    void play();
    void restart();
    void pause();
    void next();
    void prev();
    void fastReverse();
    void fastForward();
    void playFileByIndexNumber(unsigned int fileNumber);
    void volumeUp();
    void volumeDn();
    void setVolume(uint8_t volumeFrom0To30);
    void setEqualizer(uint8_t equalizerMode);
    void setLoopMode(uint8_t loopMode);
    void playFileNumber(uint8_t file, uint8_t fileNumber);

    uint8_t getStatus();
    uint8_t getVolume();
    uint8_t getEqualizer();
    uint8_t getLoopMode();
    unsigned int countFiles();
    unsigned int currentFileIndexNumber();

    void randomAll();
};

extern Mp3 mp3;

#endif // __MP3_H
