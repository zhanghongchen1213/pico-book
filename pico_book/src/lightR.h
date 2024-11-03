#ifndef __LIGHTR_H
#define __LIGHTR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

#define ADC_PIN 26

class LightR
{
public:
    void init();
    uint16_t read();
};

extern LightR adc;
#endif