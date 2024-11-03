#include "lightR.h"

LightR adc;

void LightR::init()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
}

uint16_t LightR::read()
{
    adc_select_input(0);
    return adc_read();
}