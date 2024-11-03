#ifndef __LED_H
#define __LED_H

#include "pico/stdlib.h"

#define LEDA_PIN 18
#define LEDB_PIN 19
#define LEDC_PIN 20
#define LEDD_PIN 21
#define LED_ALONE_PIN 15

class LED
{
public:
    void led_init();
    void led_on();
    void led_off();
    void led_toggle();
    void led_alone_on();
    void led_alone_off();
    void led_alone_toggle();
};

extern LED led_group;

#endif // __LED_H