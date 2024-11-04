#ifndef __IRQ_H
#define __IRQ_H

#include "data_global.h"
#include "pico/stdlib.h"
#include "debug.h"
#include "led.h"

#define TOUCHA_PIN 16
#define TOUCHB_PIN 17
#define HEARTBEAT_INT_PIN 6

void pico_irq_init(void);

void gpio_interrupt_handler(uint gpio, uint32_t events);

#endif