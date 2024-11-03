#ifndef __WS2812_H
#define __WS2812_H

#include "pico/stdlib.h"

#include "hardware/dma.h"

#include "ws2812.pio.h"

#include "debug.h"

#include "data_global.h"

#define WS2812_PIN 22

void init_pio();

void ws2812_send_data();

#endif
