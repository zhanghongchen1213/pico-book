#include "led.h"

LED led_group;

void LED::led_init()
{
    gpio_init(LEDA_PIN);
    gpio_init(LEDB_PIN);
    gpio_init(LEDC_PIN);
    gpio_init(LEDD_PIN);
    gpio_init(LED_ALONE_PIN);
    gpio_set_dir(LEDA_PIN, GPIO_OUT);
    gpio_set_dir(LEDB_PIN, GPIO_OUT);
    gpio_set_dir(LEDC_PIN, GPIO_OUT);
    gpio_set_dir(LEDD_PIN, GPIO_OUT);
    gpio_set_dir(LED_ALONE_PIN, GPIO_OUT);
}

void LED::led_on()
{
    gpio_put(LEDA_PIN, 1);
    gpio_put(LEDB_PIN, 1);
    gpio_put(LEDC_PIN, 1);
    gpio_put(LEDD_PIN, 1);
}

void LED::led_off()
{
    gpio_put(LEDA_PIN, 0);
    gpio_put(LEDB_PIN, 0);
    gpio_put(LEDC_PIN, 0);
    gpio_put(LEDD_PIN, 0);
}

void LED::led_toggle()
{
    gpio_put(LEDA_PIN, !gpio_get(LEDA_PIN));
    gpio_put(LEDB_PIN, !gpio_get(LEDB_PIN));
    gpio_put(LEDC_PIN, !gpio_get(LEDC_PIN));
    gpio_put(LEDD_PIN, !gpio_get(LEDD_PIN));
}

void LED::led_alone_on()
{
    gpio_put(LED_ALONE_PIN, 1);
}

void LED::led_alone_off()
{
    gpio_put(LED_ALONE_PIN, 0);
}

void LED::led_alone_toggle()
{
    gpio_put(LED_ALONE_PIN, !gpio_get(LED_ALONE_PIN));
}