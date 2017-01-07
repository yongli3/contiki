#include <dev/leds.h>
#include <stm32f4xx.h>
#include <core_cm4.h>

#define LEDS_CONF_GREEN 12
#define LEDS_CONF_YELLOW 13
#define LEDS_CONF_RED 14
#define LEDS_CONF_BLUE 15

void init_led(uint16_t led)
{
    /* output */
    GPIOD->MODER &= ~(3 << (2 * led));
    GPIOD->MODER |= (1 << (2 * led));
    
    /* push-pull */
    GPIOD->OTYPER &= ~(1 << led);

    /* 50 MHz */
    GPIOD->OSPEEDR &= ~(3 << (2 * led));
    GPIOD->OSPEEDR |= (3 << (2 * led));

    /* no pull */
    GPIOD->PUPDR &= ~(3 << (2 * led));
}

void leds_arch_init(void)
{
    /* initialize GPIOD peripherial clock */
    RCC->AHB1ENR |= (1 << 3);

    /* initialize GPIOD */
    init_led(LEDS_CONF_GREEN);
    init_led(LEDS_CONF_YELLOW);
    init_led(LEDS_CONF_RED);
    init_led(LEDS_CONF_BLUE);
}

unsigned char leds_arch_get(void)
{
    return
        (GPIOD->ODR & (1 << LEDS_CONF_GREEN) ? LEDS_GREEN : 0) |
        (GPIOD->ODR & (1 << LEDS_CONF_RED) ? LEDS_RED : 0) |
        (GPIOD->ODR & (1 << LEDS_CONF_BLUE) ? LEDS_BLUE : 0) |
        (GPIOD->ODR & (1 << LEDS_CONF_YELLOW) ? LEDS_YELLOW : 0);
}

void leds_arch_set(unsigned char leds)
{
    leds & LEDS_GREEN ? led_on(LEDS_CONF_GREEN) : led_off(LEDS_CONF_GREEN);
    leds & LEDS_RED ? led_on(LEDS_CONF_RED) : led_off(LEDS_CONF_RED);
    leds & LEDS_BLUE ? led_on(LEDS_CONF_BLUE) : led_off(LEDS_CONF_BLUE);
    leds & LEDS_YELLOW ? led_on(LEDS_CONF_YELLOW) :
        led_off(LEDS_CONF_YELLOW);
}

void led_on(unsigned char led)
{
    GPIOD->ODR |= (1 << led);
}

void led_off(unsigned char led)
{
    GPIOD->ODR &= ~(1 << led);
}

