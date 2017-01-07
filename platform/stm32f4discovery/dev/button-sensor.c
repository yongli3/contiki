#include <contiki.h>
#include <stm32f4xx.h>
#include <core_cm4.h>
#include <debug-uart.h>
#include <sys/ctimer.h>
#include <button-sensor.h>

#include <stdio.h>

#define LONG_PRESS_TIME (CLOCK_CONF_SECOND * 5)

/* time after which sensor will reset it's state to BUTTON_RELEASED */
#define SENSOR_RESET_TIME (CLOCK_CONF_SECOND / 10)

void
EXTI0_IRQHandler(void) __attribute__ ((interrupt));

void long_press_callback(void *);

static int8_t button_state = BUTTON_RELEASED; 
static struct ctimer button_ctimer;
static struct ctimer sensor_reset_ctimer;

static int
button_value(int type)
{
	return button_state;
}

static int
button_status(int type)
{
	switch (type)
	{
		case SENSORS_ACTIVE:
		case SENSORS_READY:
			return NVIC->ISER[0] & (1 << 6);
	}

	return 0;
}

static int
button_sensor_config(int type, int value)
{
	switch (type)
	{
		case SENSORS_HW_INIT:
			/* pushbutton is at GPIOA0 on stm32f4-discovery */
			/* initialize GPIOA peripheral clock */
			RCC->AHB1ENR |= (1 << 0);

			/* input, floating */
			GPIOA->MODER &= ~(3 << 0 * 2);
			GPIOA->PUPDR &= ~(3 << 0 * 2);

			/* raising and falling trigger on EXTI0 */
			EXTI->RTSR |= (1 << 0);
			EXTI->FTSR |= (1 << 0);

			/* unmask interrupt on EXTI0 */
			EXTI->IMR |=  (1 << 0);

			/* set NVIC EXTI0 to GPIOA0 */
			SYSCFG->EXTICR[0] &= ~(15 << 0);

			/* enable EXTI0 IRQ (6) */
			NVIC->ISER[0] |= (1 << 6);

			ctimer_init();

			return 1;
		case SENSORS_ACTIVE:
			if (value)
			{
				if (!button_status(SENSORS_ACTIVE))
				{
					NVIC->ISER[0] |= (1 << 6);

					ctimer_stop(&button_ctimer);
					ctimer_stop(&sensor_reset_ctimer);
				}
			}
			else
			{
				/* disable EXTI0 IRQ (6) */
				NVIC->ISER[0] &= ~(1 << 6);

				ctimer_stop(&button_ctimer);
				ctimer_stop(&sensor_reset_ctimer);
			}

			return 1;
	}

	return 0;
}

void sensor_reset_callback(void *data)
{
	button_state = BUTTON_RELEASED;
	ctimer_stop(&sensor_reset_ctimer);
}

void long_press_callback(void *data)
{
	button_state = BUTTON_LONG_PRESS;
	sensors_changed(&button_sensor);
	ctimer_set(&sensor_reset_ctimer, SENSOR_RESET_TIME, sensor_reset_callback, NULL);
}

void
EXTI0_IRQHandler(void)
{
	if (!(GPIOA->IDR & (1 << 0)))
	{
		if (!ctimer_expired(&button_ctimer))
		{
			ctimer_stop(&button_ctimer);
			button_state = BUTTON_SHORT_PRESS;
			sensors_changed(&button_sensor);
			ctimer_set(&sensor_reset_ctimer, SENSOR_RESET_TIME, sensor_reset_callback, NULL);
		}
	}
	else
	{
		ctimer_set(&button_ctimer, LONG_PRESS_TIME, long_press_callback, NULL);
	}

	/* clear pending interrupt */
	EXTI->PR |= (1 << 0);
}

SENSORS_SENSOR(button_sensor, "Pushbutton", button_value, button_sensor_config, button_status);
