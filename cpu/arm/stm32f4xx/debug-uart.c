#include <contiki-conf.h>
#include "contiki.h"
#include <stm32f4xx.h>
#include <core_cm4.h>
#include <debug-uart.h>
#include <string.h>

#define APB_CLOCK (MCK / 8)
#define BAUDRATE 460800

void
dbg_setup_uart()
{
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	RCC->APB1ENR = (1 << 17); /* enable clock for USART2 */
	RCC->AHB1ENR = (1 << 3);  /* enable clock for GPIOD */
	GPIOD->MODER |= (2 << 10); 
	GPIOD->MODER |= (2 << 12);
	GPIOD->OTYPER &= ~(3 << 10);
	GPIOD->OTYPER &= ~(3 << 12);
	GPIOD->OSPEEDR |= (2 << 10);
	GPIOD->OSPEEDR |= (2 << 12);
	GPIOD->PUPDR &= ~(3 << 10);
	GPIOD->PUPDR &= ~(3 << 12);
	GPIOD->AFR[0] |= (7 << 20);
	GPIOD->AFR[0] |= (7 << 24);
	USART2->CR1 |= (1 << 13);

	integerdivider = ((25 * APB_CLOCK) / (2 * BAUDRATE));
	tmpreg = (integerdivider / 100) << 4;
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
	tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	USART2->BRR = (uint16_t)tmpreg;
	USART2->CR1 |= (3 << 2);
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	int i = 0;

	while (i < len)
	{
		dbg_putchar(seq[i++]);
	} 

	return i;
}

void
dbg_putchar(char ch)
{
	while (((USART2->SR >> 6) & 0x01) == 0);
	USART2->DR = (uint8_t)ch;
}

void
dbg_blocking_putchar(const char ch)
{
	dbg_putchar(ch);
}

