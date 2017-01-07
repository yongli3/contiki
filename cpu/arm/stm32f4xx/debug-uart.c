#include <contiki-conf.h>
#include "contiki.h"
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <core_cm4.h>
#include <debug-uart.h>
#include <string.h>

#define APB_CLOCK (MCK / 8)
#define BAUDRATE 115200

#if 0
size_t strlen(const char *s)
{
    char *p; 
    for (p=s ; *p ; p++);
    return p - s;
}
#endif

static void delay()
{
  unsigned int i, j;
  for (i = 0; i < 99999; i++)
    for (j = 0; j < 99999; j++)
    {}

}

static void LED_Init(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_14);

    //while (1) 
   {
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
        delay();
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
        delay();
    }
} 

#if 0
void
dbg_setup_uart()
{
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

    LED_Init();

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
#else

static void COM1Init(u32 BaudRate)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	USART_InitStructure.USART_BaudRate = BaudRate;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
  	USART_Init(USART1, &USART_InitStructure);
  	USART_Cmd(USART1, ENABLE);
  	USART_ClearFlag(USART1, USART_FLAG_TC);

    dbg_putchar('O');
    dbg_putchar('K');
    dbg_putchar('\n');
}

void
dbg_setup_uart()
{
    COM1Init(115200);
    LED_Init();
}
#endif
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
#if 0
	while (((USART2->SR >> 6) & 0x01) == 0);
	USART2->DR = (uint8_t)ch;
#else
    USART_SendData(USART1, (u8) ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
#endif    
}

void
dbg_blocking_putchar(const char ch)
{
	dbg_putchar(ch);
}

