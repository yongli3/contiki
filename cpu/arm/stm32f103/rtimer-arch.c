#include "sys/energest.h"
#include "sys/rtimer.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// Used for udelay
static void TIM3_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = 0;
    unsigned int counter_clock = 1 * 1000 * 1000;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // counter clock(CK_CNT) = TIMxCLK(Fck_psc) / [PSC + 1]
    // TIM2CLK = PCK1 X 2
    // PSC(Prescaler) = (TIM2CLK / counter_clock) - 1
	PrescalerValue = (uint16_t) ((SystemCoreClock) / counter_clock) - 1;
    printf("%s sysclk=%d prescal=%d\n", __func__, SystemCoreClock, PrescalerValue);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, DISABLE);
}

// delay us 5us -> 5.2us;1us -> 2us
void clock_delay_usec(uint16_t t)
{
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);

    while (TIM3->CNT < (t-1) * 1);
    TIM_Cmd(TIM3, DISABLE);
}

void rtimer_arch_init(void) 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue = 0;

	// Compute the prescaler value
	// TIM2 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // counter clock(CK_CNT) = TIMxCLK(Fck_psc) / [PSC + 1]
    // TIM2CLK = PCK1 X 2
    // PSC(Prescaler) = (TIM2CLK / counter_clock) - 1
	PrescalerValue = (uint16_t) ((SystemCoreClock) / RTIMER_ARCH_SECOND) - 1;
    printf("%s sysclk=%d prescal=%d\n", __func__, SystemCoreClock, PrescalerValue);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Prescaler configuration ->PSC
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	// Output Compare Timing Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = RTIMER_ARCH_SECOND; // ->CCR1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// Interrupt generation
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);

	// NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    TIM3_init();
    
	PRINTF("rtimer_arch_init done\r\n");
}

rtimer_clock_t rtimer_arch_now(void) 
{
	return (rtimer_clock_t)TIM2->CNT;
}

void rtimer_arch_schedule(rtimer_clock_t t) 
{
    TIM_SetCompare1(TIM2, (uint16_t)t);
}

void TIM2_handler(void) __attribute__ ((interrupt));

// irq every 1 second
void TIM2_handler(void) 
{
    unsigned short capture;
    
    //printf("SR=%x DIER=%x CNT=%d CCR1=%d\n", TIM2->SR, TIM2->DIER, TIM2->CNT, TIM2->CCR1);
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) 
    {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

        TIM_SetCounter(TIM2, 0);
        //capture = TIM_GetCapture1(TIM2);
        //TIM_SetCompare1(TIM2, capture + RTIMER_ARCH_SECOND);
    
        // LED blink
        //printf("CNT=%d CCR1=%d\n", TIM2->CNT, TIM2->CCR1);
#if 0
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        else
            GPIO_SetBits(GPIOA,GPIO_Pin_8);
 #endif           
        ENERGEST_ON(ENERGEST_TYPE_IRQ);
        rtimer_run_next();
        ENERGEST_OFF(ENERGEST_TYPE_IRQ);
	}
}
