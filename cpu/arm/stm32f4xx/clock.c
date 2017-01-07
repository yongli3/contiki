#include <contiki-conf.h>
#include <stm32f4xx_conf.h>
#include <core_cm4.h>
#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>
#include <debug-uart.h>

#define TICKS_PER_INT (MCK / 8 / CLOCK_CONF_SECOND)

static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_CONF_SECOND;

void
SysTick_Handler(void) __attribute__ ((interrupt));

void
SysTick_Handler(void)
{
  current_clock++;

  if(etimer_pending() && etimer_next_expiration_time() <= current_clock)
  {
    etimer_request_poll();
  }
  if (--second_countdown == 0)
  {
    current_seconds++;
    second_countdown = CLOCK_CONF_SECOND;
  }
}

void
clock_init()
{
    SysTick_Config(TICKS_PER_INT);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    NVIC_SetPriority(SysTick_IRQn, 6);
}

clock_time_t
clock_time(void)
{
  return current_clock;
}

/* The inner loop takes 4 cycles. The outer 5+SPIN_COUNT*4. */

#define SPIN_TIME 2 /* us */
#define SPIN_COUNT (((MCK*SPIN_TIME/1000000)-5)/4)

#ifndef __MAKING_DEPS__

void
clock_delay(unsigned int t)
{
#ifdef __THUMBEL__ 
  asm volatile("1: mov r1,%2\n2:\tsub r1,#1\n\tbne 2b\n\tsub %0,#1\n\tbne 1b\n":"=l"(t):"0"(t),"l"(SPIN_COUNT));
#else
#error Must be compiled in thumb mode
#endif
}
#endif

unsigned long
clock_seconds(void)
{
  return current_seconds;
}
