#include <stm32f10x.h>
#include "stm32f10x_it.h"
#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>

// Use systick to get the startup time
static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_SECOND;
uint32_t SystemCoreClock = MCK;

void
SysTick_handler(void) __attribute__ ((interrupt));

// IRQ=CLOCK_SECOND Hz
void
SysTick_handler(void)
{
    current_clock++;
    if (etimer_pending() && etimer_next_expiration_time() <= current_clock) 
    {
        etimer_request_poll();
    }
    
    if (--second_countdown == 0) 
    {
        current_seconds++;
        second_countdown = CLOCK_SECOND;
    }
}

// delay ms 1ms -> 1ms
void mdelay(uint32_t ms) {
  uint32_t curTicks = current_clock;

  while ((current_clock - curTicks) < ms);
}

void clock_init() 
{
    if (SysTick_Config(SystemCoreClock / CLOCK_SECOND)) 
    {
        while (1){};
    }
}

// system uptime ms 
clock_time_t clock_time(void)
{
    return current_clock;
}

// system uptime seconds
unsigned long clock_seconds(void)
{
    return current_seconds;
}

void watchdog_periodic(void)
{}

