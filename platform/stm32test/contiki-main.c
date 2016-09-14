#include <stm32f10x_map.h>
#include <stm32f10x_dma.h>
#include <gpio.h>
#include <nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <debug-uart.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>

unsigned int idle_count = 0;

int
main()
{
 dbg_setup_uart(115200);
 printf("**Initialising ...\r\n");

 printf("Initialising ...\r\n");
  
  clock_init();
  
  process_init();
printf("Initialising ...\r\n");

  process_start(&etimer_process, NULL);
  //printf("Initialising ...\r\n");
  autostart_start(autostart_processes);
  //printf("Processes running\r\n");
  while(1) {
    //printf(" ... %d\r\n", idle_count);
    do {
    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
  return 0;
}




