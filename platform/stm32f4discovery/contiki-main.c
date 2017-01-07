#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include <dev/leds.h>
#include <debug-uart.h>
#include <packet_drv.h>

#include <net/netstack.h>
#include <net/uip.h>
#include <net/uip_arp.h>

#include <main.h>

#include <button-sensor.h>

unsigned int idle_count = 0;

int
main()
{
  dbg_setup_uart();
  printf("Initialising\n");
  
  clock_init();
  printf("Clock initialized\n");

  leds_init();
  printf("Leds initialized\n");

  process_init();
  process_start(&etimer_process, NULL);
	process_start(&sensors_process, NULL);
  process_start(&stm32f4discovery_packet_drv, NULL);

	{
		uip_ipaddr_t hostaddr, netmask, gwaddr;
		uip_eth_addr ethaddr;

		process_start(&tcpip_process, NULL);

		uip_init();
		uip_arp_init();
		ethaddr.addr[0] = MAC_ADDR0;
		ethaddr.addr[1] = MAC_ADDR1;
		ethaddr.addr[2] = MAC_ADDR2;
		ethaddr.addr[3] = MAC_ADDR3;
		ethaddr.addr[4] = MAC_ADDR4;
		ethaddr.addr[5] = MAC_ADDR5;
		uip_setethaddr(ethaddr);
		uip_ipaddr(&hostaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
		uip_ipaddr(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, \
										NETMASK_ADDR2, NETMASK_ADDR3);
		uip_ipaddr(&gwaddr, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		uip_sethostaddr(&hostaddr);
		uip_setnetmask(&netmask);
		uip_setdraddr(&gwaddr);
	}


  autostart_start(autostart_processes);
  printf("Processes running\n");
  while(1) {
    do {
    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
     asm("wfi"::);
  }
  return 0;
}

