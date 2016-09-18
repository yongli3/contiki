#include <stm32f10x.h>
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

#include "contiki.h"
#include "contiki-net.h"
#include "sys/autostart.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "net/netstack.h"
#include "net/ip/uip.h"
#include "net/mac/frame802154.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /*NETSTACK_CONF_WITH_IPV6*/

#if NETSTACK_CONF_WITH_IPV6
PROCINIT(&etimer_process, &tcpip_process);
#else /*NETSTACK_CONF_WITH_IPV6*/
PROCINIT(&etimer_process);
#warning "No TCP/IP process!"
#endif /*NETSTACK_CONF_WITH_IPV6*/

extern int (*uart1_input_handler)(unsigned char c);

extern uint8_t _data[];
extern uint8_t _etext[];
extern uint8_t _edata[];
extern uint8_t __bss_start[];
extern uint8_t __bss_end[];

unsigned int idle_count = 0;

uint8_t sys_seed;
uint8_t mac_longaddr[8] = { 0x80, 0x03, 0x00, 0x00, 0, 0, 0, 0xbb };
uint16_t mac_shortaddr;
int
main()
{
    linkaddr_t linkaddr;
    int i;
    
 dbg_setup_uart(115200);
 printf("**Initialising ...%d etext=%x\r\n", CLOCK_CONF_SECOND, _etext);

 printf("data[%x-%x] bss[%x-%x]\r\n", _data, _edata, __bss_start, __bss_end);
  
  clock_init();
  
  rtimer_init();
  
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();
  
  //printf("Initialising ...\r\n");

  uart1_input_handler = serial_line_input_byte;
  serial_line_init();

  queuebuf_init();

  mac_shortaddr = (mac_longaddr[0] << 8) + mac_longaddr[1]; 

    random_init(0);
  
  // set the random seed for MAC_addr[7]
  NETSTACK_RADIO.init();
  for (i = 0; i < 8; i++)
      linkaddr.u8[i] = mac_longaddr[i];
  
  memcpy(&uip_lladdr.addr, &linkaddr.u8, sizeof(linkaddr_t));
  linkaddr_set_node_addr(&linkaddr);
  
  // set short/long address using MAC address
  { 
      printf("short ADDR:%x ", mac_shortaddr);
      printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
             mac_longaddr[7], mac_longaddr[6], mac_longaddr[5], mac_longaddr[4],
             mac_longaddr[3], mac_longaddr[2], mac_longaddr[1], mac_longaddr[0]);
  
      //cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
    }
  
  
  NETSTACK_RDC.init();
  NETSTACK_LLSEC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();
  
  //printf(CONTIKI_VERSION_STRING "TCP started.. RSSI=%d\n", cc2520_rssi());
  
  process_start(&tcpip_process, NULL);

  
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


void
uip_log(char *m)
{
  printf("%s\n", m);
}

