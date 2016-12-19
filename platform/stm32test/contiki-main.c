#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
    
#include "contiki-conf.h"
#include "contiki-net.h"
#include "core/dev/serial-line.h"
#include "dev/cc2520/cc2520.h"
#include "net/ipv6/uip-ds6.h"
    
#include "apps/serial-shell/serial-shell.h"
    
#include "stm32f10x.h"
#include "contiki.h"
#include "sys/autostart.h"
    
#include "uart-debug.h"

PROCESS_NAME(udp_client_process);
PROCESS_NAME(about_process);
///PROCESS_NAME(dhcp_process);

extern uint8_t _data[];
extern uint8_t _etext[];
extern uint8_t _edata[];
extern uint8_t __bss_start[];
extern uint8_t __bss_end[];
    
extern int (*uart1_input_handler)(unsigned char c);
    
static void platform_init();

uint8_t sys_seed;
uint8_t mac_longaddr[8] = { 0x80, 0x03, 0x00, 0x00, 0, 0, 0, 0xbb };
uint16_t mac_shortaddr;
static unsigned char buffer[140];
int main()
{
    linkaddr_t linkaddr;
    int i;
    int buffer_len;
    int ret;
    unsigned char ch;
    uip_ip4addr_t ipv4addr, netmask;
    unsigned short crc16;

    platform_init();
    //while (1);
    printf("%s-%s Hello Contiki\n", __DATE__, __TIME__);
    printf("_etext=%x data[%x-%x] bss[%x-%x]\n", _etext, _data, _edata, __bss_start, __bss_end);

    //LED OFF
    GPIO_SetBits(GPIOA,GPIO_Pin_8);  // LED0
    GPIO_SetBits(GPIOD,GPIO_Pin_2); // LED1

#if 0
    // test udelay
    while (1) {
        for (i = 0; i < 1; i++) {  
            //clock_delay_usec(1); // 5us -> 5.2us ;1us -> 2us
            mdelay(500);
        }

        printf("CLK %d-%d\n", clock_seconds(), clock_time());
        
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)) // LED1
            GPIO_ResetBits(GPIOD, GPIO_Pin_2);
        else
            GPIO_SetBits(GPIOD, GPIO_Pin_2);
    }
#endif
    mdelay(300);
    // LED ON
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);

	process_init();
	process_start(&etimer_process, NULL);


    //cc2520_init();

    // Add shell commands DO NOT use the autostart shell process
    uart1_input_handler = serial_line_input_byte;
    serial_line_init();
    serial_shell_init();
    shell_ps_init();
    shell_ping_init();
    shell_time_init();
    
    shell_tcpsend_init();
    shell_udpsend_init();
    shell_wget_init();

    //TODO test udp/tcp and telnetd/web
    //    tcp_listen

    shell_httpd_init();


#if 0 // test crc
    i = 0;
    memset(buffer, 0, sizeof(buffer));
    // header_len = 3; 0x1, 0x0, 0xff
    // filename = rb.txt
    buffer[i++] = 0x72;
    buffer[i++] = 0x62;
    buffer[i++] = 0x2e;
    buffer[i++] = 0x74;
    buffer[i++] = 0x78;
    buffer[i++] = 0x74;
    buffer[i++] = 0x0;
    // filesize = 27 bytes
    buffer[i++] = 0x32;
    buffer[i++] = 0x37;
    buffer[i++] = 0x20;
    // file attrib
    buffer[i++] = 0x31;
    buffer[i++] = 0x33;
    buffer[i++] = 0x30;
    buffer[i++] = 0x32;
    buffer[i++] = 0x35;
    buffer[i++] = 0x37;
    buffer[i++] = 0x34;
    buffer[i++] = 0x32;
    buffer[i++] = 0x37;
    buffer[i++] = 0x37;
    buffer[i++] = 0x35;
    buffer[i++] = 0x20;
    
    buffer[i++] = 0x31;
    buffer[i++] = 0x30;
    buffer[i++] = 0x30;
    buffer[i++] = 0x36;
    buffer[i++] = 0x36;
    buffer[i++] = 0x34;
    buffer[i++] = 0x20;
    
    buffer[i++] = 0x30;
    buffer[i++] = 0x20;
    
    buffer[i++] = 0x31;
    buffer[i++] = 0x20;
    
    buffer[i++] = 0x32;
    buffer[i++] = 0x37;
    
    buffer[127] = 0x1;

    crc16 = crc16_ccitt(0, buffer, 128);
    printf("crc16=%x\n", crc16);
 while (1);
#endif
    // Enable uart2, add ymodem support
    // add tstc/getc/putc functions for uart2
#if 0 // test UART2 for ymodem
    buffer_len = 0;
    while (1) {
        //ch = getc2();
        putc2('C');
        while (getc2_timeout(&ch, 2000) <= 0) {// 3 +128+2
            putc2('C');
        }
        // read the first packet 128+3+2
        for (i = 0; i < 132; i++) {
            buffer[buffer_len++] = ch;
            printf("%x(%c)\n", ch, ch);
            ch = getc2();
       }
       buffer[buffer_len++] = ch; 
       printf("last %x(%c) read %d\n", ch, ch, buffer_len);

       uint16_t uCRC = crc16_ccitt(0, &buffer[3], 128);
       printf("crc=%x\n", uCRC);
       for (i = 0; i < 134; i++) {
          printf("%x(%c)\n", buffer[i], buffer[i]);
       }
        while (1);
    }
 #endif

    shell_rb_init();
 
    //net-uart
    
    //telnetd_init();

    // Add test commands
    //shell_udptest_init();

    //shell_blink_init();
    //shell_vars_init();

	ctimer_init();

#if NETSTACK_CONF_WITH_IPV6 
    // set MAC address linkaddr_node_addr

    //cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);

    // set local IP address
#if UIP_CONF_ROUTER    
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}    
#endif
#endif

#if 0
      // check local address
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14],
           lladdr->ipaddr.u8[15]);
#endif
#if 0
    // set IPV6 address based on MAC address
    {
      uip_ipaddr_t ipaddr;
      //uip_ip6addr(&ipaddr, 0xaaaa, 0, 1, 2, 3, 4, 5, 6);
        // fe80:0000:0000:0000:5413:aca7:a6ee:6ab5 
        uiplib_ipaddrconv("fe80:0000:0000:0000:a200:0000:0000:0003", &ipaddr);          
      uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
      uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
    }

for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
  if(uip_ds6_if.addr_list[i].isused) {
    printf("IPV6 Address: ");
    //sprint_ip6(uip_ds6_if.addr_list[i].ipaddr);
    uip_debug_ipaddr_print(&(uip_ds6_if.addr_list[i]).ipaddr);
    printf("\n");
  }
}

#endif

#if 0
 if(!UIP_CONF_IPV6_RPL) {
      uip_ipaddr_t ipaddr;
      int i;
      uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
      uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
      uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
      printf("Tentative global IPv6 address ");
      for(i = 0; i < 7; ++i) {
        printf("%02x%02x:",
               ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
      }
      printf("%02x%02x\n",
             ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
    }

#endif
    queuebuf_init();
    //packetbuf_clear();
#if 0
    NETSTACK_RADIO = cc2520
    NETSTACK_RDC = nullrdc;
    NETSTACK_LLSEC = nullsec;
    NETSTACK_MAC = nullmac;
    NETSTACK_NETWORK= sicslowpan;
#endif

    mac_shortaddr = (mac_longaddr[0] << 8) + mac_longaddr[1]; 

    // set the random seed for MAC_addr[7]

    NETSTACK_RADIO.init(); // cc2520_driver.init

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

    NETSTACK_RDC.init();    // nullrdc_driver
    NETSTACK_LLSEC.init();  // nullsec_driver
    NETSTACK_MAC.init();    // nullmac_driver
    NETSTACK_NETWORK.init(); // sicslowpan_driver

    printf(CONTIKI_VERSION_STRING "\n\nTCP started...\n");

    process_start(&tcpip_process, NULL);

    ip64_init();
    
#if 1
    uip_ipaddr(&ipv4addr, 0, 0, 0, 0);
    uip_ipaddr(&netmask, 255, 255, 255, 0);
    ip64_set_ipv4_address(&ipv4addr, &netmask);
#endif

    //process_start(&dhcp_process, NULL);
    //process_start(&about_process, NULL);
        // test ping6
      //process_start(&ping6_process, NULL);

{
  u8 state; 
  printf("Server IPv6 addresses:\n\r");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      //PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      uip_debug_ipaddr_print(&(uip_ds6_if.addr_list[i]).ipaddr);
      printf("\n\r");
    }
  }
}

#if 0
eceived Echo Request from  fe80:0000:0000:0000:b200:0000:0000:0001  to  ff02:0000:0000:0000:0000:0000:0000:0001
Upper layer checksum len: 64 from: 40
Sending Echo Reply to  fe80:0000:0000:0000:b200:0000:0000:0001  from  fe80:0000:0000:0000:0200:0000:0000:0000
Sending packet with length 104 (64)
tcpip_ipv6_output: neighbor not in cache
#endif      
#if 0
      if(node_id > 0) {
        printf("Node id is set to %u.\n", node_id);
      } else {
        printf("Node id is not set.\n");
      }    
#endif

#if 0
#if !UIP_CONF_IPV6_RPL
#ifdef HARD_CODED_ADDRESS
      uip_ipaddr_t ipaddr;
      uiplib_ipaddrconv(HARD_CODED_ADDRESS, &ipaddr);
      if ((ipaddr.u16[0]!=0) || (ipaddr.u16[1]!=0) || (ipaddr.u16[2]!=0) || (ipaddr.u16[3]!=0)) {
#if UIP_CONF_ROUTER
        uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else /* UIP_CONF_ROUTER */
        uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif /* UIP_CONF_ROUTER */
#if !UIP_CONF_IPV6_RPL
        uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
        uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif
      }
#endif /* HARD_CODED_ADDRESS */
#endif
#endif

    //mdelay(2000);
    // test IP
    // tcp_test_init();
    // udp_test_init();
    //process_start(&udp_client_process, NULL);
	//autostart_start(autostart_processes);

	for (;;) 
    {
		do 
        {

		} while(process_run() > 0);
	}
	
	return 0;
}

static void platform_init() 
{
	debug_init();
    uart2_init();
    led_init();
    clock_init();
	rtimer_init();    
}

void
uip_log(char *m)
{
  printf("%s\n", m);
}
