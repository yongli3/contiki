/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "contiki.h"
#include "contiki-net.h"
#include "ip64-dhcpc.h"

#include "ip64.h"
#include "ip64-eth.h"
#include "ip64-addr.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#include <stdio.h>

PROCESS_NAME(shell_httpd_process);
PROCESS_NAME(net_uart_process);
PROCESS_NAME(netclient_process);
PROCESS_NAME(netserver_process);

PROCESS(ip64_ipv4_dhcp_process, "IPv4 DHCP");

uip_ipaddr_t uip_hostaddr; /* Needed because it is referenced by dhcpc.c */


/*---------------------------------------------------------------------------*/
void
ip64_ipv4_dhcp_init(void)
{
  printf("+%s\n", __func__);
  process_start(&ip64_ipv4_dhcp_process, NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ip64_ipv4_dhcp_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("+%s MAC: %X-%X-%X-%X-%X-%X\n", __func__, ip64_eth_addr.addr[0], ip64_eth_addr.addr[1],
    ip64_eth_addr.addr[2], ip64_eth_addr.addr[3], ip64_eth_addr.addr[4], ip64_eth_addr.addr[5]);

  ip64_dhcpc_init(&ip64_eth_addr, sizeof(ip64_eth_addr));

  ip64_dhcpc_request();
  while(1) {
    PROCESS_WAIT_EVENT();

    if(ev == tcpip_event ||
       ev == PROCESS_EVENT_TIMER) {
      ip64_dhcpc_appcall(ev, data);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
ip64_dhcpc_configured(const struct ip64_dhcpc_state *s)
{
  uip_ip6addr_t ip6dnsaddr;
  uip_ip6addr_t local_ipv6_addr;
  printf("DHCP Configured with %d.%d.%d.%d\n",
	 s->ipaddr.u8[0], s->ipaddr.u8[1],
	 s->ipaddr.u8[2], s->ipaddr.u8[3]);

    // set local IPV4 and IPV6 address
  //ip64_set_hostaddr((uip_ip4addr_t *)&s->ipaddr);
  //ip64_set_netmask((uip_ip4addr_t *)&s->netmask);
  ip64_set_ipv4_address((uip_ip4addr_t *)&s->ipaddr, (uip_ip4addr_t *)&s->netmask);

  //FIXME CANNOT re-set the local ipv6 address since DS issue ipv6_local_address
  //ip64_addr_4to6((uip_ip4addr_t *)&s->ipaddr, &local_ipv6_addr);
  //ip64_set_ipv6_address(&local_ipv6_addr);
        
  ip64_set_draddr((uip_ip4addr_t *)&s->default_router);
  ip64_addr_4to6((uip_ip4addr_t *)&s->dnsaddr, &ip6dnsaddr);
  
  //  mdns_conf(&ip6dnsaddr);
#if LOCAL_BUILD
  // start httpd after get IP address FIXME needs to check dhcp expire event
  process_start(&shell_httpd_process, NULL);
  
  process_start(&net_uart_process, NULL);

  extern void set_uart2_event_process(struct process *p);

  set_uart2_event_process(&net_uart_process);
  
  //process_start(&netclient_process, NULL);
  process_start(&netserver_process, NULL);
#endif  
}
/*---------------------------------------------------------------------------*/
void
ip64_dhcpc_unconfigured(const struct ip64_dhcpc_state *s)
{
}
/*---------------------------------------------------------------------------*/
