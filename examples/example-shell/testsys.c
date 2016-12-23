/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "contiki-net.h"
#include <stdio.h>

static struct psock ps;
static uint8_t buffer[100];

PROCESS(test_etimer_process, "Event timer test process");
PROCESS_THREAD(test_etimer_process, ev, data)
{
  static struct etimer et;
  static uint16_t counter = 0;

  PROCESS_BEGIN();

  printf("Starting event timer test process (counter=%i)\n", counter);

  while(1) { 
    // systick = 10000 Hz delay 1s delta= 10013
    // systick = 10000 Hz delay 300us delta= 
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    counter++;
    printf("%lu-%i\n", clock_time(), counter);
  }

  PROCESS_END();
}

static int
handle_connection(struct psock *p)
{
  PSOCK_BEGIN(p);

  PSOCK_SEND_STR(p, "GET / HTTP/1.0\r\n");
  PSOCK_SEND_STR(p, "Server: Contiki example protosocket client\r\n");
  PSOCK_SEND_STR(p, "\r\n");

  while(1) {
    PSOCK_READTO(p, '\n');
    printf("Got: %s", buffer);
  }
  
  PSOCK_END(p);
}

#if 0
static char process_thread_example_psock_client_process(struct pt *process_pt, process_event_t ev, process_data_t data);
struct process example_psock_client_process = { 
    ((void *)0), 
    "Example protosocket client", 
    process_thread_example_psock_client_process 
};

static char process_thread_example_psock_client_process(struct pt *process_pt, process_event_t ev, process_data_t data) {
}
#endif

PROCESS(example_psock_client_process, "Example protosocket client");
PROCESS_THREAD(example_psock_client_process, ev, data)
{
  uip_ipaddr_t addr;
// struct pt *process_pt
  //{ char PT_YIELD_FLAG = 1; if (PT_YIELD_FLAG) {;} switch((process_pt)->lc) { case 0:;
  PROCESS_BEGIN();

  uip_ipaddr(&addr, 192,168,2,1);
  tcp_connect(&addr, UIP_HTONS(80), NULL);

  printf("Connecting...\n");
//  PROCESS_WAIT_EVENT();
//   do { PT_YIELD_FLAG = 0; (process_pt)->lc = 104; case 104:; if(PT_YIELD_FLAG == 0) { return 1; } } while(0);  
  PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
//   do { PT_YIELD_FLAG = 0; (process_pt)->lc = 90; case 90:; if((PT_YIELD_FLAG == 0) || !(ev == tcpip_event)) { return 1; } } while(0);
  if(uip_aborted() || uip_timedout() || uip_closed()) {
    printf("Could not establish connection\n");
  } else if(uip_connected()) {
    printf("Connected\n");
    
    PSOCK_INIT(&ps, buffer, sizeof(buffer));

    do {
      handle_connection(&ps);

      // do { PT_YIELD_FLAG = 0; (process_pt)->lc = 101; case 101:; if((PT_YIELD_FLAG == 0) || !(ev == tcpip_event)) { return 1; } } while(0);
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
      
    } while(!(uip_closed() || uip_aborted() || uip_timedout()));

    printf("\nConnection closed.\n");
  }
  PROCESS_END();

#if 0
   { 
    char PT_YIELD_FLAG = 1;
    if (PT_YIELD_FLAG) {;} 
    switch ((process_pt)->lc) { 
        case 0:;
   }; 
   PT_YIELD_FLAG = 0; 
   (process_pt)->lc = 0;; 
   return 3; 
   };
#endif
}

