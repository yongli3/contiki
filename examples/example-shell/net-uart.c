/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 */
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *     A process which receives data over UART and transmits them over UDP
 *     to a pre-defined IPv6 address and port. It also listens on the same UDP
 *     port for messages, which it prints out over UART.
 *
 *     For this example to work, you will have to modify the destination IPv6
 *     address by adjusting the set_dest_addr() macro below.
 *
 *     To listen on your linux or OS X box:
 *     nc -6ulkw 1 REMOTE_PORT
 *
 *     (REMOTE_PORT should be the actual value of the define below, e.g. 7777)
 *
 *     Once netcat is up and listening, type something to the CC26xx's terminal
 *     Bear in mind that the datagram will only be sent after a 0x0a (LF) char
 *     has been received. Therefore, if you are on Win, do NOT use PuTTY for
 *     this purpose, since it does not send 0x0a as part of the line end. On
 *     Win XP use hyperterm. On Win 7 use some other software (e.g. Tera Term,
 *     which can be configured to send CRLF on enter keystrokes).
 *
 *     To send data in the other direction from your linux or OS X box:
 *
 *        nc -6u \<node IPv6 address\> REMOTE_PORT
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/process.h"
#include "dev/serial-line.h"
#include "net/ip/uip.h"
#include "net/ip/uip-udp-packet.h"
#include "net/ip/uiplib.h"
#include "net-uart.h"
#include "sys/cc.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
#define REMOTE_PORT  7777
#define MAX_MSG_SIZE  100

/*---------------------------------------------------------------------------*/
#define ADDRESS_CONVERSION_OK       1
#define ADDRESS_CONVERSION_ERROR    0
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *udp_conn = NULL;

static uint8_t buffer[MAX_MSG_SIZE];
static uint8_t msg_len;
static uip_ip6addr_t remote_addr;
/*---------------------------------------------------------------------------*/
#define IPV6_ADDR_STR_LEN       64
/*---------------------------------------------------------------------------*/
PROCESS(net_uart_process, "Net UART Process");
/*---------------------------------------------------------------------------*/
/*
 * \brief Attempts to convert a string representation of an IPv6 address to a
 * numeric one.
 * \param buf The buffer with the string to be converted.
 * \return ADDRESS_CONVERSION_OK or ADDRESS_CONVERSION_ERROR
 *
 * ToDo: Add support for NAT64 conversion in case the incoming address is a v4
 * This is now supported in the current master, so when we pull it in this will
 * be very straightforward.
 */
/*---------------------------------------------------------------------------*/
static void
net_input(void)
{
    PRINTF("+%s uip_flags=%x\n", __func__, uip_flags);
  if(uip_newdata()) {
    memset(buffer, 0, MAX_MSG_SIZE);
    msg_len = MIN(uip_datalen(), MAX_MSG_SIZE - 1);

    /* Copy data */
    memcpy(buffer, uip_appdata, msg_len);
    printf("%s", (char *)buffer);
  }

  return;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(net_uart_process, ev, data)
{
  PROCESS_BEGIN();

  printf("+%s\n", __func__);
  uip_ip6addr(&remote_addr, 0, 0, 0, 0, 0, 0xffff, 0x0aef, 0x3506);

  PRINTF("remote IP: ");
  uip_debug_ipaddr_print(&remote_addr);
  PRINTF("\n");

  udp_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(udp_conn, UIP_HTONS(REMOTE_PORT));

  if(udp_conn == NULL) {
    printf("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }

  while(1) {
    PROCESS_YIELD();

    if(ev == serial_line_event_message) {
        uip_udp_packet_sendto(
          udp_conn, data, strlen(data), &remote_addr,
          UIP_HTONS(REMOTE_PORT));
    } else if(ev == tcpip_event) {
      net_input();
    }
  }

  PROCESS_END();
}
