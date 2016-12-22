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
#include "xyzmodem.h"

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
static uint8_t FileName[8];
static struct etimer uart_et;

extern uint8_t ymodem_buf[BUFFER_SIZE];
extern uint8_t tCRC[2];

extern struct ringbuf uart2_rxbuf1;
extern struct ringbuf uart2_rxbuf2;
extern struct ringbuf uart2_rxbuf3;
extern struct ringbuf uart2_rxbuf4;

extern struct ringbuf rxbuf2;
process_event_t uart2_event_message;

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
static void YMODEM_sendchar(const char ch)
{
    putc2(ch);
}

#if 0
// use ymodem-1k to send/receive data
PROCESS_THREAD(net_uart_process, ev, data)
{
    int res;
	int i;
	int MaxDataLengthInThisPacket = DATA_LEN_1K;
	int RetryCount = 0;
	uint8_t *pbuf;
    uint8_t PacketNumber;
    unsigned int ErrorCode;
    unsigned int FileLength;

	int ReceivedDataLength;
	int DataLengthInPacket;
	unsigned char rx_char;
    int buf_index;
	unsigned int DataSizeWritten;
    struct uip_udp_conn *udp_conn = NULL;
    uip_ipaddr_t remote_addr;

    static int c;
    static struct etimer et;

  PROCESS_BEGIN();
  etimer_set(&et, CLOCK_SECOND * 2);

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
        PRINTF("Start ...\n");
        for (i = 0; i < BUFFER_SIZE; i++)
            ymodem_buf[i] = PADDINGBYTE;
        
        StartreceiveFileNamePacket:
            ReceivedDataLength = 0;
            PacketNumber = 0;
            buf_index = 0;
            FileName[0] = 0;
        
            //waiting for sender reply first filename packet
            YMODEM_sendchar(C);
            c = -1;
            while (c < 0) {
                PROCESS_YIELD();
                YMODEM_sendchar(C);
                c = ringbuf_get(&rxbuf2);
                        
                #if 0
                etimer_set(&et, CLOCK_SECOND * 2); 
                PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || (ev == uart2_event_message));
                if (ev == uart2_event_message) {
                    c = ringbuf_get(&rxbuf2);
                    rx_char = c;
                    printf("*urt2=%x\n", c);
                } else { // timeout
                    printf("*timeout!\n");
                    c = -1;
                }
                #endif
            }
            rx_char = c;
            while (1) {
                buf_index = 0;
                PRINTF("filename=[%s]\n", FileName);
        
                if (FileName[0] != 0) {
                    //PRINTF("Got filename!\n");
                    c = -1;
                    while (c < 0) {
                        PROCESS_YIELD();
                        c = ringbuf_get(&rxbuf2); 
                    }
                    rx_char = c;
                    res = 1;
                    #if 0
                    etimer_set(&et, CLOCK_SECOND * 2); 
                    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || (ev == uart2_event_message));
                    if (ev == uart2_event_message) {
                        c = ringbuf_get(&rxbuf2);
                        rx_char = c;
                        res = 1;
                    } else { // timeout
                        printf("*timeout!\n");
                        res = -1;
                    }
                    #endif
                    //res = getc2_timeout(&rx_char, xyzModem_CHAR_TIMEOUT);        
                    if (res <= 0) {
                        ErrorCode = YMODEM_TIMEOUT;
                        goto EXIT;
                    }
                }
        
                ymodem_buf[buf_index++] = rx_char;
        
                PRINTF("rx_char=%x\n", rx_char);
                switch (rx_char) {
                case EOT:
                    PacketNumber = 0; // the END
                    //if (fp) {
                    //  f_close(fp);
                    //  fp = NULL;
                    //}
                    YMODEM_sendchar(ACK);
                    PRINTF("%d Start another new file ...\n", ReceivedDataLength);
                    //goto StartreceiveFileNamePacket;
                    ErrorCode = 0;
                    goto EXIT;
                    break;
                case SOH:
                    MaxDataLengthInThisPacket = DATA_LEN_128;
                    break;
                case STX:
                    MaxDataLengthInThisPacket = DATA_LEN_1K;
                    break;
                 default:
                    PRINTF("incorrect?\n");
                    break;
                }
        
                //receive the remaining data
                for (i = 0; i < HEADER_LEN - 1 + MaxDataLengthInThisPacket + CRC_LEN; i++) {// 3-1+2+128
                    c = -1;
                    while (c < 0) {
                        PROCESS_YIELD();
                        c = ringbuf_get(&rxbuf2); 
                    }
                    rx_char = c;
                    res = 1;
                    #if 0
                    etimer_set(&et, CLOCK_SECOND * 2); 
                    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || (ev == uart2_event_message));
                    if (ev == uart2_event_message) {
                        c = ringbuf_get(&rxbuf2);
                        res = c;
                        rx_char = c;
                        res = 1;
                    } else { // timeout
                        printf("timeout!\n");
                        res = -1;
                    }
                    #endif    
                    if (res <= 0) {
                        ErrorCode = YMODEM_TIMEOUT;
                        PRINTF("timeout on %d\n", i);
                        goto EXIT;
                    }
                    ymodem_buf[buf_index++] = rx_char;
                }
                //PRINTF("PacketNumber=%d buf_index=%d\n", PacketNumber, buf_index);
        #if 1
                for (i = 0; i < buf_index; i++) {
                    PRINTF("%x[%c]\n", ymodem_buf[i], ymodem_buf[i]);
                }
        #endif
                //if this packet is the first packet,  check packet integrity and extract filename
                if (PacketNumber == 0 && FileName[0] == 0) {
                    if (false == xCheckPacket(1, &ymodem_buf[HEADER_LEN], MaxDataLengthInThisPacket) 
                        || false == xExtractFileNameAndLength(&ymodem_buf[0], &FileName[0], &FileLength)) {
                        PRINTF("filename packet is garbled start=%x-%x-%x end=%x-%x-%s\n", 
                            ymodem_buf[0], ymodem_buf[1], ymodem_buf[2], ymodem_buf[130], ymodem_buf[131], ymodem_buf[132]);
                        YMODEM_sendchar(NAK);
                        goto StartreceiveFileNamePacket;
                    } else if (FileName[0] == 0) {
                        //no more file
                        PRINTF("no more file \n");
                        goto EXIT;
                    } else { //get filename successfully
                        //check if we can write this file, otherwise abort this transfer
                        //The argument of rb should be the directory path (ended by '\')
                        //strcpy(AbsFileName, pcArgs);
                        //strcat(AbsFileName, FileName);
                        PRINTF("fileanme=%s %d\n", FileName, FileLength);
                        //fr = f_open(&fdDst, AbsFileName, FA_CREATE_ALWAYS | FA_WRITE);
                        //if (fr) {
                            //tell sender to cancel this transfer
                            //YMODEM_sendchar(CAN);
                            //ErrorCode = YMODEM_FILEOPEN_ERROR;
                            //goto EXIT;
                        //}
                        //fp = &fdDst;
        
                        //tell sender we already open the file successfully
                        YMODEM_sendchar(ACK);
                        //initialize a CRC transfer
                        YMODEM_sendchar(C);
                    }
                } else {
                    //data packet
                    //check packet integrity
                    if (ymodem_buf[1] != 255 - ymodem_buf[2] || (ymodem_buf[1] != PacketNumber && ymodem_buf[1] != PacketNumber - 1) 
                        || false == xCheckPacket(1, &ymodem_buf[HEADER_LEN], MaxDataLengthInThisPacket)) {
                        YMODEM_sendchar(NAK);
                        RetryCount++;
                        if (RetryCount > MAXRETRYCOUNT) {
                            ErrorCode = YMODEM_RETRYFAIL;
                            goto EXIT;
                        }
                        continue;
                    }
                    //sender retransmit old packet
                    else if (ymodem_buf[1] == PacketNumber - 1) {
                        YMODEM_sendchar(ACK);
                        continue;
                    }
        
                    //got good data
                    DataLengthInPacket = FileLength - ReceivedDataLength;
                    if (DataLengthInPacket > MaxDataLengthInThisPacket)
                        DataLengthInPacket = MaxDataLengthInThisPacket;
        
                    pbuf = ymodem_buf + HEADER_LEN;
                    PRINTF("send UDP %d\n", DataLengthInPacket);
                    uip_udp_packet_sendto(udp_conn, pbuf, DataLengthInPacket, &remote_addr,
                      UIP_HTONS(7777));
            #if 0
                    fr = f_write(fp, pbuf, DataLengthInPacket, &DataSizeWritten);
                    if (fr) {
                        f_close(fp);
                        fp = NULL;
                        //tell sender to cancel this transfer
                        YMODEM_sendchar(CAN);
                        ErrorCode = YMODEM_FILEWRITE_ERROR;
                        goto EXIT;
                    }
            #endif
                    ReceivedDataLength += DataLengthInPacket;
                    //PRINTF("Recv %d-%d\n", ReceivedDataLength, DataLengthInPacket);
                    //tell sender we got packet successfully
                    YMODEM_sendchar(ACK);
                }
        
                //update status for next packet
                PacketNumber++;
                RetryCount = 0;
            }
        
        EXIT:
            YMODEM_sendchar(ACK);
            //vFlushInput();
        
            if (ErrorCode == 0) {
                PRINTF("\n\rFile is received. %d\n\r", ReceivedDataLength);
            } else {
                PRINTF("\n\rFailed to receive the file eror=%d\n\r", ErrorCode);
            }
            //return;

        
#if 0
      etimer_set(&et, CLOCK_SECOND * 2);      
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || (ev == uart2_event_message));
      if (ev == uart2_event_message) {
          c = ringbuf_get(&rxbuf2);
          printf("*urt2=%x\n", c);  
      } else { // timeout
          printf("*timeout!\n");
      }
#endif
        
  }

  PROCESS_END();
}


#else
#if 0
{
  static struct etimer et;

  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND);

  while(1) {
    PROCESS_WAIT_EVENT();


    if (etimer_expired(&et)) {
      printf("Waiting for serial data\n");
      etimer_restart(&et);
    }

    if(ev == serial_line_event_message) {
      printf("Message received: '%s'\n", data);
    }
  }

  PROCESS_END();
}
#endif

// start after dhcp get IP config
/////*---------------------------------------------------------------------------*/
PROCESS_THREAD(net_uart_process, ev, data)
{
    int size1 = 0;
    int size2 = 0;
    int size3 = 0;
    int size4 = 0;
    int i;
    int len = 0;
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

// FIXME RX overflow issue
#if 0
  In Most cases, this thread will be wakup up every 5ms, 
  but sometimes, the thread cannot get the event, casues the uart2 RX buffer overflow!
  *201-0
  +uip_udp_packet_send len=201
  ev=82 2107005 data=0
  *202-0
  +uip_udp_packet_send len=202
  ev=82 2107055 data=0
  *202-0
  +uip_udp_packet_send len=202
  ev=82 2107109 data=0
  buf1 overflow! 2107155
  ...
  ...
  
  *255-255
  +uip_udp_packet_send len=510
  ev=82 2111171 data=0

Change teh systick IRQ pro to 0, it can intrrupt the UART2 IRQ, but sometimes there are 600 ms delay
for the etimer event to boardcast?
#endif

#if 0
  while(1) {
    PROCESS_WAIT_EVENT();

    size1 = ringbuf_elements(&uart2_rxbuf1);
    size2 = ringbuf_elements(&uart2_rxbuf2);    
    printf("*%d-%d %d\n", size1, size2, clock_time());
    
#if 1
    len = 0;
    if (size1 > 0) {
        for (i = 0; i < size1; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf1);
        }        
    }
    if (size2 > 0) {
        for (i = 0; i < size2; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf2);
        }        
    }
    if (len > 0) {
        uip_udp_packet_sendto(udp_conn, ymodem_buf, len, &remote_addr, UIP_HTONS(REMOTE_PORT));
    }
#endif
     printf("ev=%x %d data=%x\n", ev, clock_time(), data); // PROCESS_EVENT_MAX

    if(ev == PROCESS_EVENT_POLL) {

    } else if(ev == tcpip_event) {
      net_input();
    }
  }
#else
// test rtimer
// delay 2 ms speed = 43KB/s
while (1) {
    etimer_set(&uart_et, 2);
    PROCESS_WAIT_EVENT();
    etimer_reset(&uart_et);
    size1 = ringbuf_elements(&uart2_rxbuf1);
    size2 = ringbuf_elements(&uart2_rxbuf2);
    size3 = ringbuf_elements(&uart2_rxbuf3);
    size4 = ringbuf_elements(&uart2_rxbuf4);
    //printf("*%d-%d %d\n", size1, size2, clock_time());

#if 1
    len = 0;
    if (size1 > 0) {
        for (i = 0; i < size1; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf1);
        }        
    }
    if (size2 > 0) {
        for (i = 0; i < size2; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf2);
        }        
    }

    if (size3 > 0) {
        for (i = 0; i < size3; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf3);
        }        
    }

    if (size4 > 0) {
        for (i = 0; i < size4; i++) {
            ymodem_buf[len++] = ringbuf_get(&uart2_rxbuf4);
        }        
    }
    
    if (len > 0) {
        uip_udp_packet_sendto(udp_conn, ymodem_buf, len, &remote_addr, UIP_HTONS(REMOTE_PORT));
    }    
    //printf("ev=%x %d data=%x\n", ev, clock_time(), data); // PROCESS_EVENT_MAX
#endif


}
#endif
  

  PROCESS_END();
}
#endif

