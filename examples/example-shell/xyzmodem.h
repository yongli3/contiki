/*
 *==========================================================================
 *
 *      xyzModem.h
 *
 *      RedBoot stream handler for xyzModem protocol
 *
 *==========================================================================
 * SPDX-License-Identifier:	eCos-2.0
 *==========================================================================
 *#####DESCRIPTIONBEGIN####
 *
 * Author(s):    gthomas
 * Contributors: gthomas
 * Date:         2000-07-14
 * Purpose:
 * Description:
 *
 * This code is part of RedBoot (tm).
 *
 *####DESCRIPTIONEND####
 *
 *==========================================================================
 */

#ifndef _XYZMODEM_H_
#define _XYZMODEM_H_

//#include <stdbool.h>
#define bool unsigned char
#define true  1
#define false 0

#define udelay    clock_delay_usec 

#define MAXRETRYCOUNT 10

#define USE_YMODEM_LENGTH


#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define BSP 0x08
#define NAK 0x15
#define CAN 0x18
#define EOF 0x1A	
#define CTRLZ 0x1A
#define C	0x43

#define YMODEM_TIMEOUT			(901)
#define YMODEM_FILEOPEN_ERROR	(902)
#define YMODEM_RETRYFAIL		(903)
#define YMODEM_FILEWRITE_ERROR	(904)

#define PADDINGBYTE	0xEE
#define HEADER_LEN 3
#define DATA_LEN_1K 1024
#define DATA_LEN_128 128
#define CRC_LEN 2

#define PACKET_LEN_1K (HEADER_LEN + DATA_LEN_1K + CRC_LEN)
//YModem 1024 + 3 head chars + 2 crc
#define BUFFER_SIZE (HEADER_LEN + DATA_LEN_1K + CRC_LEN)
#define PACKET_LEN_128 (HEADER_LEN + DATA_LEN_128 + CRC_LEN)


#define xyzModem_xmodem 1
#define xyzModem_ymodem 2
/* Don't define this until the protocol support is in place */
/*#define xyzModem_zmodem 3 */

#define xyzModem_access   -1
#define xyzModem_noZmodem -2
#define xyzModem_timeout  -3
#define xyzModem_eof      -4
#define xyzModem_cancel   -5
#define xyzModem_frame    -6
#define xyzModem_cksum    -7
#define xyzModem_sequence -8

#define xyzModem_close 1
#define xyzModem_abort 2

#define CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT
#define CYGACC_CALL_IF_SET_CONSOLE_COMM(x)

#define diag_vprintf vprintf
#define diag_printf printf
#define diag_vsprintf vsprintf

#define CYGACC_CALL_IF_DELAY_US(x) udelay(x)

typedef struct {
    char *filename;
    int   mode;
    int   chan;
} connection_info_t;

int   xyzModem_stream_open(connection_info_t *info, int *err);
void  xyzModem_stream_close(int *err);
void  xyzModem_stream_terminate(bool method, int (*getc)(void));
int   xyzModem_stream_read(char *buf, int size, int *err);
char *xyzModem_error(int err);

#endif /* _XYZMODEM_H_ */
