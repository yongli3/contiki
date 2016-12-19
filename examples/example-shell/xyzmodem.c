/*
 *==========================================================================
 *
 *      xyzModem.c
 *
 *      RedBoot stream handler for xyzModem protocol
 *
 *==========================================================================
 * SPDX-License-Identifier:	eCos-2.0
 *==========================================================================
 *#####DESCRIPTIONBEGIN####
 *
 * Author(s):    gthomas
 * Contributors: gthomas, tsmith, Yoshinori Sato
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
//#include <common.h>
#include <stm32f10x_type.h>
#include <xyzmodem.h>
//#include <stdarg.h>
#include <crc.h>

/* Assumption - run xyzModem protocol over the console port */

/* Values magic to the protocol */
#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define BSP 0x08
#define NAK 0x15
#define CAN 0x18
#define EOF 0x1A		/* ^Z for DOS officionados */
#define CTRLZ 0x1A
#define C	0x43

#define MAXRETRYCOUNT 30

#define USE_YMODEM_LENGTH

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Data & state local to the protocol */
static struct
{
  int *__chan;
  unsigned char pkt[1024], *bufp;
  unsigned char blk, cblk, crc1, crc2;
  unsigned char next_blk;	/* Expected block */
  int len, mode, total_retries;
  int total_SOH, total_STX, total_CAN;
  bool crc_mode, at_eof, tx_ack;
#ifdef USE_YMODEM_LENGTH
  unsigned long file_length, read_length;
#endif
} xyz;

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

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

uint8_t ymodem_buf[BUFFER_SIZE];
uint8_t tCRC[2];
unsigned int ErrorCode;
unsigned int FileLength;

#define xyzModem_CHAR_TIMEOUT            2000	/* 2 seconds */
#define xyzModem_MAX_RETRIES             20
#define xyzModem_MAX_RETRIES_WITH_CRC    10
#define xyzModem_CAN_COUNT                3	/* Wait for 3 CAN before quitting */

typedef int cyg_int32;
static int CYGACC_COMM_IF_GETC_TIMEOUT(char chan, char *c)
{
  return getc2_timeout(c, xyzModem_CHAR_TIMEOUT);
}


static void CYGACC_COMM_IF_PUTC(char x, char y)
{
  putc2(y);
}

static void YMODEM_sendchar(const char ch)
{
    putc2(ch);
}


/* Validate a hex character */
__inline__ static bool
_is_hex (char c)
{
  return (((c >= '0') && (c <= '9')) ||
	  ((c >= 'A') && (c <= 'F')) || ((c >= 'a') && (c <= 'f')));
}

/* Convert a single hex nibble */
__inline__ static int
_from_hex (char c)
{
  int ret = 0;

  if ((c >= '0') && (c <= '9'))
    {
      ret = (c - '0');
    }
  else if ((c >= 'a') && (c <= 'f'))
    {
      ret = (c - 'a' + 0x0a);
    }
  else if ((c >= 'A') && (c <= 'F'))
    {
      ret = (c - 'A' + 0x0A);
    }
  return ret;
}

/* Convert a character to lower case */
__inline__ static char
_tolower (char c)
{
  if ((c >= 'A') && (c <= 'Z'))
    {
      c = (c - 'A') + 'a';
    }
  return c;
}

/* Parse (scan) a number */
static bool
parse_num(char *s, unsigned long *val, char **es, char *delim)
{
  bool first = true;
  int radix = 10;
  char c;
  unsigned long result = 0;
  int digit;

  while (*s == ' ')
    s++;
  while (*s)
    {
      if (first && (s[0] == '0') && (_tolower (s[1]) == 'x'))
	{
	  radix = 16;
	  s += 2;
	}
      first = false;
      c = *s++;
      if (_is_hex (c) && ((digit = _from_hex (c)) < radix))
	{
	  /* Valid digit */
	  result = (result * radix) + digit;
	}
      else
	{
	  if (delim != (char *) 0)
	    {
	      /* See if this character is one of the delimiters */
	      char *dp = delim;
	      while (*dp && (c != *dp))
		dp++;
	      if (*dp)
		break;		/* Found a good delimiter */
	    }
	  return false;		/* Malformatted number */
	}
    }
  *val = result;
  if (es != (char **) 0)
    {
      *es = s;
    }
  return true;
}

#define USE_SPRINTF
#define ZM_DEBUG(x)

/* Wait for the line to go idle */
static void
xyzModem_flush(void)
{
  int res;
  char c;

    PRINTF("+%s\n", __func__);
  
  while (true)
    {
      res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, &c);
      if (!res)
	return;
    }
}

static bool xCheckPacket(int iCRCMode, const uint8_t *pcBuffer, int iSize)
{
    uint16_t uCRC;
	if (iCRCMode) {
		uint16_t uCRC = crc16_ccitt(0, (unsigned char *)pcBuffer, iSize);
		tCRC[0] = uCRC >> 8;
		tCRC[1] = uCRC & 0xFF;
		uint16_t uCRCInPacket = (pcBuffer[iSize] << 8) + pcBuffer[iSize + 1];
		if (uCRC == uCRCInPacket) {
			return true;
         }
        else {
            PRINTF("%x != %x\n", uCRC, uCRCInPacket);
        }
	} else {
		int i;
		uint8_t uChecksum = 0;
		for (i = 0; i < iSize; ++i) {
			uChecksum += pcBuffer[i];
		}
		if (uChecksum == pcBuffer[iSize])
			return true;
	}

	ErrorCode = 901;
	return false;
}

static bool xExtractFileNameAndLength(uint8_t *buffer, uint8_t *fileName, unsigned int *fileLength)
{

	if (!((*buffer == STX || *buffer == SOH) && *(buffer + 1) == 0 && *(buffer + 2) == 255))
		return false;

	buffer += HEADER_LEN;
	if (*buffer == 0) {
		*fileName = 0;
		*fileLength = 0;
		return true;
	}

	while (*buffer != 0)
		*fileName++ = *buffer++;
	*fileName = 0;

	//extract FileLength
	*fileLength = 0;
	buffer++;
	while ((*buffer != ' ') && (*buffer != 0))
		*fileLength = *fileLength * 10 + (*buffer++ - '0');

	return true;
}

void vRbCommand(char *pcArgs)
{
    int res;
	int i;
	int MaxDataLengthInThisPacket = DATA_LEN_1K;
	int RetryCount = 0;
	uint8_t *pbuf;
    uint8_t FileName[8];
    uint8_t PacketNumber;

	int ReceivedDataLength;
	int DataLengthInPacket;
	unsigned char rx_char;
    int buf_index;
    unsigned int ErrorCode;

	unsigned int DataSizeWritten;

	ErrorCode = 0;

    PRINTF("+%s ymodem_buffer=%x\n", __func__, ymodem_buf);

	while (*pcArgs == ' ' && pcArgs++);

#if 0
	if (*pcArgs == 0) {
		PRINTF("Invalid patch\n\r");
		return;
	}
#endif
	//Fill the data with PADDINGBYTE, then we can know if we miss some data.
	for (i = 0; i < BUFFER_SIZE; i++)
		ymodem_buf[i] = PADDINGBYTE;

	//StartYMODEM();

StartreceiveFileNamePacket :
	ReceivedDataLength = 0;
	PacketNumber = 0;
	buf_index = 0;
	FileName[0] = 0;

	//waiting for sender reply first filename packet
	YMODEM_sendchar(C);
    //rx_char = iInByte(DELAY_1S);
    //while ((rx_char = iInByte(DELAY_1S)) < 0) {
	while (getc2_timeout(&rx_char, xyzModem_CHAR_TIMEOUT) <= 0) {
		RetryCount++;
		YMODEM_sendchar(C);
		if (RetryCount >= MAXRETRYCOUNT) {
			ErrorCode = YMODEM_TIMEOUT;
			goto EXIT;
		}
	}

	//data_buf_index = 0;
	while (1) {
		buf_index = 0;
        PRINTF("filename=[%s]\n", FileName);
		//if filename isn't null, it means we already got the first byte of first filename packet,
		//so we don't receive the first byte again
		if (FileName[0] != 0) {
            res = getc2_timeout(&rx_char, xyzModem_CHAR_TIMEOUT);        
			if (res <= 0) {
				ErrorCode = YMODEM_TIMEOUT;
				goto EXIT;
			}
		}

		ymodem_buf[buf_index++] = rx_char;

        PRINTF("rx_char=%x\n", rx_char);
		switch (rx_char) {
		case EOT:
			PacketNumber = 0;
			//if (fp) {
			//	f_close(fp);
			//	fp = NULL;
			//}
			YMODEM_sendchar(ACK);
			goto StartreceiveFileNamePacket;
			break;
		case SOH:
			MaxDataLengthInThisPacket = DATA_LEN_128;
			break;
		case STX:
			MaxDataLengthInThisPacket = DATA_LEN_1K;
			break;
         default:
            PRINTF("incorrect ? \n");
            break;
		}

		//receive the remaining data
		for (i = 0; i < HEADER_LEN - 1 + MaxDataLengthInThisPacket + CRC_LEN; i++) {// 3-1+2+128
			if (getc2_timeout(&rx_char, xyzModem_CHAR_TIMEOUT) <= 0) {
				ErrorCode = YMODEM_TIMEOUT;
                PRINTF("timeout on %d\n", i);
				goto EXIT;
			}
			ymodem_buf[buf_index++] = rx_char;
		}
        PRINTF("PacketNumber=%d buf_index=%d\n", PacketNumber, buf_index);
        for (i = 0; i < buf_index; i++) {
            PRINTF("%x[%c]\n", ymodem_buf[i], ymodem_buf[i]);
        }
        
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
				goto EXIT;
			} else { //get filename successfully
				//check if we can write this file, otherwise abort this transfer
				//The argument of rb should be the directory path (ended by '\')
				//strcpy(AbsFileName, pcArgs);
				//strcat(AbsFileName, FileName);
                PRINTF("fileanme=%s\n", FileName);
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

			//tell sender we got packet successfully
			YMODEM_sendchar(ACK);
		}

		//update status for next packet
		PacketNumber++;
		RetryCount = 0;
	}
	// end while (1)

EXIT:
	YMODEM_sendchar(ACK);
	//vFlushInput();

	if (ErrorCode == 0) {
		PRINTF("\n\rFile is received.\n\r");
	} else {
		PRINTF("\n\rFailed to receive the file eror=%d\n\r", ErrorCode);
	}
	return;
}


static int
xyzModem_get_hdr(void)
{
  char c = 0;
  int res;
  bool hdr_found = false;
  int i, can_total, hdr_chars;
  unsigned short cksum;

  ZM_DEBUG (zm_new ());
  /* Find the start of a header */
  can_total = 0;
  hdr_chars = 0;

PRINTF("+%s\n", __func__);

  if (xyz.tx_ack)
    {
      CYGACC_COMM_IF_PUTC (*xyz.__chan, ACK);
      xyz.tx_ack = false;
    }
  while (!hdr_found)
    {
      res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, &c);
      PRINTF("%s res=%d c=%x\n", __func__, res, c);
      if (res)
	{
	  hdr_chars++;
	  switch (c)
	    {
	    case SOH:
	      xyz.total_SOH++;
	    case STX:
	      if (c == STX)
		xyz.total_STX++;
	      hdr_found = true;
	      break;
	    case CAN:
	      xyz.total_CAN++;
	      ZM_DEBUG (zm_dump (__LINE__));
	      if (++can_total == xyzModem_CAN_COUNT)
		{
		  return xyzModem_cancel;
		}
	      else
		{
		  /* Wait for multiple CAN to avoid early quits */
		  break;
		}
	    case EOT:
	      /* EOT only supported if no noise */
	      if (hdr_chars == 1)
		{
		  CYGACC_COMM_IF_PUTC (*xyz.__chan, ACK);
		  ZM_DEBUG (zm_dprintf ("ACK on EOT #%d\n", __LINE__));
		  ZM_DEBUG (zm_dump (__LINE__));
		  return xyzModem_eof;
		}
	    default:
	      /* Ignore, waiting for start of header */
	      ;
	    }
	}
      else
	{
	  /* Data stream timed out */
	  xyzModem_flush ();	/* Toss any current input */
	  ZM_DEBUG (zm_dump (__LINE__));
      mdelay(250);
	  //CYGACC_CALL_IF_DELAY_US ((cyg_int32) 250000);
	  return xyzModem_timeout;
	}
    }

  /* Header found, now read the data */
  res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, (char *) &xyz.blk);
  PRINTF("blk=%x\n", xyz.blk);
  if (!res)
    {
      ZM_DEBUG (zm_dump (__LINE__));
      return xyzModem_timeout;
    }
  res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, (char *) &xyz.cblk);
  PRINTF("cblk=%x\n", xyz.cblk);
  if (!res)
    {
      ZM_DEBUG (zm_dump (__LINE__));
      return xyzModem_timeout;
    }
  xyz.len = (c == SOH) ? 128 : 1024;
  xyz.bufp = xyz.pkt;
  PRINTF("buf_len=%d\n", xyz.len);
  for (i = 0; i < xyz.len; i++)
    {
      res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, &c);
      ZM_DEBUG (zm_save (c));
      if (res)
	{
	  xyz.pkt[i] = c;
	}
      else
	{
	  ZM_DEBUG (zm_dump (__LINE__));
	  return xyzModem_timeout;
	}
    }
  res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, (char *) &xyz.crc1);
  ZM_DEBUG (zm_save (xyz.crc1));
  if (!res)
    {
      ZM_DEBUG (zm_dump (__LINE__));
      return xyzModem_timeout;
    }
  if (xyz.crc_mode)
    {
      res = CYGACC_COMM_IF_GETC_TIMEOUT (*xyz.__chan, (char *) &xyz.crc2);
      ZM_DEBUG (zm_save (xyz.crc2));
      if (!res)
	{
	  ZM_DEBUG (zm_dump (__LINE__));
	  return xyzModem_timeout;
	}
    }
  ZM_DEBUG (zm_dump (__LINE__));
  /* Validate the message */
  if ((xyz.blk ^ xyz.cblk) != (unsigned char) 0xFF)
    {
      PRINTF("Framing error - blk: %x/%x/%x\n", xyz.blk, xyz.cblk,
		 (xyz.blk ^ xyz.cblk))
      ZM_DEBUG (zm_dprintf
		("Framing error - blk: %x/%x/%x\n", xyz.blk, xyz.cblk,
		 (xyz.blk ^ xyz.cblk)));
      ZM_DEBUG (zm_dump_buf (xyz.pkt, xyz.len));
      xyzModem_flush ();
      return xyzModem_frame;
    }
  /* Verify checksum/CRC */
  if (xyz.crc_mode)
    {
      cksum = crc16_ccitt(0, xyz.pkt, xyz.len);
      if (cksum != ((xyz.crc1 << 8) | xyz.crc2))
	{
	    PRINTF("CRC error - recvd: %02x%02x, computed: %x\n",
				xyz.crc1, xyz.crc2, cksum & 0xFFFF);
	  ZM_DEBUG (zm_dprintf ("CRC error - recvd: %02x%02x, computed: %x\n",
				xyz.crc1, xyz.crc2, cksum & 0xFFFF));
	  return xyzModem_cksum;
	}
    }
  else
    {
      cksum = 0;
      for (i = 0; i < xyz.len; i++)
	{
	  cksum += xyz.pkt[i];
	}
      if (xyz.crc1 != (cksum & 0xFF))
	{
	  PRINTF("Checksum error - recvd: %x, computed: %x\n", xyz.crc1,
		     cksum & 0xFF);
	  ZM_DEBUG (zm_dprintf
		    ("Checksum error - recvd: %x, computed: %x\n", xyz.crc1,
		     cksum & 0xFF));
	  return xyzModem_cksum;
	}
    }
  /* If we get here, the message passes [structural] muster */
  return 0;
}

int
xyzModem_stream_open(connection_info_t * info, int *err)
{
  int stat = 0;
  int retries = xyzModem_MAX_RETRIES;
  int crc_retries = xyzModem_MAX_RETRIES_WITH_CRC;

/* TODO: CHECK ! */
  int dummy = 0;
  xyz.__chan = &dummy;
  xyz.len = 0;
  xyz.crc_mode = true;
  xyz.at_eof = false;
  xyz.tx_ack = false;
  xyz.mode = info->mode;
  xyz.total_retries = 0;
  xyz.total_SOH = 0;
  xyz.total_STX = 0;
  xyz.total_CAN = 0;
#ifdef USE_YMODEM_LENGTH
  xyz.read_length = 0;
  xyz.file_length = 0;
#endif

  CYGACC_COMM_IF_PUTC (*xyz.__chan, (xyz.crc_mode ? 'C' : NAK));

  if (xyz.mode == xyzModem_xmodem)
    {
      /* X-modem doesn't have an information header - exit here */
      xyz.next_blk = 1;
      return 0;
    }

  while (retries-- > 0)
    {
      stat = xyzModem_get_hdr ();
      if (stat == 0)
	{
	  /* Y-modem file information header */
	  if (xyz.blk == 0)
	    {
#ifdef USE_YMODEM_LENGTH
	      /* skip filename */
	      while (*xyz.bufp++);
	      /* get the length */
	      parse_num ((char *) xyz.bufp, &xyz.file_length, NULL, " ");
          PRINTF("file_len=%d\n", xyz.file_length);
#endif
	      /* The rest of the file name data block quietly discarded */
	      xyz.tx_ack = true;
	    }
	  xyz.next_blk = 1;
	  xyz.len = 0;
	  return 0;
	}
      else if (stat == xyzModem_timeout)
	{
	    PRINTF("%s retries=%d crc_retries=%d crc_mode=%d\n", __func__, retries, crc_retries, xyz.crc_mode);
	  if (--crc_retries <= 0)
	    xyz.crc_mode = false;

      mdelay(500);
	  //CYGACC_CALL_IF_DELAY_US (5 * 100000);	/* Extra delay for startup */
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, (xyz.crc_mode ? 'C' : NAK));
	  xyz.total_retries++;
	  ZM_DEBUG (zm_dprintf ("NAK (%d)\n", __LINE__));
	}
      if (stat == xyzModem_cancel)
	{
	  break;
	}
    }
  *err = stat;
  ZM_DEBUG (zm_flush ());
  return -1;
}

int
xyzModem_stream_read (char *buf, int size, int *err)
{
  int stat, total, len;
  int retries;

  total = 0;
  stat = xyzModem_cancel;
  /* Try and get 'size' bytes into the buffer */
  while (!xyz.at_eof && (size > 0))
    {
      if (xyz.len == 0)
	{
	  retries = xyzModem_MAX_RETRIES;
	  while (retries-- > 0)
	    {
	      stat = xyzModem_get_hdr ();
	      if (stat == 0)
		{
		  if (xyz.blk == xyz.next_blk)
		    {
		      xyz.tx_ack = true;
		      ZM_DEBUG (zm_dprintf
				("ACK block %d (%d)\n", xyz.blk, __LINE__));
		      xyz.next_blk = (xyz.next_blk + 1) & 0xFF;

#ifdef USE_YMODEM_LENGTH
		      if (xyz.mode == xyzModem_xmodem || xyz.file_length == 0)
			{
#else
		      if (1)
			{
#endif
			  /* Data blocks can be padded with ^Z (EOF) characters */
			  /* This code tries to detect and remove them */
			  if ((xyz.bufp[xyz.len - 1] == EOF) &&
			      (xyz.bufp[xyz.len - 2] == EOF) &&
			      (xyz.bufp[xyz.len - 3] == EOF))
			    {
			      while (xyz.len
				     && (xyz.bufp[xyz.len - 1] == EOF))
				{
				  xyz.len--;
				}
			    }
			}

#ifdef USE_YMODEM_LENGTH
		      /*
		       * See if accumulated length exceeds that of the file.
		       * If so, reduce size (i.e., cut out pad bytes)
		       * Only do this for Y-modem (and Z-modem should it ever
		       * be supported since it can fall back to Y-modem mode).
		       */
		      if (xyz.mode != xyzModem_xmodem && 0 != xyz.file_length)
			{
			  xyz.read_length += xyz.len;
			  if (xyz.read_length > xyz.file_length)
			    {
			      xyz.len -= (xyz.read_length - xyz.file_length);
			    }
			}
#endif
		      break;
		    }
		  else if (xyz.blk == ((xyz.next_blk - 1) & 0xFF))
		    {
		      /* Just re-ACK this so sender will get on with it */
		      CYGACC_COMM_IF_PUTC (*xyz.__chan, ACK);
		      continue;	/* Need new header */
		    }
		  else
		    {
		      stat = xyzModem_sequence;
		    }
		}
	      if (stat == xyzModem_cancel)
		{
		  break;
		}
	      if (stat == xyzModem_eof)
		{
		  CYGACC_COMM_IF_PUTC (*xyz.__chan, ACK);
		  ZM_DEBUG (zm_dprintf ("ACK (%d)\n", __LINE__));
		  if (xyz.mode == xyzModem_ymodem)
		    {
		      CYGACC_COMM_IF_PUTC (*xyz.__chan,
					   (xyz.crc_mode ? 'C' : NAK));
		      xyz.total_retries++;
		      ZM_DEBUG (zm_dprintf ("Reading Final Header\n"));
		      stat = xyzModem_get_hdr ();
		      CYGACC_COMM_IF_PUTC (*xyz.__chan, ACK);
		      ZM_DEBUG (zm_dprintf ("FINAL ACK (%d)\n", __LINE__));
		    }
		  xyz.at_eof = true;
		  break;
		}
	      CYGACC_COMM_IF_PUTC (*xyz.__chan, (xyz.crc_mode ? 'C' : NAK));
	      xyz.total_retries++;
	      ZM_DEBUG (zm_dprintf ("NAK (%d)\n", __LINE__));
	    }
	  if (stat < 0)
	    {
	      *err = stat;
	      xyz.len = -1;
	      return total;
	    }
	}
      /* Don't "read" data from the EOF protocol package */
      if (!xyz.at_eof)
	{
	  len = xyz.len;
	  if (size < len)
	    len = size;
	  memcpy (buf, xyz.bufp, len);
	  size -= len;
	  buf += len;
	  total += len;
	  xyz.len -= len;
	  xyz.bufp += len;
	}
    }
  return total;
}

void
xyzModem_stream_close (int *err)
{
  diag_printf
    ("xyzModem - %s mode, %d(SOH)/%d(STX)/%d(CAN) packets, %d retries\n",
     xyz.crc_mode ? "CRC" : "Cksum", xyz.total_SOH, xyz.total_STX,
     xyz.total_CAN, xyz.total_retries);
  ZM_DEBUG (zm_flush ());
}

/* Need to be able to clean out the input buffer, so have to take the */
/* getc */
void
xyzModem_stream_terminate (bool abort, int (*getc) (void))
{
  int c;

  if (abort)
    {
      ZM_DEBUG (zm_dprintf ("!!!! TRANSFER ABORT !!!!\n"));
      switch (xyz.mode)
	{
	case xyzModem_xmodem:
	case xyzModem_ymodem:
	  /* The X/YMODEM Spec seems to suggest that multiple CAN followed by an equal */
	  /* number of Backspaces is a friendly way to get the other end to abort. */
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, CAN);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, CAN);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, CAN);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, CAN);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, BSP);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, BSP);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, BSP);
	  CYGACC_COMM_IF_PUTC (*xyz.__chan, BSP);
	  /* Now consume the rest of what's waiting on the line. */
	  ZM_DEBUG (zm_dprintf ("Flushing serial line.\n"));
	  xyzModem_flush ();
	  xyz.at_eof = true;
	  break;
#ifdef xyzModem_zmodem
	case xyzModem_zmodem:
	  /* Might support it some day I suppose. */
#endif
	  break;
	}
    }
  else
    {
      ZM_DEBUG (zm_dprintf ("Engaging cleanup mode...\n"));
      /*
       * Consume any trailing crap left in the inbuffer from
       * previous received blocks. Since very few files are an exact multiple
       * of the transfer block size, there will almost always be some gunk here.
       * If we don't eat it now, RedBoot will think the user typed it.
       */
      ZM_DEBUG (zm_dprintf ("Trailing gunk:\n"));
      while ((c = (*getc) ()) > -1)
        ;
      ZM_DEBUG (zm_dprintf ("\n"));
      /*
       * Make a small delay to give terminal programs like minicom
       * time to get control again after their file transfer program
       * exits.
       */
      mdelay(250); 
      //CYGACC_CALL_IF_DELAY_US ((cyg_int32) 250000);
    }
}

char *
xyzModem_error (int err)
{
  switch (err)
    {
    case xyzModem_access:
      return "Can't access file";
      break;
    case xyzModem_noZmodem:
      return "Sorry, zModem not available yet";
      break;
    case xyzModem_timeout:
      return "Timed out";
      break;
    case xyzModem_eof:
      return "End of file";
      break;
    case xyzModem_cancel:
      return "Cancelled";
      break;
    case xyzModem_frame:
      return "Invalid framing";
      break;
    case xyzModem_cksum:
      return "CRC/checksum error";
      break;
    case xyzModem_sequence:
      return "Block sequence error";
      break;
    default:
      return "Unknown error";
      break;
    }
}
