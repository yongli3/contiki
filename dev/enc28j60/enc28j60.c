/*
 * Copyright (c) 2012-2013, Thingsquare, http://www.thingsquare.com/.
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
 *
 */

#include "contiki.h"
#include "enc28j60.h"
#include <stdio.h>
#include <string.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define EIE   0x1b
#define EIR   0x1c
#define ESTAT 0x1d
#define ECON2 0x1e
#define ECON1 0x1f

#define ESTAT_CLKRDY 0x01
#define ESTAT_TXABRT 0x02

#define ECON1_RXEN   0x04
#define ECON1_TXRTS  0x08

#define ECON2_AUTOINC 0x80
#define ECON2_PKTDEC  0x40

#define EIR_TXIF      0x08

#define ERXTX_BANK 0x00

#define ERDPTL 0x00
#define ERDPTH 0x01
#define EWRPTL 0x02
#define EWRPTH 0x03
#define ETXSTL 0x04
#define ETXSTH 0x05
#define ETXNDL 0x06
#define ETXNDH 0x07
#define ERXSTL 0x08
#define ERXSTH 0x09
#define ERXNDL 0x0a
#define ERXNDH 0x0b
#define ERXRDPTL 0x0c
#define ERXRDPTH 0x0d

#define RX_BUF_START 0x0000
#define RX_BUF_END   0x0fff

#define TX_BUF_START 0x1200

/* MACONx registers are in bank 2 */
#define MACONX_BANK 0x02

#define MACON1  0x00
#define MACON3  0x02
#define MACON4  0x03
#define MABBIPG 0x04
#define MAIPGL  0x06
#define MAIPGH  0x07
#define MAMXFLL 0x0a
#define MAMXFLH 0x0b

#define MACON1_TXPAUS 0x08
#define MACON1_RXPAUS 0x04
#define MACON1_MARXEN 0x01

#define MACON3_PADCFG_FULL 0xe0
#define MACON3_TXCRCEN     0x10
#define MACON3_FRMLNEN     0x02
#define MACON3_FULDPX      0x01

#define MAX_MAC_LENGTH 1518

#define MAADRX_BANK 0x03
#define MAADR1 0x04 /* MAADR<47:40> */
#define MAADR2 0x05 /* MAADR<39:32> */
#define MAADR3 0x02 /* MAADR<31:24> */
#define MAADR4 0x03 /* MAADR<23:16> */
#define MAADR5 0x00 /* MAADR<15:8> */
#define MAADR6 0x01 /* MAADR<7:0> */
#define MISTAT 0x0a
#define EREVID 0x12

#define EPKTCNT_BANK 0x01
#define ERXFCON 0x18
#define EPKTCNT 0x19

#define ERXFCON_UCEN  0x80
#define ERXFCON_ANDOR 0x40
#define ERXFCON_CRCEN 0x20
#define ERXFCON_MCEN  0x02
#define ERXFCON_BCEN  0x01


PROCESS(enc_watchdog_process, "Enc28j60 watchdog");

static uint8_t initialized = 0;
static uint8_t bank = ERXTX_BANK;
static uint8_t enc_mac_addr[6];
static int received_packets = 0;
static int sent_packets = 0;

void enc28j60_arch_spi_init(void)
{
    u8 reg, val;

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure; 
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_SPI1, ENABLE);	

    // PA4 = SPI_NSS output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    // PC4 = RESET output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //pull up output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);

    // PA1 = VREG_EN output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP  ;   // pull up output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);

    mdelay(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_4);
    mdelay(200);

    //PA4=NSS/PA5=SCK/PA6=MISO/PA7=MOSI = SPI1 master
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);	

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//选择了串行时钟的稳态:时钟悬空低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//数据捕获于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式

    SPI_Init(SPI1, &SPI_InitStructure);
    // use hardware NSS
    //SPI_SSOutputCmd(SPI1, ENABLE);

    SPI_Cmd(SPI1, ENABLE);

#if 0
    val = cc2520_read_reg(CC2520_CHIPID);
    printf("\r\nCC2520 ID=0x%x\r\n", val);

    val = cc2520_read_reg(CC2520_CHIPID);
    printf("\r\nCC2520 ID=0x%x\r\n", val);    

    val = cc2520_read_reg(CC2520_VERSION);
    printf("\r\CC2520 nversion=0x%x\r\n", val);    

    CC2520_SPI_DISABLE();                /* Unselect radio. */
    cc2520_irq_init();
#endif    
}

static u8 SPI1_ReadWriteByte(u8 TxData)
{		
    // Loop while DR register in not emplty
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // TXE=1 = TX empty
    
	SPI_I2S_SendData(SPI1, TxData);

    // Wait to receive a byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);// RXNE=1 RX full
	  						    
	return SPI_I2S_ReceiveData(SPI1);					    
}

static uint8_t enc28j60_arch_spi_write(u8 TxData)
{
    SPI1_ReadWriteByte(TxData);
}

static uint8_t enc28j60_arch_spi_read(void)
{
    return SPI1_ReadWriteByte(0xAA);
}
void enc28j60_arch_spi_select(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    //mdelay(1);
}
void enc28j60_arch_spi_deselect(void)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

/*---------------------------------------------------------------------------*/
static uint8_t
is_mac_mii_reg(uint8_t reg)
{
  /* MAC or MII register (otherwise, ETH register)? */
  switch(bank) {
  case MACONX_BANK:
    return reg < EIE;
  case MAADRX_BANK:
    return reg <= MAADR2 || reg == MISTAT;
  case ERXTX_BANK:
  case EPKTCNT_BANK:
  default:
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static uint8_t
readreg(uint8_t reg)
{
  uint8_t r;
  enc28j60_arch_spi_select();
  enc28j60_arch_spi_write(0x00 | (reg & 0x1f));
  if(is_mac_mii_reg(reg)) {
    /* MAC and MII registers require that a dummy byte be read first. */
    enc28j60_arch_spi_read();
  }
  r = enc28j60_arch_spi_read();
  enc28j60_arch_spi_deselect();
  return r;
}
/*---------------------------------------------------------------------------*/
static void
writereg(uint8_t reg, uint8_t data)
{
  enc28j60_arch_spi_select();
  enc28j60_arch_spi_write(0x40 | (reg & 0x1f));
  enc28j60_arch_spi_write(data);
  enc28j60_arch_spi_deselect();
}
/*---------------------------------------------------------------------------*/
static void
setregbitfield(uint8_t reg, uint8_t mask)
{
  if(is_mac_mii_reg(reg)) {
    writereg(reg, readreg(reg) | mask);
  } else {
    enc28j60_arch_spi_select();
    enc28j60_arch_spi_write(0x80 | (reg & 0x1f));
    enc28j60_arch_spi_write(mask);
    enc28j60_arch_spi_deselect();
  }
}
/*---------------------------------------------------------------------------*/
static void
clearregbitfield(uint8_t reg, uint8_t mask)
{
  if(is_mac_mii_reg(reg)) {
    writereg(reg, readreg(reg) & ~mask);
  } else {
    enc28j60_arch_spi_select();
    enc28j60_arch_spi_write(0xa0 | (reg & 0x1f));
    enc28j60_arch_spi_write(mask);
    enc28j60_arch_spi_deselect();
  }
}
/*---------------------------------------------------------------------------*/
static void
setregbank(uint8_t new_bank)
{
  writereg(ECON1, (readreg(ECON1) & 0xfc) | (new_bank & 0x03));
  bank = new_bank;
}
/*---------------------------------------------------------------------------*/
static void
writedata(const uint8_t *data, int datalen)
{
  int i;
  enc28j60_arch_spi_select();
  /* The Write Buffer Memory (WBM) command is 0 1 1 1 1 0 1 0  */
  enc28j60_arch_spi_write(0x7a);
  for(i = 0; i < datalen; i++) {
    enc28j60_arch_spi_write(data[i]);
  }
  enc28j60_arch_spi_deselect();
}
/*---------------------------------------------------------------------------*/
static void
writedatabyte(uint8_t byte)
{
  writedata(&byte, 1);
}
/*---------------------------------------------------------------------------*/
static int
readdata(uint8_t *buf, int len)
{
  int i;
  enc28j60_arch_spi_select();
  /* THe Read Buffer Memory (RBM) command is 0 0 1 1 1 0 1 0 */
  enc28j60_arch_spi_write(0x3a);
  for(i = 0; i < len; i++) {
    buf[i] = enc28j60_arch_spi_read();
  }
  enc28j60_arch_spi_deselect();
  return i;
}
/*---------------------------------------------------------------------------*/
static uint8_t
readdatabyte(void)
{
  uint8_t r;
  readdata(&r, 1);
  return r;
}
/*---------------------------------------------------------------------------*/
static void
softreset(void)
{
  enc28j60_arch_spi_select();
  /* The System Command (soft reset) is 1 1 1 1 1 1 1 1 */
  enc28j60_arch_spi_write(0xff);
  enc28j60_arch_spi_deselect();
  clock_delay_usec(2000);
  bank = ERXTX_BANK;
}
/*---------------------------------------------------------------------------*/
#if DEBUG
static uint8_t
readrev(void)
{
  uint8_t rev;
  setregbank(MAADRX_BANK);
  rev = readreg(EREVID);
  switch(rev) {
  case 2:
    return 1;
  case 6:
    return 7;
  default:
    return rev;
  }
}
#endif
/*---------------------------------------------------------------------------*/
static void
reset(void)
{
  int i = 0;
  PRINTF("enc28j60: resetting chip\n");

  enc28j60_arch_spi_init();

  /*
    6.0 INITIALIZATION

    Before the ENC28J60 can be used to transmit and receive packets,
    certain device settings must be initialized. Depending on the
    application, some configuration options may need to be
    changed. Normally, these tasks may be accomplished once after
    Reset and do not need to be changed thereafter.

    6.1 Receive Buffer

    Before receiving any packets, the receive buffer must be
    initialized by programming the ERXST and ERXND pointers. All
    memory between and including the ERXST and ERXND addresses will be
    dedicated to the receive hardware. It is recommended that the
    ERXST pointer be programmed with an even address.

    Applications expecting large amounts of data and frequent packet
    delivery may wish to allocate most of the memory as the receive
    buffer. Applications that may need to save older packets or have
    several packets ready for transmission should allocate less
    memory.

    When programming the ERXST pointer, the ERXWRPT registers will
    automatically be updated with the same values. The address in
    ERXWRPT will be used as the starting location when the receive
    hardware begins writing received data. For tracking purposes, the
    ERXRDPT registers should additionally be programmed with the same
    value. To program ERXRDPT, the host controller must write to
    ERXRDPTL first, followed by ERXRDPTH.  See Section 7.2.4 鈥淔reeing
    Receive Buffer Space for more information

    6.2 Transmission Buffer

    All memory which is not used by the receive buffer is considered
    the transmission buffer. Data which is to be transmitted should be
    written into any unused space.  After a packet is transmitted,
    however, the hardware will write a seven-byte status vector into
    memory after the last byte in the packet. Therefore, the host
    controller should leave at least seven bytes between each packet
    and the beginning of the receive buffer. No explicit action is
    required to initialize the transmission buffer.

    6.3 Receive Filters

    The appropriate receive filters should be enabled or disabled by
    writing to the ERXFCON register. See Section 8.0 鈥淩eceive Filters
    for information on how to configure it.

    6.4 Waiting For OST

    If the initialization procedure is being executed immediately
    following a Power-on Reset, the ESTAT.CLKRDY bit should be polled
    to make certain that enough time has elapsed before proceeding to
    modify the MAC and PHY registers. For more information on the OST,
    see Section 2.2 鈥淥scillator Start-up Timer.
  */

    softreset();

    /* Wait for OST */
    while((readreg(ESTAT) & ESTAT_CLKRDY) == 0);

    //PRINTF("ID=%d\n", readreg(EREVID));
#if 0
    setregbank(ERXTX_BANK);
    for (i = 0; i < 0x20; i++) {
        PRINTF("dump reg%d-%x 0x%x\n", bank, i, readreg(i));
    }

    setregbank(EPKTCNT_BANK);
    for (i = 0; i < 0x20; i++) {
        PRINTF("dump reg%d-%x 0x%x\n", bank, i, readreg(i));
    }   
  
    setregbank(MACONX_BANK);
    for (i = 0; i < 0x20; i++) {
        PRINTF("dump reg%d-%x 0x%x\n", bank, i, readreg(i));
    }        

    setregbank(MAADRX_BANK);
    for (i = 0; i < 0x20; i++) {
        PRINTF("dump reg%d-%x 0x%x\n", bank, i, readreg(i));
    }
#endif
    PRINTF("rev=%d\n", readrev());  

  setregbank(ERXTX_BANK);
  /* Set up receive buffer */
  writereg(ERXSTL, RX_BUF_START & 0xff);
  writereg(ERXSTH, RX_BUF_START >> 8);
  writereg(ERXNDL, RX_BUF_END & 0xff);
  writereg(ERXNDH, RX_BUF_END >> 8);
  writereg(ERDPTL, RX_BUF_START & 0xff);
  writereg(ERDPTH, RX_BUF_START >> 8);
  writereg(ERXRDPTL, RX_BUF_END & 0xff);
  writereg(ERXRDPTH, RX_BUF_END >> 8);

  /* Receive filters */
  setregbank(EPKTCNT_BANK);
  writereg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

  /*
    6.5 MAC Initialization Settings

    Several of the MAC registers require configuration during
    initialization. This only needs to be done once; the order of
    programming is unimportant.

    1. Set the MARXEN bit in MACON1 to enable the MAC to receive
    frames. If using full duplex, most applications should also set
    TXPAUS and RXPAUS to allow IEEE defined flow control to function.

    2. Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3. Most
    applications should enable automatic padding to at least 60 bytes
    and always append a valid CRC. For convenience, many applications
    may wish to set the FRMLNEN bit as well to enable frame length
    status reporting. The FULDPX bit should be set if the application
    will be connected to a full-duplex configured remote node;
    otherwise, it should be left clear.

    3. Configure the bits in MACON4. For conformance to the IEEE 802.3
    standard, set the DEFER bit.

    4. Program the MAMXFL registers with the maximum frame length to
    be permitted to be received or transmitted. Normal network nodes
    are designed to handle packets that are 1518 bytes or less.

    5. Configure the Back-to-Back Inter-Packet Gap register,
    MABBIPG. Most applications will program this register with 15h
    when Full-Duplex mode is used and 12h when Half-Duplex mode is
    used.

    6. Configure the Non-Back-to-Back Inter-Packet Gap register low
    byte, MAIPGL. Most applications will program this register with
    12h.

    7. If half duplex is used, the Non-Back-to-Back Inter-Packet Gap
    register high byte, MAIPGH, should be programmed. Most
    applications will program this register to 0Ch.

    8. If Half-Duplex mode is used, program the Retransmission and
    Collision Window registers, MACLCON1 and MACLCON2. Most
    applications will not need to change the default Reset values.  If
    the network is spread over exceptionally long cables, the default
    value of MACLCON2 may need to be increased.

    9. Program the local MAC address into the MAADR1:MAADR6 registers.
  */

  setregbank(MACONX_BANK);

  /* Turn on reception and IEEE-defined flow control */
  setregbitfield(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

  /* Set padding, crc, full duplex */
  setregbitfield(MACON3, MACON3_PADCFG_FULL | MACON3_TXCRCEN | MACON3_FULDPX |
                         MACON3_FRMLNEN);

  /* Don't modify MACON4 */

  /* Set maximum frame length in MAMXFL */
  writereg(MAMXFLL, MAX_MAC_LENGTH & 0xff);
  writereg(MAMXFLH, MAX_MAC_LENGTH >> 8);

  /* Set back-to-back inter packet gap */
  writereg(MABBIPG, 0x15);

  /* Set non-back-to-back packet gap */
  writereg(MAIPGL, 0x12);

  /* Set MAC address */
  setregbank(MAADRX_BANK);
  writereg(MAADR6, enc_mac_addr[5]);
  writereg(MAADR5, enc_mac_addr[4]);
  writereg(MAADR4, enc_mac_addr[3]);
  writereg(MAADR3, enc_mac_addr[2]);
  writereg(MAADR2, enc_mac_addr[1]);
  writereg(MAADR1, enc_mac_addr[0]);

  /*
    6.6 PHY Initialization Settings

    Depending on the application, bits in three of the PHY module鈥檚
    registers may also require configuration.  The PHCON1.PDPXMD bit
    partially controls the device鈥檚 half/full-duplex
    configuration. Normally, this bit is initialized correctly by the
    external circuitry (see Section 2.6 鈥淟ED Configuration). If the
    external circuitry is not present or incorrect, however, the host
    controller must program the bit properly. Alternatively, for an
    externally configurable system, the PDPXMD bit may be read and the
    FULDPX bit be programmed to match.

    For proper duplex operation, the PHCON1.PDPXMD bit must also match
    the value of the MACON3.FULDPX bit.

    If using half duplex, the host controller may wish to set the
    PHCON2.HDLDIS bit to prevent automatic loopback of the data which
    is transmitted.  The PHY register, PHLCON, controls the outputs of
    LEDA and LEDB. If an application requires a LED configuration
    other than the default, PHLCON must be altered to match the new
    requirements. The settings for LED operation are discussed in
    Section 2.6 鈥淟ED Configuration. The PHLCON register is shown in
    Register 2-2 (page 9).
  */

  /* Don't worry about PHY configuration for now */

  /* Turn on autoincrement for buffer access */
  setregbitfield(ECON2, ECON2_AUTOINC);

  /* Turn on reception */
  writereg(ECON1, ECON1_RXEN);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_init(const uint8_t *mac_addr)
{
  if(initialized) {
    return;
  }

  memcpy(enc_mac_addr, mac_addr, 6);

  /* Start watchdog process */
  process_start(&enc_watchdog_process, NULL);

  reset();

  PRINTF("ENC28J60 rev. B%d\n", readrev());

  initialized = 1;
}
/*---------------------------------------------------------------------------*/
int
enc28j60_send(const uint8_t *data, uint16_t datalen)
{
  uint16_t dataend;

  PRINTF("%s %d\n", __func__, datalen);

  if(!initialized) {
    return -1;
  }

  /*
    1. Appropriately program the ETXST pointer to point to an unused
       location in memory. It will point to the per packet control
       byte. In the example, it would be programmed to 0120h. It is
       recommended that an even address be used for ETXST.

    2. Use the WBM SPI command to write the per packet control byte,
       the destination address, the source MAC address, the
       type/length and the data payload.

    3. Appropriately program the ETXND pointer. It should point to the
       last byte in the data payload.  In the example, it would be
       programmed to 0156h.

    4. Clear EIR.TXIF, set EIE.TXIE and set EIE.INTIE to enable an
       interrupt when done (if desired).

    5. Start the transmission process by setting
       ECON1.TXRTS.
  */

  setregbank(ERXTX_BANK);
  /* Set up the transmit buffer pointer */
  writereg(ETXSTL, TX_BUF_START & 0xff);
  writereg(ETXSTH, TX_BUF_START >> 8);
  writereg(EWRPTL, TX_BUF_START & 0xff);
  writereg(EWRPTH, TX_BUF_START >> 8);

  /* Write the transmission control register as the first byte of the
     output packet. We write 0x00 to indicate that the default
     configuration (the values in MACON3) will be used.  */
  writedatabyte(0x00); /* MACON3 */

  writedata(data, datalen);

  /* Write a pointer to the last data byte. */
  dataend = TX_BUF_START + datalen;
  writereg(ETXNDL, dataend & 0xff);
  writereg(ETXNDH, dataend >> 8);

  /* Clear EIR.TXIF */
  clearregbitfield(EIR, EIR_TXIF);

  /* Don't care about interrupts for now */

  /* Send the packet */
  setregbitfield(ECON1, ECON1_TXRTS);
  while((readreg(ECON1) & ECON1_TXRTS) > 0);

#if DEBUG
  if((readreg(ESTAT) & ESTAT_TXABRT) != 0) {
    uint16_t erdpt;
    uint8_t tsv[7];
    erdpt = (readreg(ERDPTH) << 8) | readreg(ERDPTL);
    writereg(ERDPTL, (dataend + 1) & 0xff);
    writereg(ERDPTH, (dataend + 1) >> 8);
    readdata(tsv, sizeof(tsv));
    writereg(ERDPTL, erdpt & 0xff);
    writereg(ERDPTH, erdpt >> 8);
    PRINTF("enc28j60: tx err: %d: %02x:%02x:%02x:%02x:%02x:%02x\n"
           "                  tsv: %02x%02x%02x%02x%02x%02x%02x\n", datalen,
           0xff & data[0], 0xff & data[1], 0xff & data[2],
           0xff & data[3], 0xff & data[4], 0xff & data[5],
           tsv[6], tsv[5], tsv[4], tsv[3], tsv[2], tsv[1], tsv[0]);
  } else {
    PRINTF("enc28j60: tx: %d: %02x:%02x:%02x:%02x:%02x:%02x\n", datalen,
           0xff & data[0], 0xff & data[1], 0xff & data[2],
           0xff & data[3], 0xff & data[4], 0xff & data[5]);
  }
#endif

  sent_packets++;
  PRINTF("enc28j60: sent_packets %d\n", sent_packets);
  return datalen;
}
/*---------------------------------------------------------------------------*/
int
enc28j60_read(uint8_t *buffer, uint16_t bufsize)
{
  int n, len, next, err, i;

  uint8_t nxtpkt[2];
  uint8_t status[2];
  uint8_t length[2];

  if(!initialized) {
    return -1;
  }

  err = 0;

  setregbank(EPKTCNT_BANK);
  n = readreg(EPKTCNT);

  if(n == 0) {
    return 0;
  }

  PRINTF("%s: EPKTCNT 0x%02x\n", __func__, n);

  setregbank(ERXTX_BANK);
  /* Read the next packet pointer */
  nxtpkt[0] = readdatabyte();
  nxtpkt[1] = readdatabyte();

  PRINTF("enc28j60: nxtpkt 0x%02x%02x\n", nxtpkt[1], nxtpkt[0]);

  length[0] = readdatabyte();
  length[1] = readdatabyte();

  PRINTF("enc28j60: length 0x%02x%02x\n", length[1], length[0]);

  status[0] = readdatabyte();
  status[1] = readdatabyte();

  /* This statement is just to avoid a compiler warning: */
  status[0] = status[0];
  PRINTF("enc28j60: status 0x%02x%02x\n", status[1], status[0]);

  len = (length[1] << 8) + length[0];
  if(bufsize >= len) {
    readdata(buffer, len);
  } else {
    uint16_t i;

    err = 1;

    /* flush rx fifo */
    for(i = 0; i < len; i++) {
      readdatabyte();
    }
  }

  /* Read an additional byte at odd lengths, to avoid FIFO corruption */
  if((len % 2) != 0) {
    readdatabyte();
  }

  /* Errata #14 */
  next = (nxtpkt[1] << 8) + nxtpkt[0];
  if(next == RX_BUF_START) {
    next = RX_BUF_END;
  } else {
    next = next - 1;
  }
  writereg(ERXRDPTL, next & 0xff);
  writereg(ERXRDPTH, next >> 8);

  setregbitfield(ECON2, ECON2_PKTDEC);

  if(err) {
    PRINTF("enc28j60: rx err: flushed %d\n", len);
    return 0;
  }
  PRINTF("enc28j60: rx=%d", len);

  // dump IP packet
#if 1
  for (i = 0; i < len; i++) {
    if (0 == (i % 16))
        PRINTF("\n");

    PRINTF("%02x ", buffer[i]);
  }

PRINTF("\ndest MAC: %02x-%02x-%02x-%02x-%02x-%02x ", buffer[0], buffer[1], buffer[2],
    buffer[3], buffer[4], buffer[5]);

PRINTF("Src MAC: %02x-%02x-%02x-%02x-%02x-%02x\n", buffer[6], buffer[7], buffer[8],
      buffer[9], buffer[10], buffer[11]);

if ((buffer[12] == 0x8) && (buffer[13] == 0)) {
    PRINTF("IPV4 ");
} else 
    PRINTF("IPV6? ");

    PRINTF("Proto: %x ", buffer[23]);

    if (buffer[23] == 0x11) { // UDP
        PRINTF("UDP Src IP: %d.%d.%d.%d ", buffer[26], buffer[27], buffer[28], buffer[29]);
        PRINTF("Dst IP: %d.%d.%d.%d\n", buffer[30], buffer[31], buffer[32], buffer[33]);    
    }

  if (buffer[23] == 0x6) { // TCP
      PRINTF("TCP Src IP: %d.%d.%d.%d ", buffer[26], buffer[27], buffer[28], buffer[29]);
      PRINTF("Dst IP: %d.%d.%d.%d\n", buffer[30], buffer[31], buffer[32], buffer[33]);    
  }

  if (buffer[23] == 0x1) { // ICMP
      PRINTF("ICMP Src IP: %d.%d.%d.%d ", buffer[26], buffer[27], buffer[28], buffer[29]);
      PRINTF("Dst IP: %d.%d.%d.%d\n", buffer[30], buffer[31], buffer[32], buffer[33]);    
  }
#endif

  received_packets++;
  PRINTF("\nenc28j60: received_packets %d\n", received_packets);
  return len;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(enc_watchdog_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  while(1) {
#define RESET_PERIOD (30 * CLOCK_SECOND)
    etimer_set(&et, RESET_PERIOD);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    PRINTF("enc28j60: test received_packet=%d, sent_packets=%d\n", received_packets, sent_packets);
    if ((received_packets > 0) && (received_packets <= sent_packets)) {
      PRINTF("enc28j60: resetting chip\n");
      reset();
    }
    received_packets = 0;
    sent_packets = 0;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
