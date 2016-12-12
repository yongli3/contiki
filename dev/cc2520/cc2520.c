/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
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
 * This file is part of the Contiki operating system.
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include "contiki.h"

#include "dev/spi.h"
#include "dev/cc2520/cc2520.h"
#include "dev/cc2520/cc2520_const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include <string.h>

#ifndef CC2520_CONF_AUTOACK
#define CC2520_CONF_AUTOACK 0
#endif /* CC2520_CONF_AUTOACK */

#define WITH_SEND_CCA 1

#define FOOTER_LEN 2

#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

#include <stdio.h>

#define clock_delay(t) mdelay(t)

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#if DEBUG
#include "dev/leds.h"
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

extern uint8_t sys_seed;
extern uint8_t mac_longaddr[8];
extern uint16_t mac_shortaddr;

static u8 SPI1_ReadWriteByte(u8 TxData)
{		
    // Loop while DR register in not emplty
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
	SPI_I2S_SendData(SPI1, TxData);

    // Wait to receive a byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	  						    
	return SPI_I2S_ReceiveData(SPI1);					    
}

static u8 cc2520_read_reg(u16 reg) 
{
    u8 val;
    
    CC2520_SPI_ENABLE();
    SPI1_ReadWriteByte((CC2520_INS_MEMRD | ((reg >> 8) & 0xff)));
    SPI1_ReadWriteByte((reg & 0xff));
    val = SPI1_ReadWriteByte(0XFF);
    CC2520_SPI_DISABLE();
    return val;
}

static u8 cc2520_write_reg(u16 reg, u8 val) 
{
    u8 temp;
    
    CC2520_SPI_ENABLE();    
    SPI1_ReadWriteByte((CC2520_INS_MEMWR | ((reg >> 8) & 0xff)));
    SPI1_ReadWriteByte(reg & 0xff);
    SPI1_ReadWriteByte(val);
    CC2520_SPI_DISABLE();
    return val;
}

static void cc2520_irq_init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

    // PA0 = WK_UP
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PORTA,PORTC时钟
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

    //PA0 EXT0
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
   	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
 	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
    
    // PC1 EXT1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);    
}

static void cc2520_arch_init(void)
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
    GPIO_SetBits(GPIOA,GPIO_Pin_4);

    // FIFO = input pull down
    // FIFOP = input pull down
    // CCA = input pull down
    // SFD = input pull down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;   //input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;   //input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;   //input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;   //input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC4 = RESET output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //pull up output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);

    // PA1 = VREG_EN output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP  ;   // pull up output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);

    mdelay(200);
    GPIO_SetBits(GPIOC,GPIO_Pin_4);
    mdelay(200);

    //PA4/5/6/7 = SPI1 master 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);	
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

    val = cc2520_read_reg(CC2520_CHIPID);
    printf("\r\nCC2520 ID=0x%x\r\n", val);

    val = cc2520_read_reg(CC2520_CHIPID);
    printf("\r\nCC2520 ID=0x%x\r\n", val);    

    val = cc2520_read_reg(CC2520_VERSION);
    printf("\r\CC2520 nversion=0x%x\r\n", val);    
    

#if 0
  /* all input by default, set these as output */
  CC2520_CSN_PORT(DIR) |= BV(CC2520_CSN_PIN);
  CC2520_VREG_PORT(DIR) |= BV(CC2520_VREG_PIN);
  CC2520_RESET_PORT(DIR) |= BV(CC2520_RESET_PIN);

  CC2520_FIFOP_PORT(DIR) &= ~(BV(CC2520_FIFOP_PIN));
  CC2520_FIFO_PORT(DIR) &= ~(BV(CC2520_FIFO_PIN));
  CC2520_CCA_PORT(DIR) &= ~(BV(CC2520_CCA_PIN));
  CC2520_SFD_PORT(DIR) &= ~(BV(CC2520_SFD_PIN));

#if CONF_SFD_TIMESTAMPS
  cc2520_arch_sfd_init();
#endif
#endif
    CC2520_SPI_DISABLE();                /* Unselect radio. */
    cc2520_irq_init();
}

static void cc2520_arch_fifop_int_init(void) 
{
}

static void cc2520_arch_fifop_int_enable(void) 
{
    NVIC_EnableIRQ(EXTI1_IRQn);
}

static void cc2520_arch_fifop_int_disable(void) 
{
    NVIC_DisableIRQ(EXTI1_IRQn);
}

static void cc2520_arch_fifop_int_clear(void)
{
}

void EXTI0_handler(void) __attribute__ ((interrupt));
void EXTI1_handler(void) __attribute__ ((interrupt));

void EXTI0_handler(void)
{
    //mdelay(10);    //消抖
    printf("EXT0\r\n");
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除EXTI0线路挂起位
}

void EXTI1_handler(void)
{
    printf("EXT1\r\n");
    cc2520_interrupt();
	EXTI_ClearITPendingBit(EXTI_Line1);  //清除EXTI1线路挂起位
}

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t cc2520_time_of_arrival, cc2520_time_of_departure;

int cc2520_authority_level_of_sender;

int cc2520_packets_seen, cc2520_packets_read;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

volatile uint8_t cc2520_sfd_counter;
volatile uint16_t cc2520_sfd_start_time;
volatile uint16_t cc2520_sfd_end_time;

static volatile uint16_t last_packet_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(cc2520_process, "CC2520 driver");
/*---------------------------------------------------------------------------*/


int cc2520_on(void);
int cc2520_off(void);

static int cc2520_read(void *buf, unsigned short bufsize);

static int cc2520_prepare(const void *data, unsigned short len);
static int cc2520_transmit(unsigned short len);
static int cc2520_send(const void *data, unsigned short len);

static int cc2520_receiving_packet(void);
static int pending_packet(void);
static int cc2520_cca(void);
/* static int detected_energy(void); */

signed char cc2520_last_rssi;
uint8_t cc2520_last_correlation;

static uint8_t receive_on;
static int channel;

static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = receive_on ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    *value = cc2520_get_channel();
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MIN:
    *value = 11;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 26;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      cc2520_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      cc2520_off();
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    if(value < 11 || value > 26) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    cc2520_set_channel(value);
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}

const struct radio_driver cc2520_driver =
  {
    cc2520_init,
    cc2520_prepare,
    cc2520_transmit,
    cc2520_send,
    cc2520_read,
    /* cc2520_set_channel, */
    /* detected_energy, */
    cc2520_cca,
    cc2520_receiving_packet,
    pending_packet,
    cc2520_on,
    cc2520_off,
    get_value,
    set_value,
    get_object,
    set_object
  };

/*---------------------------------------------------------------------------*/

static void
getrxdata(void *buf, int len)
{
  CC2520_READ_FIFO_BUF(buf, len);
}
static void
getrxbyte(uint8_t *byte)
{
  CC2520_READ_FIFO_BYTE(*byte);
}
static void
flushrx(void)
{
  uint8_t dummy;

  CC2520_READ_FIFO_BYTE(dummy);
  /* read and discard dummy to avoid "variable set but not used" warning */
  (void)dummy;
  CC2520_STROBE(CC2520_INS_SFLUSHRX);
  CC2520_STROBE(CC2520_INS_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
static void
strobe(uint8_t regname)
{
  CC2520_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
static unsigned int
status(void)
{
  uint8_t status;
  CC2520_GET_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
    PRINTF("on...\n");       
  CC2520_ENABLE_FIFOP_INT();
  strobe(CC2520_INS_SRXON);

  BUSYWAIT_UNTIL(status() & (BV(CC2520_XOSC16M_STABLE)), RTIMER_SECOND / 100);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
static void
off(void)
{
  PRINTF("off...\n");
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2520_TX_ACTIVE)), RTIMER_SECOND / 10);

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  strobe(CC2520_INS_SRFOFF);
  CC2520_DISABLE_FIFOP_INT();

  if(!CC2520_FIFOP_IS_1) {
    flushrx();
  }
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static uint8_t
getreg(uint16_t regname)
{
  uint8_t reg;
  CC2520_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
static void
setreg(uint16_t regname, uint8_t value)
{
  CC2520_WRITE_REG(regname, value);
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  setreg(CC2520_TXPOWER, power);
}
/*---------------------------------------------------------------------------*/
#define AUTOCRC (1 << 6)
#define AUTOACK (1 << 5)
#define FRAME_MAX_VERSION ((1 << 3) | (1 << 2))
#define FRAME_FILTER_ENABLE (1 << 0)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
/*---------------------------------------------------------------------------*/
int
cc2520_init(void)
{
    unsigned char r;
  {
    int s = splhigh();
    cc2520_arch_init();		/* Initalize ports and SPI. */
    CC2520_DISABLE_FIFOP_INT();
    splx(s);
  }

#if 0
  SET_VREG_INACTIVE();
  clock_delay(250);
  /* Turn on voltage regulator and reset. */
  SET_VREG_ACTIVE();
  clock_delay(250);
  SET_RESET_ACTIVE();
  clock_delay(127);
  SET_RESET_INACTIVE();
  clock_delay(125);
#endif
  /* Turn on the crystal oscillator. */
  strobe(CC2520_INS_SXOSCON);
  clock_delay(125);

  BUSYWAIT_UNTIL(status() & (BV(CC2520_XOSC16M_STABLE)), RTIMER_SECOND / 100);

  /* Change default values as recommended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA.*/

  setreg(CC2520_TXCTRL,      0x94);
  setreg(CC2520_TXPOWER,     0x13);    // Output power 1 dBm

  /*

	valeurs de TXPOWER
	  0x03 -> -18 dBm
	  0x2C -> -7 dBm
	  0x88 -> -4 dBm
	  0x81 -> -2 dBm
	  0x32 -> 0 dBm
	  0x13 -> 1 dBm
	  0x32 -> 0 dBm
	  0x13 -> 1 dBm
	  0xAB -> 2 dBm
	  0xF2 -> 3 dBm
	  0xF7 -> 5 dBm
  */
  setreg(CC2520_CCACTRL0,    0xF8);  // CCA treshold -80dBm

  // Recommended RX settings
  setreg(CC2520_MDMCTRL0,    0x84);  // Controls modem
  setreg(CC2520_MDMCTRL1,    0x14);  // Controls modem
  setreg(CC2520_RXCTRL,      0x3F);  // Adjust currents in RX related analog modules
  setreg(CC2520_FSCTRL,      0x5A);  // Adjust currents in synthesizer.
  setreg(CC2520_FSCAL1,      0x2B);  // Adjust currents in VCO
  setreg(CC2520_AGCCTRL1,    0x11);  // Adjust target value for AGC control loop
  setreg(CC2520_AGCCTRL2,    0xEB);

  //  Disable external clock
  setreg(CC2520_EXTCLOCK,    0x00);

  //  Tune ADC performance
  setreg(CC2520_ADCTEST0,    0x10);
  setreg(CC2520_ADCTEST1,    0x0E);
  setreg(CC2520_ADCTEST2,    0x03);

  /* Set auto CRC on frame. */
#if CC2520_CONF_AUTOACK
  setreg(CC2520_FRMCTRL0,    AUTOCRC | AUTOACK);
  setreg(CC2520_FRMFILT0,    FRAME_MAX_VERSION|FRAME_FILTER_ENABLE);
#else
  /* setreg(CC2520_FRMCTRL0,    0x60); */
  setreg(CC2520_FRMCTRL0,    AUTOCRC);
  /* Disable filter on @ (remove if you want to address specific wismote) */
  setreg(CC2520_FRMFILT0,    0x00);
#endif /* CC2520_CONF_AUTOACK */
  /* SET_RXENMASK_ON_TX */
  setreg(CC2520_FRMCTRL1,          1);
  /* Set FIFOP threshold to maximum .*/
  setreg(CC2520_FIFOPCTRL,   FIFOP_THR(0x7F));

  CC2520_GET_RANDOM(r); 
  printf("%s %d-%d-%d-%d\n", __func__, clock_time(), RTIMER_NOW(), cc2520_rssi(), r);

  CC2520_GET_RANDOM(r);  
  printf("%s %d-%d-%d-%d\n", __func__, clock_time(), RTIMER_NOW(), cc2520_rssi(), r);

    sys_seed = r;
    random_init(sys_seed);  

    mac_longaddr[7] = random_rand() & 0xFF;
    mac_longaddr[6] = random_rand() > 8;

  cc2520_set_pan_addr(IEEE802154_PANID, mac_shortaddr, mac_longaddr);
  cc2520_set_channel(RF_CHANNEL);

  flushrx();

  process_start(&cc2520_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2520_transmit(unsigned short payload_len)
{
  int i, txpower;

  GET_LOCK();

  txpower = 0;
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Remember the current transmission power */
    txpower = cc2520_get_txpower();
    /* Set the specified transmission power */
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  }

  /* The TX FIFO can only hold one packet. Make sure to not overrun
   * FIFO by waiting for transmission to start here and synchronizing
   * with the CC2520_TX_ACTIVE check in cc2520_send.
   *
   * Note that we may have to wait up to 320 us (20 symbols) before
   * transmission starts.
   */
#ifndef CC2520_CONF_SYMBOL_LOOP_COUNT
#error CC2520_CONF_SYMBOL_LOOP_COUNT needs to be set!!!
#else
#define LOOP_20_SYMBOLS CC2520_CONF_SYMBOL_LOOP_COUNT
#endif

#if WITH_SEND_CCA
  strobe(CC2520_INS_SRXON);
  BUSYWAIT_UNTIL(status() & BV(CC2520_RSSI_VALID) , RTIMER_SECOND / 10);
  strobe(CC2520_INS_STXONCCA);
#else /* WITH_SEND_CCA */
  strobe(CC2520_INS_STXON);
#endif /* WITH_SEND_CCA */
  for(i = LOOP_20_SYMBOLS; i > 0; i--) {
    if(CC2520_SFD_IS_1) {
#if PACKETBUF_WITH_PACKET_TYPE
      {
        rtimer_clock_t sfd_timestamp;
        sfd_timestamp = cc2520_sfd_start_time;
        if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
           PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP) {
          /* Write timestamp to last two bytes of packet in TXFIFO. */
          CC2520_WRITE_RAM(&sfd_timestamp, CC2520RAM_TXFIFO + payload_len - 1, 2);
        }
      }
#endif /* PACKETBUF_WITH_PACKET_TYPE */

      if(!(status() & BV(CC2520_TX_ACTIVE))) {
        /* SFD went high but we are not transmitting. This means that
           we just started receiving a packet, so we drop the
           transmission. */
        RELEASE_LOCK();
        return RADIO_TX_COLLISION;
      }
      if(receive_on) {
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
      }
      ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
      /* We wait until transmission has ended so that we get an
	 accurate measurement of the transmission time.*/
     //BUSYWAIT_UNTIL(getreg(CC2520_EXCFLAG0) & TX_FRM_DONE , RTIMER_SECOND / 100);
      BUSYWAIT_UNTIL(!(status() & BV(CC2520_TX_ACTIVE)), RTIMER_SECOND / 10);

#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
      ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,cc2520_get_txpower());
#endif
      ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
      if(receive_on) {
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
      } else {
	/* We need to explicitly turn off the radio,
	 * since STXON[CCA] -> TX_ACTIVE -> RX_ACTIVE */
	off();
      }

      if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Restore the transmission power */
        set_txpower(txpower & 0xff);
      }

      RELEASE_LOCK();

      return RADIO_TX_OK;
    }
  }

  /* If we are using WITH_SEND_CCA, we get here if the packet wasn't
     transmitted because of other channel activity. */
  RIMESTATS_ADD(contentiondrop);
  PRINTF("cc2520: do_send() transmission never started\n");

  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Restore the transmission power */
    set_txpower(txpower & 0xff);
  }

  RELEASE_LOCK();
  return RADIO_TX_COLLISION;
}
/*---------------------------------------------------------------------------*/
static int
cc2520_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
  GET_LOCK();

  PRINTF("##cc2520_sending %d bytes\n", payload_len);
  /*int i;
  for(i = 0; i < payload_len;i++)
	  printf("%x",((uint8_t *) payload)[i]);
  printf("\n");*/
  RIMESTATS_ADD(lltx);

  /* Wait for any previous transmission to finish. */
  /*  while(status() & BV(CC2520_TX_ACTIVE));*/

  /* Write packet to TX FIFO. */
  strobe(CC2520_INS_SFLUSHTX);

  total_len = payload_len + FOOTER_LEN;
  CC2520_WRITE_FIFO_BUF(&total_len, 1);
  CC2520_WRITE_FIFO_BUF(payload, payload_len);

  RELEASE_LOCK();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2520_send(const void *payload, unsigned short payload_len)
{
  cc2520_prepare(payload, payload_len);
  return cc2520_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2520_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\n", locked);*/
    lock_off = 1;
    return 1;
  }

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if(status() & BV(CC2520_TX_ACTIVE)) {
    lock_off = 1;
  } else {
    off();
  }
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2520_on(void)
{
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  on();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2520_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
cc2520_set_channel(int c)
{
  uint16_t f;

  GET_LOCK();
  /*
   * Subtract the base channel (11), multiply by 5, which is the
   * channel spacing. 357 is 2405-2048 and 0x4000 is LOCK_THR = 1.
   */
  channel = c;

  f = MIN_CHANNEL + ((channel - MIN_CHANNEL) * CHANNEL_SPACING);
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL((status() & (BV(CC2520_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  /* Wait for any transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2520_TX_ACTIVE)), RTIMER_SECOND / 10);

  /* Define radio channel (between 11 and 25) */
  setreg(CC2520_FREQCTRL, f);

  /* If we are in receive mode, we issue an SRXON command to ensure
     that the VCO is calibrated. */
  if(receive_on) {
    strobe(CC2520_INS_SRXON);
  }

  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2520_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  uint8_t tmp[2];

  GET_LOCK();

  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL(status() & (BV(CC2520_XOSC16M_STABLE)), RTIMER_SECOND / 10);

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  CC2520_WRITE_RAM(&tmp, CC2520RAM_PANID, 2);


  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  CC2520_WRITE_RAM(&tmp, CC2520RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    int f;
    uint8_t tmp_addr[8];
    // LSB first, MSB last for 802.15.4 addresses in CC2520
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    CC2520_WRITE_RAM(tmp_addr, CC2520RAM_IEEEADDR, 8);
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
int
cc2520_interrupt(void)
{
  CC2520_CLEAR_FIFOP_INT();
  process_poll(&cc2520_process);

  last_packet_timestamp = cc2520_sfd_start_time;
  cc2520_packets_seen++;
  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2520_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("cc2520_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("**cc2520_receiver");

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = cc2520_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    packetbuf_set_datalen(len);

    if (len > FOOTER_LEN)
        NETSTACK_RDC.input();
    else 
        flushrx();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2520_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len;

  printf("+%s\n", __func__);

  if(!CC2520_FIFOP_IS_1) {
    return 0;
  }

  GET_LOCK();

  cc2520_packets_read++;

  getrxbyte(&len);

  //printf("rx-len=%d\n", len);  
  if(len > CC2520_MAX_PACKET_LEN) {
    /* Oops, we must be out of sync. */
    flushrx();
    RIMESTATS_ADD(badsynch);
    RELEASE_LOCK();
    return 0;
  }

  if(len <= FOOTER_LEN) {
    flushrx();
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }

  if(len - FOOTER_LEN > bufsize) {
    flushrx();
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }

  getrxdata(buf, len - FOOTER_LEN);
  getrxdata(footer, FOOTER_LEN);

  if(footer[1] & FOOTER1_CRC_OK) {
    cc2520_last_rssi = footer[0];
    cc2520_last_correlation = footer[1] & FOOTER1_CORRELATION;


    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2520_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2520_last_correlation);

    RIMESTATS_ADD(llrx);

  } else {
    RIMESTATS_ADD(badcrc);
    len = FOOTER_LEN;
  }

  if(CC2520_FIFOP_IS_1) {
    if(!CC2520_FIFO_IS_1) {
      /* Clean up in case of FIFO overflow!  This happens for every
       * full length frame and is signaled by FIFOP = 1 and FIFO =
       * 0. */
      flushrx();
    } else {
      /* Another packet has been received and needs attention. */
      process_poll(&cc2520_process);
    }
  }

  RELEASE_LOCK();

  if(len < FOOTER_LEN) {
    return 0;
  }

  return len - FOOTER_LEN;
}
/*---------------------------------------------------------------------------*/
void
cc2520_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
cc2520_get_txpower(void)
{
  uint8_t power;
  GET_LOCK();
  power = getreg(CC2520_TXPOWER);
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
int
cc2520_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }

  GET_LOCK();

  if(!receive_on) {
    radio_was_off = 1;
    cc2520_on();
  }
  BUSYWAIT_UNTIL(status() & BV(CC2520_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)getreg(CC2520_RSSI));

  if(radio_was_off) {
    cc2520_off();
  }
  RELEASE_LOCK();
  return rssi;
}
/*---------------------------------------------------------------------------*/
/*
static int
detected_energy(void)
{
  return cc2520_rssi();
}
*/
/*---------------------------------------------------------------------------*/
int
cc2520_cca_valid(void)
{
  int valid;
  if(locked) {
    return 1;
  }
  GET_LOCK();
  valid = !!(status() & BV(CC2520_RSSI_VALID));
  RELEASE_LOCK();
  return valid;
}
/*---------------------------------------------------------------------------*/
static int
cc2520_cca(void)
{
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  if(!receive_on) {
    radio_was_off = 1;
    cc2520_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    RELEASE_LOCK();
    if(radio_was_off) {
      cc2520_off();
    }
    return 1;
  }

  BUSYWAIT_UNTIL(status() & BV(CC2520_RSSI_VALID), RTIMER_SECOND / 100);

  cca = CC2520_CCA_IS_1;

  if(radio_was_off) {
    cc2520_off();
  }
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2520_receiving_packet(void)
{
  return CC2520_SFD_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return CC2520_FIFOP_IS_1;
}
/*---------------------------------------------------------------------------*/
void
cc2520_set_cca_threshold(int value)
{
  GET_LOCK();
  setreg(CC2520_CCACTRL0, value & 0xff);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
