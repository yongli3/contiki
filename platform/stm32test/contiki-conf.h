#ifndef CONTIKI_CONF_H_CDBB4VIH3I__
#define CONTIKI_CONF_H_CDBB4VIH3I__

#include <stdint.h>
//#include <gpio.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include <nvic.h>

#define CCIF
#define CLIF

//#define NETSTACK_CONF_WITH_IPV4 1
#define WITH_ASCII 1

// 16-bit must < 0xFFFF 
#define RTIMER_ARCH_SECOND  60000 
// systick tick/second 1000 Hz for systick
#define CLOCK_CONF_SECOND 1000

// rtimer
typedef unsigned short rtimer_clock_t;
//#define RTIMER_CLOCK_LT(a,b)     ((int16_t)((a)-(b)) < 0)

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef   signed char    int8_t;
typedef unsigned char   uint8_t;
typedef   signed short  int16_t;
typedef unsigned short uint16_t;

/* These names are deprecated, use C99 names. */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

/* Radio and 802.15.4 params */
/* 802.15.4 radio channel */
#define RF_CHANNEL						        16
/* 802.15.4 PAN ID */
#define IEEE802154_CONF_PANID					0x1234

#define NETSTACK_CONF_MAC					nullmac_driver
#define NETSTACK_CONF_RDC					nullrdc_driver
#define NETSTACK_CONF_FRAMER			    framer_802154
#define NETSTACK_CONF_RADIO                 cc2520_driver

#ifndef CC2520_CONF_AUTOACK
#define CC2520_CONF_AUTOACK              1
#endif /* CC2520_CONF_AUTOACK */
#define CC2520_CONF_SYMBOL_LOOP_COUNT 26050

// SPI for CC2520
#define READ_PAD(port, pad) GPIO_ReadInputDataBit(port, pad)
#define SET_PAD(port, pad) GPIO_SetBits(port, pad)
#define CLEAR_PAD(port, pad) GPIO_ResetBits(port, pad)
#define CC2520_CONF_SYMBOL_LOOP_COUNT 26050
 
#define SPI_TXBUF SPI1->DR
#define SPI_RXBUF SPI1->DR

// used for SPI read; wait for RXNE, recived data is valid 
#define SPI_WAITFOREORx() do { while(RESET == SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)); } while (0)

// wiat for TX before Before send data, TXE must = 1 = TX empty, data can be sent out 
#define SPI_WAITFORTxREADY() do { while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); } while (0)

// wait for TX ended 
#define SPI_WAITFOREOTx() do { while(RESET == SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)); SPI_I2S_ReceiveData(SPI1); } while (0)


#define CC2520_SPI_ENABLE()    do  {CLEAR_PAD(GPIOA,GPIO_Pin_4); } while (0)
#define CC2520_SPI_DISABLE()   do  {SET_PAD(GPIOA,GPIO_Pin_4);} while (0) 

/* Pin status.CC2520 */
#define CC2520_FIFO_IS_1 (READ_PAD(GPIOC, 0))
#define CC2520_FIFOP_IS_1  (READ_PAD(GPIOC, 1))
#define CC2520_CCA_IS_1   (READ_PAD(GPIOC, 2))
#define CC2520_SFD_IS_1   (READ_PAD(GPIOC, 3))

/* The CC2520 reset pin. */
#define SET_RESET_INACTIVE()  do  {SET_PAD(GPIOC,GPIO_Pin_4); } while (0)
#define SET_RESET_ACTIVE()    do  {CLEAR_PAD(GPIOC,GPIO_Pin_4); } while (0)

/* CC2520 voltage regulator enable pin. */
#define SET_VREG_ACTIVE()   do  {SET_PAD(GPIOA,GPIO_Pin_1); } while (0)
#define SET_VREG_INACTIVE() do  {CLEAR_PAD(GPIOA,GPIO_Pin_1); } while (0)

/* CC2520 rising edge trigger for external interrupt 0 (FIFOP). */
#define CC2520_FIFOP_INT_INIT() cc2520_arch_fifop_int_init()

/* FIFOP on external interrupt C4. */
/* FIFOP on external interrupt C4. */
#define CC2520_ENABLE_FIFOP_INT() cc2520_arch_fifop_int_enable()
#define CC2520_DISABLE_FIFOP_INT() cc2520_arch_fifop_int_disable()
#define CC2520_CLEAR_FIFOP_INT() cc2520_arch_fifop_int_clear()   

// ??
#define splhigh() 0
#define splx(arg)



// enable IPV6
#ifndef NETSTACK_CONF_WITH_IPV6
#define  NETSTACK_CONF_WITH_IPV6    1
#endif

#if NETSTACK_CONF_WITH_IPV6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD			63

#define UIP_CONF_UDP                				1

#define UIP_CONF_ROUTER						1
#define UIP_CONF_ND6_SEND_RA					0

#define UIP_CONF_IPV6_QUEUE_PKT					0
#define UIP_CONF_IPV6_CHECKS					1
#define UIP_CONF_IPV6_REASSEMBLY				0
#define UIP_CONF_IP_FORWARD					0
#define UIP_CONF_BUFFER_SIZE					140
#define UIP_CONF_MAX_CONNECTIONS				4
#define UIP_CONF_MAX_LISTENPORTS				8
#define UIP_CONF_UDP_CONNS					4

#define SICSLOWPAN_CONF_COMPRESSION				SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG					1
#endif /* SICSLOWPAN_CONF_FRAG */
#ifndef SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS			2
#endif /* SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS */
#ifndef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE					2
#endif /* SICSLOWPAN_CONF_MAXAGE */

#else /* NETSTACK_CONF_WITH_IPV6 */

/* Network setup for non-IPv6 (rime). */
#define NETSTACK_CONF_NETWORK					rime_driver

#endif /* NETSTACK_CONF_WITH_IPV6 */


/* uIP configuration */
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING 1
//#define UIP_CONF_BUFFER_SIZE 116

#define UIP_CONF_TCP_FORWARD 1

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define USB_EP1_SIZE 64
#define USB_EP2_SIZE 64

#define RAND_MAX 0x7fff
#endif /* CONTIKI_CONF_H_CDBB4VIH3I__ */
