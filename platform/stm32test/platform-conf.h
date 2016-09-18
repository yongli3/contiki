#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "stm32f10x.h"
#include <stdint.h>

/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */

//#define PLATFORM_HAS_LEDS   1
//
//#ifndef PRINTF_VCP
//#define PRINTF_VCP		1
//#endif
//
#ifndef BV
#define BV(x) (1<<(x))
#endif

#define ENERGEST_TOTAL_IS_RTIMER_T

// the low-level radio driver
#define NETSTACK_CONF_RADIO   cc2520_driver

/*
 * SPI Driver
 */ 
#define READ_PAD(port, pad) GPIO_ReadInputDataBit(port, pad)
//(((port->IDR) >> (pad)) & 1)

#define SET_PAD(port, pad) GPIO_SetBits(port, pad)
//(port->BRR = (1 << pad))

#define CLEAR_PAD(port, pad) GPIO_ResetBits(port, pad)
//(port->BSRR = (1 << pad))

#define CC2520_CONF_SYMBOL_LOOP_COUNT 26050
 
#define SPI_TXBUF SPI1->DR
#define SPI_RXBUF SPI1->DR

#define SPI_WAITFOREOTx() do { while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)); SPI_I2S_ReceiveData(SPI2); } while (0)
#define SPI_WAITFOREORx() do { while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)); } while (0)
#define SPI_WAITFORTxREADY() do {} while (0)

/*
 * Enables/disables CC2520 access to the SPI bus (not the bus).
 * (Chip Select)
 */

/* ENABLE CSn (active low) */
#define CC2520_SPI_ENABLE()    do  {CLEAR_PAD(GPIOA,GPIO_Pin_4); } while (0)

/* DISABLE CSn (active low) */
#define CC2520_SPI_DISABLE()   do  {SET_PAD(GPIOA,GPIO_Pin_4);} while (0) 

//do{SET_PAD(GPIOB, 12);clock_delay(1);}while(0)

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

#endif /* PLATFORM_CONF_H_ */

