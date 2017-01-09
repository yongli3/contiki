/*
 * File      : dm9000a.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-07-01     Bernard      the first version
 */
#include "dm9000a.h"
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_fsmc.h>
#include <stdio.h>

void EXTI15_10_IRQHandler(void) __attribute__ ((interrupt));

#define DM9000_DEBUG		1
#if DM9000_DEBUG
#define DM9000_TRACE	printf
#else
#define DM9000_TRACE(...)
#endif

typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_uint32_t;

/*
 * DM9000 interrupt line is connected to PE4
 */
//--------------------------------------------------------

unsigned char rx_buffer[DM9000_PKT_MAX];
#define DM9000_PHY          0x40    /* PHY address 0x01 */
#define RST_1()             GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define RST_0()             GPIO_ResetBits(GPIOE,GPIO_Pin_5)

#define MAX_ADDR_LEN 6
enum DM9000_PHY_mode
{
    DM9000_10MHD = 0, DM9000_100MHD = 1,
    DM9000_10MFD = 4, DM9000_100MFD = 5,
    DM9000_AUTO  = 8, DM9000_1M_HPNA = 0x10
};

enum DM9000_TYPE
{
    TYPE_DM9000E,
    TYPE_DM9000A,
    TYPE_DM9000B
};

struct rt_dm9000_eth
{
    /* inherit from ethernet device */
    //struct eth_device parent;

    enum DM9000_TYPE type;
	enum DM9000_PHY_mode mode;

    rt_uint8_t imr_all;

    rt_uint8_t packet_cnt;                  /* packet I or II */
    rt_uint16_t queue_packet_len;           /* queued packet (packet II) */

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];		/* hw address	*/
};
static struct rt_dm9000_eth dm9000_device;
static struct rt_semaphore sem_ack, sem_lock;

void rt_dm9000_isr(void);

static void delay_ms(rt_uint32_t ms)
{
   volatile rt_uint32_t len;
    for (;ms > 0; ms --)
        for (len = 0; len < 10; len++ );
}

static void loop_delay(rt_uint32_t count)
{
   volatile rt_uint32_t len;
    for (len = 0; len < count; len++);
}

/* Read a byte from I/O port */
 rt_uint8_t dm9000_io_read(rt_uint16_t reg)
{
    unsigned short temp = 0;
    DM9000_IO = reg;
    //loop_delay(1);
    temp = (rt_uint8_t) DM9000_DATA;
    return temp;
}

/* Write a byte to I/O port */
void dm9000_io_write(rt_uint16_t reg, rt_uint16_t value)
{
    DM9000_IO = reg;
    //loop_delay(1);
    DM9000_DATA = value;
}

/* Read a word from phyxcer */
 rt_uint16_t phy_read(rt_uint16_t reg)
{
    rt_uint16_t val;

    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);
    dm9000_io_write(DM9000_EPCR, 0xc);	/* Issue phyxcer read command */

    delay_ms(100);		/* Wait read complete */

    dm9000_io_write(DM9000_EPCR, 0x0);	/* Clear phyxcer read command */
    val = (dm9000_io_read(DM9000_EPDRH) << 8) | dm9000_io_read(DM9000_EPDRL);

    return val;
}

/* Write a word to phyxcer */
 void phy_write(rt_uint16_t reg, rt_uint16_t value)
{
    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);

    /* Fill the written data into REG_0D & REG_0E */
    dm9000_io_write(DM9000_EPDRL, (value & 0xff));
    dm9000_io_write(DM9000_EPDRH, ((value >> 8) & 0xff));
    dm9000_io_write(DM9000_EPCR, 0xa);	/* Issue phyxcer write command */

    delay_ms(500);		/* Wait write complete */

    dm9000_io_write(DM9000_EPCR, 0x0);	/* Clear phyxcer write command */
}

/* Set PHY operationg mode */
 void phy_mode_set(rt_uint32_t media_mode)
{
    rt_uint16_t phy_reg4 = 0x01e1, phy_reg0 = 0x1000;
    if (!(media_mode & DM9000_AUTO))
    {
        switch (media_mode)
        {
        case DM9000_10MHD:
            phy_reg4 = 0x21;
            phy_reg0 = 0x0000;
            break;
        case DM9000_10MFD:
            phy_reg4 = 0x41;
            phy_reg0 = 0x1100;
            break;
        case DM9000_100MHD:
            phy_reg4 = 0x81;
            phy_reg0 = 0x2000;
            break;
        case DM9000_100MFD:
            phy_reg4 = 0x101;
            phy_reg0 = 0x3100;
            break;
        }
        phy_write(4, phy_reg4);	/* Set PHY media mode */
        phy_write(0, phy_reg0);	/*  Tmp */
    }

    dm9000_io_write(DM9000_GPCR, 0x01);	/* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0x00);	/* Enable PHY */
}

int rt_dm9000_init()
{
    int i, oft, lnk;
    rt_uint32_t value;
    printf("+%s\n", __func__);
    /* RESET device */
    dm9000_io_write(DM9000_NCR, NCR_RST);
    mdelay(5);		/* delay 1ms */

#if 1
 	dm9000_io_write(DM9000_GPCR,0x01);			//第一步:设置GPCR寄存器(0X1E)的bit0为1 
	dm9000_io_write(DM9000_GPR,0);				//第二步:设置GPR寄存器(0X1F)的bit1为0，DM9000内部的PHY上电

    dm9000_io_write(DM9000_NCR,(0x02|NCR_RST));	//第三步:软件复位DM9000 
	do 
	{
		mdelay(5); 	
	}while(dm9000_io_read(DM9000_NCR)&1);		//等待DM9000软复位完成
	
	dm9000_io_write(DM9000_NCR,0);
	dm9000_io_write(DM9000_NCR,(0x02|NCR_RST));	//DM9000第二次软复位
	do 
	{
		mdelay(5);	
	}while (dm9000_io_read(DM9000_NCR)&1);
#endif

    /* identfy DM9000 */
    value  = dm9000_io_read(DM9000_VIDL);
    value |= dm9000_io_read(DM9000_VIDH) << 8;
    value |= dm9000_io_read(DM9000_PIDL) << 16;
    value |= dm9000_io_read(DM9000_PIDH) << 24;
    if (value == DM9000_ID)
    {
        printf("dm9000 id: 0x%08x\n", value);
    }
    else
    {
        printf("DM9000 ID eror! 0x%08x\n", value);
        return -1;
    }

    /* GPIO0 on pre-activate PHY */
    dm9000_io_write(DM9000_GPR, 0x00);	            /* REG_1F bit0 activate phyxcer */
    dm9000_io_write(DM9000_GPCR, GPCR_GEP_CNTL);    /* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0x00);                 /* Enable PHY */

    /* Set PHY */
    phy_mode_set(dm9000_device.mode);

    /* Program operating register */
    dm9000_io_write(DM9000_NCR, 0x0);	/* only intern phy supported by now */
    dm9000_io_write(DM9000_TCR, 0);	    /* TX Polling clear */
    dm9000_io_write(DM9000_BPTR, 0x3f);	/* Less 3Kb, 200us */
    dm9000_io_write(DM9000_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8));	/* Flow Control : High/Low Water */
    dm9000_io_write(DM9000_FCR, 0x0);	/* SH FIXME: This looks strange! Flow Control */
    dm9000_io_write(DM9000_SMCR, 0);	/* Special Mode */
    dm9000_io_write(DM9000_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);	/* clear TX status */
    dm9000_io_write(DM9000_ISR, 0x0f);	/* Clear interrupt status */
    dm9000_io_write(DM9000_TCR2, 0x80);	/* Switch LED to mode 1 */

    /* set mac address */
    for (i = 0, oft = 0x10; i < 6; i++, oft++)
        dm9000_io_write(oft, dm9000_device.dev_addr[i]);
    /* set multicast address */
    for (i = 0, oft = 0x16; i < 8; i++, oft++)
        dm9000_io_write(oft, 0xff);

    /* Activate DM9000 */
    dm9000_io_write(DM9000_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_PRMSC | RCR_RXEN);	/* RX enable */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

	if (dm9000_device.mode == DM9000_AUTO)
	{
	    while (!(phy_read(1) & 0x20))
	    {
	        mdelay(1);
	        /* autonegation complete bit */
	        //rt_thread_delay(10);
	        i++;
	        if (i == 10000)
	        {
	            printf("could not establish link\n");
	            return 0;
	        }
	    }
	}

    /* see what we've got */
    lnk = phy_read(17) >> 12;
    printf("operating at ");
    switch (lnk)
    {
    case 1:
        printf("10M half duplex ");
        break;
    case 2:
        printf("10M full duplex ");
        break;
    case 4:
        printf("100M half duplex ");
        break;
    case 8:
        printf("100M full duplex ");
        break;
    default:
        printf("unknown: %d ", lnk);
        break;
    }
    printf("mode\n");

    dm9000_io_write(DM9000_IMR, dm9000_device.imr_all);	/* Enable TX/RX interrupt mask */

    return 0;
}

/* interrupt service routine */
void rt_dm9000_isr()
{
    rt_uint16_t int_status;
    rt_uint16_t last_io;

    last_io = DM9000_IO;

    /* Disable all interrupts */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Got DM9000 interrupt status */
    int_status = dm9000_io_read(DM9000_ISR);               /* Got ISR */
    dm9000_io_write(DM9000_ISR, int_status);    /* Clear ISR status */

	//DM9000_TRACE("dm9000 isr: int status %04x\n", int_status);

    /* receive overflow */
    if (int_status & ISR_ROS)
    {
        printf("overflow\n");
    }

    if (int_status & ISR_ROOS)
    {
        printf("overflow counter overflow\n");
    }

    /* Received the coming packet */
    if (int_status & ISR_PRS)
    {
	    /* disable receive interrupt */
	    dm9000_device.imr_all = IMR_PAR | IMR_PTM;

        /* a frame has been received */
        //eth_device_ready(&(dm9000_device.parent));
    }

    /* Transmit Interrupt check */
    if (int_status & ISR_PTS)
    {
        /* transmit done */
        int tx_status = dm9000_io_read(DM9000_NSR);    /* Got TX status */

        if (tx_status & (NSR_TX2END | NSR_TX1END))
        {
            dm9000_device.packet_cnt --;
            if (dm9000_device.packet_cnt > 0)
            {
            	//DM9000_TRACE("dm9000 isr: tx second packet\n");

                /* transmit packet II */
                /* Set TX length to DM9000 */
                dm9000_io_write(DM9000_TXPLL, dm9000_device.queue_packet_len & 0xff);
                dm9000_io_write(DM9000_TXPLH, (dm9000_device.queue_packet_len >> 8) & 0xff);

                /* Issue TX polling command */
                dm9000_io_write(DM9000_TCR, TCR_TXREQ);	/* Cleared after TX complete */
            }

            /* One packet sent complete */
            //rt_sem_release(&sem_ack);
        }
    }

    /* Re-enable interrupt mask */
    dm9000_io_write(DM9000_IMR, dm9000_device.imr_all);

    DM9000_IO = last_io;
}

#if 0
/* RT-Thread Device Interface */
/* initialize the interface */

static rt_err_t rt_dm9000_open(rt_device_t dev, rt_uint16_t oflag)
{
    return 0;
}

static rt_err_t rt_dm9000_close(rt_device_t dev)
{
    /* RESET devie */
    phy_write(0, 0x8000);	/* PHY RESET */
    dm9000_io_write(DM9000_GPR, 0x01);	/* Power-Down PHY */
    dm9000_io_write(DM9000_IMR, 0x80);	/* Disable all interrupt */
    dm9000_io_write(DM9000_RCR, 0x00);	/* Disable RX */

    return 0;
}

static rt_size_t rt_dm9000_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_dm9000_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_dm9000_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, dm9000_device.dev_addr, 6);
        else return -1;
        break;

    default :
        break;
    }

    return 0;
}

/* ethernet device interface */
/* transmit packet. */
int rt_dm9000_tx( rt_device_t dev, struct pbuf* p)
{
	DM9000_TRACE("dm9000 tx: %d\n", p->tot_len);

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Move data to DM9000 TX RAM */
    DM9000_outb(DM9000_IO_BASE, DM9000_MWCMD);

    {
		/* q traverses through linked list of pbuf's
		 * This list MUST consist of a single packet ONLY */
		struct pbuf *q;
		rt_uint16_t pbuf_index = 0;
		rt_uint8_t word[2], word_index = 0;

		q = p;
		/* Write data into dm9000a, two bytes at a time
		 * Handling pbuf's with odd number of bytes correctly
		 * No attempt to optimize for speed has been made */
		while (q)
		{
			if (pbuf_index < q->len)
			{
				word[word_index++] = ((u8_t*)q->payload)[pbuf_index++];
				if (word_index == 2)
				{
				    DM9000_outw(DM9000_DATA_BASE, (word[1] << 8) | word[0]);
					word_index = 0;
				}
			}
			else
			{
				q = q->next;
				pbuf_index = 0;
			}
		}
		/* One byte could still be unsent */
		if (word_index == 1)
		{
		    DM9000_outw(DM9000_DATA_BASE, word[0]);
		}
    }

    if (dm9000_device.packet_cnt == 0)
    {
    	DM9000_TRACE("dm9000 tx: first packet\n");

        dm9000_device.packet_cnt ++;
        /* Set TX length to DM9000 */
        dm9000_io_write(DM9000_TXPLL, p->tot_len & 0xff);
        dm9000_io_write(DM9000_TXPLH, (p->tot_len >> 8) & 0xff);

        /* Issue TX polling command */
        dm9000_io_write(DM9000_TCR, TCR_TXREQ);	/* Cleared after TX complete */
    }
    else
    {
    	DM9000_TRACE("dm9000 tx: second packet\n");

        dm9000_device.packet_cnt ++;
        dm9000_device.queue_packet_len = p->tot_len;
    }

    /* enable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, dm9000_device.imr_all);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    /* wait ack */
    rt_sem_take(&sem_ack, RT_WAITING_FOREVER);

	DM9000_TRACE("dm9000 tx done\n");

    return RT_EOK;
}
#endif

/* reception packet. */
int rt_dm9000_rx()
{
    unsigned char *p = NULL;
    unsigned int rxbyte;
    int len;
    int i;

    /* Check packet ready or not */
    dm9000_io_read(DM9000_MRCMDX);	    		/* Dummy read */
    rxbyte = DM9000_inb(DM9000_DATA_BASE);		/* Got most updated data */
    if (rxbyte)
    {
        rt_uint16_t rx_status, rx_len;
        rt_uint16_t* data;

        if (rxbyte > 1)
        {
			printf("dm9000 rx: rx error, stop device\n");
            return -1;
            //dm9000_io_write(DM9000_RCR, 0x00);	/* Stop Device */
            //dm9000_io_write(DM9000_ISR, 0x80);	/* Stop INT request */
        }

        /* A packet ready now  & Get status/length */
        DM9000_outb(DM9000_IO_BASE, DM9000_MRCMD);

        rx_status = DM9000_inw(DM9000_DATA_BASE);
        rx_len = DM9000_inw(DM9000_DATA_BASE);
        len = rx_len;
		DM9000_TRACE("dm9000 rx: status %08x len %d\n", rx_status, rx_len);
        if ((rx_status & 0xbf00) || (rx_len < 0x40)
                || (rx_len > DM9000_PKT_MAX))
        {
			printf("rx error: status %04x\n", rx_status);

            if (rx_status & 0x100)
            {
                printf("rx fifo error\n");
            }
            if (rx_status & 0x200)
            {
                printf("rx crc error\n");
            }
            if (rx_status & 0x8000)
            {
                printf("rx length error\n");
            }
            if (rx_len > DM9000_PKT_MAX)
            {
                printf("rx length too big\n");

                /* RESET device */
                dm9000_io_write(DM9000_NCR, NCR_RST);
                mdelay(5);
               // rt_thread_delay(1); /* delay 5ms */
            }
            return -1;
        }

        /* allocate buffer */
        p = rx_buffer;
        data = p;
        while (len > 0)
        {
            *data = DM9000_inw(DM9000_DATA_BASE);
            data ++;
            len -= 2;
        }

        // dump RX
        for (i = 0; i < rx_len; i++) {
            if (0 == (i % 16))
                printf("\n");
            
            printf("%02x ", rx_buffer[i]);
        }
        printf("\n");
    }
    else
    {
        /* restore receive interrupt */
	    dm9000_device.imr_all = IMR_PAR | IMR_PTM | IMR_PRM;
        dm9000_io_write(DM9000_IMR, dm9000_device.imr_all);
    }

    return p;
}

static void RCC_Configuration(void)
{
    /* enable gpiob port clock */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	/* enable FSMC clock */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the EXTI4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void) {
	u16 int_status;
	u16 last_io;

    /* Make sure that interrupt flag is set */
    printf("EXTI15_10 ");
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
        /* Do your stuff when PB12 is changed */

    	last_io = DM9000_IO;
        // disable IRQ
        dm9000_io_write(DM9000_IMR, IMR_PAR);
        int_status=dm9000_io_read(DM9000_ISR); 
    	dm9000_io_write(DM9000_ISR, int_status);				
        //清除中断标志位，DM9000的ISR寄存器的bit0~bit5写1清零
        printf("%x\n", int_status);
        if(int_status & ISR_ROS)printf("overflow \r\n");
        if(int_status & ISR_ROOS)printf("overflow counter overflow \r\n");	
    	if(int_status & ISR_PRS) {		//接收中断
            rt_dm9000_rx();
    	} 
    	if(int_status&ISR_PTS) {			//发送中断
    	 
    		//发送完成中断，用户自行添加所需代码
    	}
        /* Re-enable interrupt mask */
        dm9000_io_write(DM9000_IMR, dm9000_device.imr_all);
 	    DM9000_IO = last_io;        

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

// config IRQ PD13
static void Configure_PD13(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);
    
    /* PB12 is connected to EXTI_Line12 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line13;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
    EXTI_ClearITPendingBit(EXTI_Line13);

    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}


static void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    Configure_PD13();

#if 0
    /* configure PE5 as eth RST */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);
    GPIO_SetBits(GPIOE,GPIO_Pin_5);
    //RST_1();

    /* configure PE4 as external interrupt */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* Connect DM9000 EXTI Line to GPIOE Pin 4 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);

    /* Configure DM9000 EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Clear DM9000A EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);
#endif    
}

static void FSMC_Configuration()
{
	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef p;

    /* FSMC GPIO configure */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
        RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

   /* GPIOD configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_4  | GPIO_Pin_5  | 
                                GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* GPIOE configuration */
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource2 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource6 , GPIO_AF_FSMC);
  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 |
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

  GPIO_Init(GPIOE, &GPIO_InitStructure);
    }
    /* FSMC GPIO configure */

    memset(&p, 0, sizeof(p));
	/*-- FSMC Configuration ------------------------------------------------------*/
	p.FSMC_AddressSetupTime = 9;
	p.FSMC_AddressHoldTime = 0;
	p.FSMC_DataSetupTime = 9;
	p.FSMC_BusTurnAroundDuration = 0;
	p.FSMC_CLKDivision = 1;
	p.FSMC_DataLatency = 9;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/* Enable FSMC Bank1_SRAM Bank4 */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

void rt_hw_dm9000_init()
{
    unsigned int value = 0;
    
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
	FSMC_Configuration();

    dm9000_device.type  = TYPE_DM9000A;
	dm9000_device.mode	= DM9000_AUTO;
	dm9000_device.packet_cnt = 0;
	dm9000_device.queue_packet_len = 0;

    /*
     * SRAM Tx/Rx pointer automatically return to start address,
     * Packet Transmitted, Packet Received
     */
    dm9000_device.imr_all = IMR_PAR | IMR_PTM | IMR_PRM;

    dm9000_device.dev_addr[0] = 0x00;
    dm9000_device.dev_addr[1] = 0x60;
    dm9000_device.dev_addr[2] = 0x6E;
    dm9000_device.dev_addr[3] = 0x11;
    dm9000_device.dev_addr[4] = 0x22;
    dm9000_device.dev_addr[5] = 0x33;

    /* RESET device */
    dm9000_io_write(DM9000_NCR, NCR_RST);
    delay_ms(1000);		/* delay 1ms */

    //while (1) 
{
    /* identfy DM9000 */
    value  = dm9000_io_read(DM9000_VIDL);
    value |= dm9000_io_read(DM9000_VIDH) << 8;
    value |= dm9000_io_read(DM9000_PIDL) << 16;
    value |= dm9000_io_read(DM9000_PIDH) << 24;
    //if (value == DM9000_ID) // 0x90000A46
    {
        printf("dm9000 id: 0x%08x\n", value);
    }
}
    rt_dm9000_init();
}

void dm9000_dump(void)
{
    printf("\n");
    printf("NCR   (0x00): %02x\n", dm9000_io_read(DM9000_NCR));
    printf("NSR   (0x01): %02x\n", dm9000_io_read(DM9000_NSR));
    printf("TCR   (0x02): %02x\n", dm9000_io_read(DM9000_TCR));
    printf("TSRI  (0x03): %02x\n", dm9000_io_read(DM9000_TSR1));
    printf("TSRII (0x04): %02x\n", dm9000_io_read(DM9000_TSR2));
    printf("RCR   (0x05): %02x\n", dm9000_io_read(DM9000_RCR));
    printf("RSR   (0x06): %02x\n", dm9000_io_read(DM9000_RSR));
    printf("ORCR  (0x07): %02x\n", dm9000_io_read(DM9000_ROCR));
    printf("CRR   (0x2C): %02x\n", dm9000_io_read(DM9000_CHIPR));
    printf("CSCR  (0x31): %02x\n", dm9000_io_read(DM9000_CSCR));
    printf("RCSSR (0x32): %02x\n", dm9000_io_read(DM9000_RCSSR));
    printf("ISR   (0xFE): %02x\n", dm9000_io_read(DM9000_ISR));
    printf("IMR   (0xFF): %02x\n", dm9000_io_read(DM9000_IMR));
    printf("\n");
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(dm9000, dm9000 register dump);
#endif
