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
 */

not used 

#include "contiki.h"
#include "contiki-net.h"

#include "dev/spi.h"
#include "dev/cc2520/cc2520.h"

#ifdef CC2520_CONF_SFD_TIMESTAMPS
#define CONF_SFD_TIMESTAMPS CC2520_CONF_SFD_TIMESTAMPS
#endif /* CC2520_CONF_SFD_TIMESTAMPS */

#ifndef CONF_SFD_TIMESTAMPS
#define CONF_SFD_TIMESTAMPS 0
#endif /* CONF_SFD_TIMESTAMPS */

#if 0
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
#endif

void EXTI0_handler(void) __attribute__ ((interrupt));
void EXTI1_handler(void) __attribute__ ((interrupt));

void EXTI0_handler(void)
{
    printf("EXT0\r\n");
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除EXTI0线路挂起位
}

void EXTI1_handler(void)
{
    printf("EXT1\r\n");
    cc2520_interrupt();
	EXTI_ClearITPendingBit(EXTI_Line1);  //清除EXTI1线路挂起位
}

static void cc2520_irq_init()
{
          EXTI_InitTypeDef EXTI_InitStructure;
          //NVIC_InitTypeDef NVIC_InitStructure;
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
          EXTI_Init(&EXTI_InitStructure);     //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
          //NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            //使能按键所在的外部中断通道
          //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;    //抢占优先级2 
          //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;                   //子优先级1
          //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
          //NVIC_Init(&NVIC_InitStructure);       //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

          NVIC_SetPriority(EXTI0_IRQn, 2);
          NVIC_ENABLE_INT(EXTI0_IRQn);

          
          // PC1 EXT1
          GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
          EXTI_InitStructure.EXTI_Line=EXTI_Line1;
          EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
          EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//
          EXTI_InitStructure.EXTI_LineCmd = ENABLE;
          EXTI_Init(&EXTI_InitStructure);     //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
          //NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;            //使能按键所在的外部中断通道
          //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;    //抢占优先级2， 
          //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;                   //子优先级1
          //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
          //NVIC_Init(&NVIC_InitStructure); 

          NVIC_SetPriority(EXTI1_IRQn, 2);
          NVIC_ENABLE_INT(EXTI1_IRQn);
          
}

void
cc2520_arch_init(void)
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
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //设置SPI工作模式:设置为主SPI
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;       //设置SPI的数据大小:SPI发送接收8位帧结构
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      //选择了串行时钟的稳态:时钟悬空低电平
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;    //数据捕获于第一个时钟沿
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;       //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;      //定义波特率预分频的值:波特率预分频值为256
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
        SPI_InitStructure.SPI_CRCPolynomial = 7;    //CRC值计算的多项式
    
        SPI_Init(SPI1, &SPI_InitStructure);
        // use hardware NSS
        //SPI_SSOutputCmd(SPI1, ENABLE);
    
        SPI_Cmd(SPI1, ENABLE);

        CC2520_READ_REG(CC2520_CHIPID, val);
        printf("\r\nID=0x%x\r\n", val);
    
        CC2520_READ_REG(CC2520_CHIPID, val);
        printf("\r\nID=0x%x\r\n", val);    
    
        CC2520_READ_REG(CC2520_CHIPID, val);
        printf("\r\nversion=0x%x\r\n", val);    
        
    
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


void cc2520_arch_fifop_int_init(void) 
{
}

void cc2520_arch_fifop_int_enable(void) 
{
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void cc2520_arch_fifop_int_disable(void) 
{
    NVIC_DisableIRQ(EXTI1_IRQn);
}

void cc2520_arch_fifop_int_clear(void)
{
}

/*---------------------------------------------------------------------------*/
