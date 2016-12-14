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
	EXTI_ClearITPendingBit(EXTI_Line0);  //���EXTI0��·����λ
}

void EXTI1_handler(void)
{
    printf("EXT1\r\n");
    cc2520_interrupt();
	EXTI_ClearITPendingBit(EXTI_Line1);  //���EXTI1��·����λ
}

static void cc2520_irq_init()
{
          EXTI_InitTypeDef EXTI_InitStructure;
          //NVIC_InitTypeDef NVIC_InitStructure;
          GPIO_InitTypeDef GPIO_InitStructure;
      
          // PA0 = WK_UP
          RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTA,PORTCʱ��
          GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0���ó����룬Ĭ������
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.0
      
          RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
      
          //PA0 EXT0
          GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
          EXTI_InitStructure.EXTI_Line=EXTI_Line0;
          EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
          EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
          EXTI_InitStructure.EXTI_LineCmd = ENABLE;
          EXTI_Init(&EXTI_InitStructure);     //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
          //NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
          //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;    //��ռ���ȼ�2 
          //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;                   //�����ȼ�1
          //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //ʹ���ⲿ�ж�ͨ��
          //NVIC_Init(&NVIC_InitStructure);       //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

          NVIC_SetPriority(EXTI0_IRQn, 2);
          NVIC_ENABLE_INT(EXTI0_IRQn);

          
          // PC1 EXT1
          GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
          EXTI_InitStructure.EXTI_Line=EXTI_Line1;
          EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
          EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//
          EXTI_InitStructure.EXTI_LineCmd = ENABLE;
          EXTI_Init(&EXTI_InitStructure);     //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
          //NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;            //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
          //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;    //��ռ���ȼ�2�� 
          //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;                   //�����ȼ�1
          //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //ʹ���ⲿ�ж�ͨ��
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
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure); 
        GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);    
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //����SPI����ģʽ:����Ϊ��SPI
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      //ѡ���˴���ʱ�ӵ���̬:ʱ�����յ͵�ƽ
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;    //���ݲ����ڵ�һ��ʱ����
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;       //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;      //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
        SPI_InitStructure.SPI_CRCPolynomial = 7;    //CRCֵ����Ķ���ʽ
    
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
