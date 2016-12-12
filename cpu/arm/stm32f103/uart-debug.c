#include <stdio.h>

#include "stm32f10x.h"
#include "uart-debug.h"

int (* uart1_input_handler)(unsigned char c) = NULL;

void USART1_handler() __attribute__((interrupt));
void USART2_handler() __attribute__((interrupt));

// redefine IRQ handler functions
void USART1_handler(void)
{
    static int counter = 0;
    uint8_t c = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        c = USART_ReceiveData(USART1);

        // FIXME
        USART_SendData(USART1, c);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        
        if (NULL != uart1_input_handler) {
            uart1_input_handler(c);
        }
        //counter++;
        /// printf("%d\r\n",counter);
        //process_post(&receive_process, event_data_ready, (void*)&counter);        
    }
}

void USART2_handler(void)
{
    static unsigned char led = 1;
    uint8_t temp = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        temp = USART_ReceiveData(USART2);
        printf("UART2=%x\r\n", temp);

        if (0x31 == temp) { // LEF ON
            led = 1;            
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
            GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
            USART_SendData(USART2, led?'1':'0');
            while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);    
        }
        
        if (0x30 == temp) {// LED OFF
            led = 0;
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
            GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);             
            USART_SendData(USART2, led?'1':'0');
            while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        }
        //process_post(&receive_process, event_data_ready, (void*)&counter);        
    }
}

void debug_init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // enable clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    // PA9 TX1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // PA10 RX1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    // enable USART1
    USART_Cmd(USART1, ENABLE);
    
    //
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    //
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
}

void uart2_init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE ); 
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );    
    
    /* Configure USART2 Rx (PA3) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    /* Configure USART2 Tx (PA2) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
  
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    USART_Cmd(USART2, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
}


// LED0=PA8 LED1=PD2
void led_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD,ENABLE);

  //LED OFF
  GPIO_SetBits(GPIOA,GPIO_Pin_8);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // LED OFF
  GPIO_SetBits(GPIOD,GPIO_Pin_2);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

#if 1
int fputc(int ch, FILE * f)
{
#if 0
    if ('\r' == ch)
        return ch;
#endif    
    // send '\r' if ch = 'n'
    if ('\n' == ch) {
        USART_SendData(USART1, '\r');
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
    }
    USART_SendData(USART1, (uint8_t)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET ); 
    return ch;
}
#endif

void
dbg_putchar(const char ch)
{
    fputc(ch, NULL);
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
    int i = 0;
    
    while (seq && *seq != 0) {
        if (i >= len) 
            break;
        
        fputc(*seq++, NULL);
        i++;
    }
    return i;
}
