
#include <Tuareg_platform.h>

#include "uart.h"
#include "uart_printf.h"

#include "serial_buffer.h"


//#define UART_PUSH_DEBUG

#ifdef UART_PUSH_DEBUG
#warning uart push debug enabled
#endif // UART_PUSH_DEBUG


//#define SERIAL_MONITOR

#ifdef SERIAL_MONITOR
#warning serial monitor enabled
#endif // SERIAL_MONITOR

//#define UART_SEND_DEBUG

#ifdef UART_SEND_DEBUG
#warning uart send debug enabled
#endif // UART_SEND_DEBUG





/**
UART initialisation functions
/// TODO (oli#5#03/08/22): see page 640 of reference manual regarding baud rate generation
/// TODO (oli#5#03/08/22): implement dma modes https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx

*/
void UART_TS_PORT_Init()
{
    USART_InitTypeDef usart_conf;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    //GPIOA alternative functions 9 > Tx, 10 > Rx
    GPIO_configure(GPIOA, 9, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 10, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);

    //connect USART1 to PA9/10 and USART6 to PA11
    GPIO_SetAF(GPIOA, 9, 7);
    GPIO_SetAF(GPIOA, 10, 7);

    /**
    8-bit data, one stop bit, no parity,
    do both Rx and Tx, no HW flow control
    */
    //usart_conf.USART_BaudRate = 115200;
    usart_conf.USART_BaudRate = 230400;
    usart_conf.USART_WordLength = USART_WordLength_8b;
    usart_conf.USART_StopBits = USART_StopBits_1;
    usart_conf.USART_Parity = USART_Parity_No ;
    usart_conf.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_conf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    //configure and start
    USART_Init(USART1, &usart_conf);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    //Enable USART1 global interrupt (prio 14)
    NVIC_SetPriority(USART1_IRQn, 14UL);
    NVIC_EnableIRQ(USART1_IRQn);

	#ifdef SERIAL_MONITOR
    //monitor buffer
	mrx_ptr =0;
	mrd_ptr =0;
	#endif // SERIAL_MONITOR

}


/**
using USART6 for debugging
*/
void UART_DEBUG_PORT_Init()
{
    USART_InitTypeDef usart_conf;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    //GPIOA alternative functions 11 > Tx, RX not connected
    GPIO_configure(GPIOA, 11, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_NONE);

    //connect USART1 to PA9/10 and USART6 to PA11
    GPIO_SetAF(GPIOA, 11, 8);

    /**
    8-bit data, one stop bit, no parity,
    do both Rx and Tx, no HW flow control
    */
    //usart_conf.USART_BaudRate = 115200;
    usart_conf.USART_BaudRate = 230400;
    usart_conf.USART_WordLength = USART_WordLength_8b;
    usart_conf.USART_StopBits = USART_StopBits_1;
    usart_conf.USART_Parity = USART_Parity_No ;
    usart_conf.USART_Mode = USART_Mode_Tx;
    usart_conf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    //configure and start
    USART_Init(USART6, &usart_conf);
    USART_Cmd(USART6, ENABLE);
}



/**
legacy UART interface with direct hw access
blocks program flow until data sent!
*/
void UART_Tx(USART_TypeDef * Port, char msg)
{
    #ifdef SERIAL_MONITOR
    UART_nolisten();
    monitor_log(msg, DIRECTION_OUT);
    #endif // SERIAL_MONITOR

    //loop until the end of transmission
    while( !(Port->SR & USART_FLAG_TXE) )
    {
    }

    Port->DR= msg;

    #ifdef SERIAL_MONITOR
    UART_listen();
    #endif // SERIAL_MONITOR

}




void UART_listen(USART_TypeDef * Port)
{
    /* enable RXNE interrupt */
    USART_ITConfig(Port, USART_IT_RXNE, ENABLE);
}


void UART_nolisten(USART_TypeDef * Port)
{
    /* disable RXNE interrupt */
    USART_ITConfig(Port, USART_IT_RXNE, DISABLE);
}








/****************************************************************************************************************************************************
*
* send out binary data
*
****************************************************************************************************************************************************/
void UART_send_data(USART_TypeDef * pPort, volatile U8 * const pData, U32 Length)
{
    U32 i;
    U8 msg;

    for(i=0; i < Length; i++)
    {
        msg= *(pData + i);

        UART_Tx(pPort, msg);
    }
}






/****************************************************************************************************************************************************
*
* USART1 Interrupt Service Routine
*
****************************************************************************************************************************************************/

void USART1_IRQHandler(void)
{
    VU32 data_in;

    /* RXNE handler */
    if(USART1->SR & USART_SR_RXNE)
    {
        //Clear Pending Bit
        USART1->SR= ~USART_SR_RXNE ;

        data_in= USART_ReceiveData(USART1);
        //data_in= (char) (USART1->DR & (uint16_t)0x01FF);
        //return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);

/// TODO (oli#8#03/18/22): add error / diag statistics for com port

        ComRx_Buffer_push(data_in);

        #ifdef SERIAL_MONITOR
        monitor_log(data_in, DIRECTION_IN);
        #endif // SERIAL_MONITOR
    }

}

