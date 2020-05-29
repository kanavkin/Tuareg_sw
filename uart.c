#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "uart.h"
#include "lowspeed_timers.h"
#include "conversion.h"

#include "debug.h"

VU8 TS_Rx_Buffer_data[TS_RX_BUFFER_SIZE];
//VU8 Debug_Tx_Buffer_data[DEBUG_TX_BUFFER_SIZE];

volatile serial_buffer_t TS_Rx_Buffer= {

    .buffer= TS_Rx_Buffer_data,
    .length= TS_RX_BUFFER_SIZE,
    .head =0,
    .tail =0,
    .available =0
};

/*
volatile serial_buffer_t Debug_Tx_Buffer= {

    .buffer= Debug_Tx_Buffer_data,
    .length= DEBUG_TX_BUFFER_SIZE,
    .head =0,
    .tail =0,
    .available =0
};
*/



/**
UART initialisation functions
*/
void UART_TS_PORT_Init()
{
    USART_InitTypeDef usart_conf;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    //GPIOA alternative functions 9 > Tx, 10 > Rx
    GPIO_configure(GPIOA, 9, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
#warning TODO (oli#1#): observe sporadic uart errors: GPIOA 10 GPIO_MODE_IN / GPIO_MODE_AF config dependant
    GPIO_configure(GPIOA, 10, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);

    //connect USART1 to PA9/10 and USART6 to PA11
    GPIO_SetAF(GPIOA, 9, 7);
    GPIO_SetAF(GPIOA, 10, 7);

    /**
    8-bit data, one stop bit, no parity,
    do both Rx and Tx, no HW flow control
    */
    usart_conf.USART_BaudRate = 115200;
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
    //GPIO_configure(GPIOA, 12, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_NONE);

    //connect USART1 to PA9/10 and USART6 to PA11
    GPIO_SetAF(GPIOA, 11, 8);
    //GPIO_SetAF(GPIOA, 12, 8);

    /**
    8-bit data, one stop bit, no parity,
    do both Rx and Tx, no HW flow control
    */
    usart_conf.USART_BaudRate = 115200;
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
    /**
    serial monitor
    */
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

void UART_Send(USART_TypeDef * Port, char messg[] )
{
    for( ; *messg != 0 ; messg++)
    {
        UART_Tx(Port, *messg);
    }
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



/**
buffered TunerStudio communication
legacy style interface
*/
U32 UART_available()
{
    return TS_Rx_Buffer.available;
}

U32 UART_getRX()
{
    U8 data;
    serial_buffer_pull(&TS_Rx_Buffer, &data);
    return data;
}

/**
buffered serial communication
*/
/*
U32 UART_transmit(USART_TypeDef * Port, char data)
{
    volatile serial_buffer_t * buffer;

    if(Port == USART3)
    {
        buffer= &Debug_Tx_Buffer;
    }
    else
    {
        //invalid port
        return 1;
    }

    serial_buffer_push(buffer, data);

    return 0;
}


U32 UART_write(USART_TypeDef * Port, char * msg)
{

    volatile serial_buffer_t * buffer;


    if(Port == USART3)
    {
        buffer= &Debug_Tx_Buffer;
    }
    else
    {
        //invalid port
        return 1;
    }

    for( ; *msg != 0 ; msg++)
    {
        serial_buffer_push(buffer, *msg);
    }

    return 0;
}
*/


/**
Uart cyclic buffer access functions
*/
U32 serial_buffer_push(volatile serial_buffer_t * buffer, VU8 data_in)
{
    U32 next;

    /**
    block buffer access
    buffer_push has priority over all other
    buffer accesses
    -> new data received will be stored immediately
    */
    //buffer->semaphor= SEMAPHOR_BLOCK;

    next= buffer->head +1;

    if(next >= buffer->length)
    {
        next =0;
    }

    if(next == buffer->tail)
    {
        /**
        buffer full
        */
        //buffer->semaphor= SEMAPHOR_FREE;
        return BUFFER_PUSH_ERROR_FULL;
    }

    buffer->buffer[buffer->head]= data_in;
    buffer->head= next;

    buffer->available++;

/*
#warning TODO (oli#1#): debug action enabled
    //debug
        UART_Send(DEBUG_PORT, "\r\n pushed-");
        UART_Tx(DEBUG_PORT, data_in);
        //UART_Send(DEBUG_PORT, " avail-");
        //UART_Print_U(DEBUG_PORT, buffer->available, TYPE_U32, NO_PAD);
*/

    //return buffer access
    //buffer->semaphor= SEMAPHOR_FREE;

    return BUFFER_PUSH_SUCCESS;

}


U32 serial_buffer_pull(volatile serial_buffer_t * buffer, VU8 * data_out)
{
    U32 next;

    /*
    if(buffer->semaphor)
    {
        //debug
        UART_Send(DEBUG_PORT, "\r\n pl-s!");

        return BUFFER_PULL_ERROR_SEMAPHOR;
    }
    */


    if(buffer->head == buffer->tail)
    {
        /**
        called but no data in buffer
        */
        return BUFFER_PULL_ERROR_EMPTY;
    }

    next= buffer->tail +1;

    if(next >= buffer->length)
    {
        next =0;
    }

    *data_out= buffer->buffer[buffer->tail];
    buffer->tail= next;

    buffer->available--;

    /*
    //debug
        UART_Send(DEBUG_PORT, "\r\n\t pulled-");
        UART_Tx(DEBUG_PORT, * data_out);
        UART_Send(DEBUG_PORT, " avail-");
        UART_Print_U(DEBUG_PORT, buffer->available, TYPE_U32, NO_PAD);
    */

    return  BUFFER_PULL_SUCCESS;


}


/*
void UART_periodic()
{
    U32 result;
    U8 data_out;

    //push out debug messaged
    result= serial_buffer_pull(&Debug_Tx_Buffer, &data_out);

    if(result == BUFFER_PULL_SUCCESS)
    {
        UART_Tx(DEBUG_PORT, data_out);
    }

}
*/






/**********************************************************
 * USART1 Interrupt Service Routine
 *********************************************************/
void USART1_IRQHandler(void)
{
    VU8 data_in;

    /* RXNE handler */
    if(USART1->SR & USART_SR_RXNE)
    {
        //Clear Pending Bit
        USART1->SR= ~USART_SR_RXNE ;

        data_in= (char) USART_ReceiveData(USART1);
        //data_in= (char) (USART1->DR & (uint16_t)0x01FF);

        serial_buffer_push(&TS_Rx_Buffer, data_in);

        #ifdef SERIAL_MONITOR
        /**
        serial monitor
        */
        monitor_log(data_in, DIRECTION_IN);
        #endif // SERIAL_MONITOR
    }

}

