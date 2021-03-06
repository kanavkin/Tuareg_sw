#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "uart.h"
#include "uart_printf.h"

#include "systick_timer.h"
#include "conversion.h"

//#include "debug.h"

//#define UART_PUSH_DEBUG
//#define UART_SEND_DEBUG

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



VU32 TS_char_count =0;



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

	TS_char_count =0;
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

    if(Port == TS_PORT)
    {
        TS_char_count++;
    }
}



void UART_Tx_n(USART_TypeDef * Port, char Message, U32 Times)
{
    U32 i;

    for(i=0; i < Times; i++)
    {
        UART_Tx(Port, Message);
    }
}


void UART_TS_PORT_NEXT_LINE()
{
    UART_Tx(TS_PORT, '\r');
    UART_Tx(TS_PORT, '\n');

    TS_char_count =0;
}

void UART_TS_PORT_reset_char_count()
{
    TS_char_count =0;
}


VU32 UART_TS_PORT_get_char_count()
{
    return TS_char_count;
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

    #ifdef UART_PUSH_DEBUG
    /// TODO (oli#1#): debug action enabled
    if(data_in != 'A')
    {
        print(DEBUG_PORT, "\r\n+");
        Print_U8Hex(DEBUG_PORT, data_in);

        if(data_in >= 0x20)
        {
            UART_Tx(DEBUG_PORT, 0x60);
            UART_Tx(DEBUG_PORT, data_in);
            UART_Tx(DEBUG_PORT, 0x60);
        }

        UART_Tx(DEBUG_PORT, ' ');
        UART_Tx(DEBUG_PORT, 0x28);
        printf_U8(DEBUG_PORT, data_in);
        UART_Tx(DEBUG_PORT, 0x29);
    }
    #endif //UART_DEBUG


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
        print(DEBUG_PORT, "\r\n pl-s!");

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

    #ifdef UART_DEBUG
    /// TODO (oli#1#): debug action enabled
    UART_Tx(DEBUG_PORT, '-');
    UART_Tx(DEBUG_PORT, buffer->buffer[buffer->tail]);
    #endif //UART_DEBUG


    *data_out= buffer->buffer[buffer->tail];
    buffer->tail= next;

    buffer->available--;

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



/****************************************************************************************************************************************************
*
* send out binary data
*
****************************************************************************************************************************************************/
void UART_send_data(USART_TypeDef * pPort, volatile U8 * const pData, U32 Length)
{
    U32 i;
    U8 msg;

    #ifdef UART_SEND_DEBUG
    /// TODO (oli#1#): debug action enabled
    print(DEBUG_PORT, "\r\nsending l:");
    printf_U(DEBUG_PORT, Length, NO_PAD | NO_TRAIL);
    #endif //UART_DEBUG

    for(i=0; i < Length; i++)
    {
        msg= *(pData + i);

        #ifdef UART_SEND_DEBUG
        /// TODO (oli#1#): debug action enabled
        print(DEBUG_PORT, "\r\n");
        printf_U(DEBUG_PORT, i, NO_PAD);
        print(DEBUG_PORT, ": ");
        Print_U8Hex(DEBUG_PORT, msg);
        #endif //UART_DEBUG

        UART_Tx(pPort, msg);
    }
}



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

