#ifndef UARTDMA_H_INCLUDED
#define UARTDMA_H_INCLUDED

#include <Tuareg_platform.h>




#define UART_TX_BUFFER_DATA_LEN_BYTES 512

typedef struct _UART_Tx_Buffer_t {

    U32 alloc_len;

    U32 data[UART_TX_BUFFER_DATA_LEN_BYTES];

} UART_Tx_Buffer_t;




void UART_TS_DMA_Init();


//buffered serial communication
//U32 UART_transmit(USART_TypeDef * Port, char data);
//U32 UART_write(USART_TypeDef * Port, char * msg);

//init functions
//void UART_TS_PORT_Init();
//void UART_DEBUG_PORT_Init();

//legacy direct hw access
//void UART_Tx(USART_TypeDef * Port, char msg);
//void UART_Tx_n(USART_TypeDef * Port, char Message, U32 Times);

//buffered TS serial communication
//U32 UART_getRX();
//U32 UART_available();

//void UART_reset();

//void UART_send_data(USART_TypeDef * pPort, volatile U8 * const pData, U32 Length);



#endif // UART_H_INCLUDED
