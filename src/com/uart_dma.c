
#include <Tuareg_platform.h>

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "uart_dma.h"

#ifdef DMA_USART_WIP

volatile UART_Tx_Buffer_t TXbuf_1, TXbuf_2;

VU32 Iterations =0;

#define MAX_ITERATIONS 100
timestamp_t system_ts[MAX_ITERATIONS +10];
timestamp_t fraction_ts[MAX_ITERATIONS +10];

/**
UART initialisation functions
/// TODO (oli#5#03/08/22): see page 640 of reference manual regarding baud rate generation
/// TODO (oli#5#03/08/22): implement dma modes https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx

*/
void UART_TS_DMA_Init()
{
    DMA_InitTypeDef DMA_InitStructure;

    //disable console
    UART_nolisten(TS_PORT);

    //fill TX buffer with ascii 0 .. 9 characters
    U32 i, k= 0x30;

    for(i=0; i < UART_TX_BUFFER_DATA_LEN_BYTES; i++)
    {
        if(k > 0x39) k=0x30;

        TXbuf_1.data[i]=k;

        k++;
    }

    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;



    /**
    USART1_TX is DMA2 Stream 7 CH 4
    USART6_TX is DMA2 Stream 6 CH 5
    */

    /*
    configure DMA2 CH4 Stream7 for ADC
    */
    DMA_StructInit(&DMA_InitStructure);

    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr= (U32)&(TS_PORT->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr= (U32) &(TXbuf_1.data);
    //DMA_InitStructure.DMA_Memory1BaseAddr= (U32)TXbuf_2;
    DMA_InitStructure.DMA_DIR= DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize= UART_TX_BUFFER_DATA_LEN_BYTES;
    DMA_InitStructure.DMA_PeripheralInc= DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc= DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize= DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode= DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority= DMA_Priority_High;

    //fire up config
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    //configure USART 1 for DMA TX
    TS_PORT->CR3 |= USART_CR3_DMAT;


    /**
    NVIC
    */

    //DMA2 stream 7 (prio 10)
    NVIC_SetPriority(DMA2_Stream7_IRQn, 10UL);
    NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);


    //Clear all DMA2 stream 7 interrupt pending bits
    DMA2->HIFCR |= DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

    //enable DMA2 stream 7 and its irq
    DMA2_Stream7->CR |= (U32) (DMA_SxCR_EN | DMA_SxCR_TCIE);

}


/**
results for 512 byte DMA transfer:
115200 bps -> 44,44 ms -> 86,8 us/byte
230400 bps -> 22,22 ms -> 43,4 us/byte
460800 bps -> 11,11 ms -> 21,7 us/byte
500000 -> error
576000 -> 8,9 ms -> 17,38 us/byte
921600 -> 5,5 ms -> 10,75 us/byte
115200 bps -> 4,45 ms -> 8,7 us/byte

desired transmission time: max. 8 ms!
*/
void DMA2_Stream7_IRQHandler()
{
    U32 i, dur, ts1, ts2;


    //DMA2 stream 7 Transfer Complete interrupt
    if(DMA2->HISR & DMA_HISR_TCIF7)
    {

        //Clear all DMA2 stream 7 interrupt pending bits
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

        //timestamps
        system_ts[Iterations]= Tuareg.pTimer->system_time;
        fraction_ts[Iterations]= get_timestamp_fraction_us();
        Iterations++;

        if(Iterations >= MAX_ITERATIONS)
        {
            //pause transfer
            DMA2_Stream7->CR= (U32) 0;


            print(TS_PORT, "\r\nTransfers completed: ");
            printf_U(TS_PORT, Iterations, NO_PAD);

            print(TS_PORT, "\r\nTimestamps:\r\n");
            for(i=0; i < MAX_ITERATIONS; i++)
            {
                printf_U(TS_PORT, system_ts[i], NO_PAD);
                printf_U(TS_PORT, fraction_ts[i], NO_PAD);

                if(i>0)
                {
                    ts1=system_ts[i-1] * 1000 + fraction_ts[i-1];
                    ts2=system_ts[i] * 1000 + fraction_ts[i];
                    dur=ts2 - ts1;

                    printf_U(TS_PORT, dur, NO_PAD);
                }

                print(TS_PORT, "\r\n");
            }



        }








    }

}


#endif // DMA_USART_WIP




