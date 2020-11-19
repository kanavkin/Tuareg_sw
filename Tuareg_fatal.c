#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "uart.h"
#include "conversion.h"

#include "Tuareg_ID.h"


/**
Puts the system to a safe state when a critical error has been detected
*/
void Tuareg_Fatal(Tuareg_ID Id, U32 Location)
{
    UART_Send(DEBUG_PORT, "\r\n*** FATAL ERROR *** in Module ");
    UART_Print_U(DEBUG_PORT, Id, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT, "at Location ");
    UART_Print_U(DEBUG_PORT, Location, TYPE_U32, NO_PAD);

    while(1);

}


void Tuareg_Assert(bool Condition, Tuareg_ID Id, U32 Location)
{
    if(!Condition)
    {
        Tuareg_Fatal(Id, Location);
    }

}
