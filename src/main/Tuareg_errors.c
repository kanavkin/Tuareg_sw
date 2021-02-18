#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg.h"
#include "Tuareg_errors.h"
#include "Tuareg_ID.h"


/**
Puts the system to a safe state when a critical error has been detected
*/
void Tuareg_Fatal(Tuareg_ID Id, U32 Location)
{
    print(DEBUG_PORT, "\r\n*** FATAL ERROR *** in Module ");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    print(DEBUG_PORT, "at Location ");
    printf_U(DEBUG_PORT, Location, NO_PAD);

    while(1);

}


void Tuareg_Assert(bool Condition, Tuareg_ID Id, U32 Location)
{
    if(!Condition)
    {
        Tuareg_Fatal(Id, Location);
    }

}
