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

#include "syslog.h"
#include "debug_port_messages.h"

/**
Puts the system to a safe state when a critical error has been detected
*/
void Fatal(Tuareg_ID Id, U8 Location)
{

    Syslog_Error(Id, Location);

    DebugMsg_Error("FATAL --");


/// TODO (oli#3#): implement FATAL mode with only defensive debug printouts enabled

    while(1);

}


inline void Assert(bool Condition, Tuareg_ID Id, U8 Location)
{
    if(!Condition)
    {
        Fatal(Id, Location);
    }
}
