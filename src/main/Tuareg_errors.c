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
#include "Tuareg_syslog_locations.h"
#include "debug_port_messages.h"


#define ERRORS_DEBUG_OUTPUT

#ifdef ERRORS_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // ERRORS_DEBUG_OUTPUT



/**
Puts the system to a safe state when a critical error has been detected

This function has to be very restrictive.
BUT
An error could upset the program logic in a way that RunMode will not be updated to FATAL.

So vital actors operation is inhibited already here.
*/
void Fatal(Tuareg_ID Id, U8 Location)
{
    __disable_irq();

    Tuareg.errors.fatal_error= true;
    Tuareg.flags.run_inhibit= true;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_FATAL_ERROR);

    #ifdef ERRORS_DEBUG_OUTPUT
    DebugMsg_Error("FATAL --");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUG_OUTPUT
}


void Assert(bool Condition, Tuareg_ID Id, U8 Location)
{
    if(!Condition)
    {
        Fatal(Id, Location);
    }
}


/**
Activates the systems limited operation strategy when a critical error has been detected
*/
void Limp(Tuareg_ID Id, U8 Location)
{
    Tuareg.flags.limited_op= true;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_ENTER_LIMP_MODE);

    #ifdef ERRORS_DEBUG_OUTPUT
    DebugMsg_Error("LIMP --");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUG_OUTPUT
}

