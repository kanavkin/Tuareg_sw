#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"

#include "base_calc.h"

#include "uart.h"
#include "uart_printf.h"
#include "debug_port_messages.h"

#include "diagnostics.h"
#include "highspeed_loggers.h"

/******************************************************************************************************************************
decoder helper functions - debug actions
******************************************************************************************************************************/

/// TODO (oli#4#): redirect decoder debug outputs to syslog

void decoder_process_debug_events()
{
    if(Decoder.debug.all_flags > 0)
    {
        DebugMsg_Warning("Decoder Debug Events (sync_lost got_sync timer_overflow standstill): ");
        UART_Tx(DEBUG_PORT, (Decoder.debug.lost_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.got_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.timer_overflow? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.standstill? '1' :'0'));

        //reset flags
        Decoder.debug.all_flags= 0;
    }
}

 void register_sync_lost_debug_event()
{
    //set flag
    Decoder.debug.lost_sync= true;
}


 void register_got_sync_debug_event()
{
    //set flag
    Decoder.debug.got_sync= true;
}


 void register_timer_overflow_debug_event()
{
    //set flag
    Decoder.debug.timer_overflow= true;
}


 void register_timeout_debug_event()
{
    //set flag
    Decoder.debug.standstill= true;
}



