#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "decoder_logic.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "scheduler.h"
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "table.h"
#include "eeprom.h"

#include "syslog.h"
#include "Fueling_syslog_locations.h"
#include "debug_port_messages.h"
#include "diagnostics.h"
#include "Tuareg.h"


#define FUELING_DEBUG_OUTPUT

#ifdef FUELING_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // FUELING_DEBUG_OUTPUT


/**



*/


void init_Fueling()
{
    exec_result_t result;
    /*

    //setup shall be loaded first
    result= load_Fueling_Config();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Decoder Config
        load_essential_Fueling_Config();

        Syslog_Error(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_LOAD_FAIL);

        #ifdef IGNITION_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Ignition config!");
        DebugMsg_Warning("Ignition essential config has been loaded");
        #endif // IGNITION_DEBUG_OUTPUT
    }
    else if(Ignition_Setup.Version != IGNITION_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong Decoder Config Version
        load_essential_Ignition_Config();

        Syslog_Error(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_VERSION_MISMATCH);

        #ifdef IGNITION_DEBUG_OUTPUT
        DebugMsg_Error("Ignition config version does not match");
        DebugMsg_Warning("Ignition essential config has been loaded");
        #endif // IGNITION_DEBUG_OUTPUT
    }
    else
    {
        //loaded Ignition config with correct Version
        Tuareg.Errors.ignition_config_error= false;

        Syslog_Info(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_LOAD_SUCCESS);
    }

    //init hw part
    init_ignition_hw();

    //provide ignition controls for startup
    Tuareg_update_ignition_controls();
    */
    return EXEC_ERROR;
}


/**
emits the control events to actor (scheduler / coil) layer
*/
void Tuareg_fueling_update_crankpos_handler()
{



}


/*
The ignition system will set up the scheduler channels for dwell in dynamic mode
*/
void Tuareg_fueling_irq_handler()
{


}

