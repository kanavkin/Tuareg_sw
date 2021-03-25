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
crank position when the fueling controls shall be updated
*/
crank_position_t fueling_controls_update_pos= CRK_POSITION_B1;




/**
The fueling module relies on external triggers to start injection, the scheduler is allocated only to maintain the injector interval


*/


void init_Fueling()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Fueling_Config();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Fueling Config
        load_essential_Fueling_Config();

        Syslog_Error(TID_TUAREG_FUELING, FUELING_LOC_CONFIG_LOAD_FAIL);

        #ifdef FUELING_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Fueling config!");
        DebugMsg_Warning("Fueling essential config has been loaded");
        #endif // FUELING_DEBUG_OUTPUT
    }
    else if(Fueling_Setup.Version != FUELING_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong Fueling Config Version
        load_essential_Fueling_Config();

        Syslog_Error(TID_TUAREG_FUELING, FUELING_LOC_CONFIG_VERSION_MISMATCH);

        #ifdef FUELING_DEBUG_OUTPUT
        DebugMsg_Error("Fueling config version does not match");
        DebugMsg_Warning("Fueling essential config has been loaded");
        #endif // FUELING_DEBUG_OUTPUT
    }
    else
    {
        //loaded Fueling config with correct Version
        Tuareg.Errors.fueling_config_error= false;

        Syslog_Info(TID_TUAREG_FUELING, FUELING_LOC_CONFIG_LOAD_SUCCESS);
    }

    //init hw part
    init_fueling_hw();

}


/**
emits the control events to actor (scheduler / injector / pump) layer

precondition:
Tuareg.pDecoder->outputs.timeout == false
Tuareg.pDecoder->outputs.position_valid == true

test result: ~61 us delay from signal edge B2 to injection begin
*/
void Tuareg_fueling_update_crankpos_handler()
{

    //collect diagnostic information
    //ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);

    /**
    check vital precondition
    In service mode ignition inhibit might be reset, but engine operation is not allowed!
    */
    if((Tuareg.Errors.fatal_error == true) || (Tuareg.actors.ignition_inhibit == true) || (Tuareg.Runmode == TMODE_SERVICE))
    {
        //collect diagnostic information
        //ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //turn off all powered coils
        set_injector1_unpowered();
        set_injector2_unpowered();
        set_fuel_pump_unpowered();

        //delete fuel controls
        Tuareg_update_fueling_controls();

        //nothing to do
        return;
    }

    //check if ignition controls shall be updated
    if(Tuareg.pDecoder->crank_position == fueling_controls_update_pos)
    {
        //update ignition controls
        Tuareg_update_fueling_controls();
    }


    //check preconditions for fueling control based actions
    if(Tuareg.fueling_controls.flags.valid == false)
    {
        //collect diagnostic information
        //ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //nothing to do
        return;
    }

    //check if the crank is at the injection begin position
    if(Tuareg.pDecoder->crank_position == Tuareg.fueling_controls.injection_begin_pos)
    {

        //collect diagnostic information
       // ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);


        //check if sequential mode has been requested
        if(Tuareg.fueling_controls.flags.sequential_mode == true)
        {
            //check if sufficient information for this mode is available
            if(Tuareg.pDecoder->outputs.phase_valid == false)
            {
                //register ERROR


                return;
            }


            /*
            sequential mode
            */

            //injector #1
            if(Tuareg.fueling_controls.seq_injector1_begin_phase == Tuareg.pDecoder->phase)
            {
                set_injector1(ACTOR_POWERED);

                scheduler_set_channel(SCHEDULER_CH_FUEL1, ACTOR_UNPOWERED, Tuareg.fueling_controls.injector1_interval_us, false);

                //collect diagnostic information
               // ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER);
            }

            //injector #2
            if(Tuareg.fueling_controls.seq_injector2_begin_phase == Tuareg.pDecoder->phase)
            {
                set_injector2(ACTOR_POWERED);

                scheduler_set_channel(SCHEDULER_CH_FUEL2, ACTOR_UNPOWERED, Tuareg.fueling_controls.injector2_interval_us, false);

                //collect diagnostic information
               // ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER);
            }

        }
        else
        {
            /*
            batch mode
            */
            set_injector1(ACTOR_POWERED);
            set_injector2(ACTOR_POWERED);

            scheduler_set_channel(SCHEDULER_CH_FUEL1, ACTOR_UNPOWERED, Tuareg.fueling_controls.injector1_interval_us, false);
            scheduler_set_channel(SCHEDULER_CH_FUEL2, ACTOR_UNPOWERED, Tuareg.fueling_controls.injector2_interval_us, false);
        }
    }
}


