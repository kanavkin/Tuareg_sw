#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "decoder_logic.h"

#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"
#include "ignition_hw.h"
#include "ignition_config.h"

#include "scheduler.h"
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "table.h"
#include "eeprom.h"

#include "syslog.h"
#include "Ignition_syslog_locations.h"
#include "debug_port_messages.h"
#include "diagnostics.h"
#include "Tuareg.h"


#define IGNITION_REQUIRED_CONFIG_VERSION 4


#define IGNITION_DEBUG_OUTPUT

#ifdef IGNITION_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // IGNITION_DEBUG_OUTPUT


/**
crank position when the ignition controls shall be updated
*/
const crank_position_t ignition_controls_update_pos= CRK_POSITION_B2;


/**
How the Tuareg Ignition system works:

- With default or cranking ignition controls dwell shall begin immediately when the decoder indicates that the crank position is at dwell_pos.
- With default or cranking ignition controls ignition shall occur immediately when the decoder indicates that the crank position is at ignition_pos.

- With dynamic ignition controls the scheduler shall start coil dwell after the delay indicated by dwell_timing_sequential_us or dwell_timing_batch_us right after the spark has fired
- With dynamic ignition controls the scheduler shall trigger the spark after the delay indicated by ignition_timing_us after the decoder indicates that the crank position is at ignition_pos.

IMPLICATIONS

-> having only one scheduler for each coil controlling dwell and ignition events means that this scheduler can be allocated for dwell OR ignition timing at a time
-> conflicts arise when allocating the scheduler for ignition timing and dwell has not yet begun
-> to address this an override option "SCHEDOPT_REALLOC_COMPLETE" has been implemented


*/


void init_Ignition()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Ignition_Config();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Ignition Config
        Tuareg.errors.ignition_config_error= true;
        load_essential_Ignition_Config();

        Syslog_Error(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_LOAD_FAIL);

        #ifdef IGNITION_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Ignition config!");
        DebugMsg_Warning("Ignition essential config has been loaded");
        #endif // IGNITION_DEBUG_OUTPUT
    }
    else if(Ignition_Setup.Version != IGNITION_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong Ignition Config Version
        Tuareg.errors.ignition_config_error= true;
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
        Tuareg.errors.ignition_config_error= false;

        Syslog_Info(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_LOAD_SUCCESS);
    }

    //init hw part
    init_ignition_hw();

    //provide ignition controls for startup
    Tuareg_update_ignition_controls();
}


/**
emits the control events to actor (scheduler / coil) layer

precondition:
Tuareg.pDecoder->outputs.timeout == false
Tuareg.pDecoder->outputs.position_valid == true
*/
void Tuareg_ignition_update_crankpos_handler()
{
    VU32 age_us, corr_timing_us;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);

    /**
    check vital preconditions
    */
    if((Tuareg.flags.run_inhibit == true) || (Tuareg.flags.standstill == true))
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //turn off all coils
        set_coil1_unpowered();
        set_coil2_unpowered();

        //delete ignition controls
        Tuareg_update_ignition_controls();

        //nothing to do
        return;
    }

    //check if ignition controls shall be updated
    if(Tuareg.pDecoder->crank_position == ignition_controls_update_pos)
    {
        //update ignition controls
        Tuareg_update_ignition_controls();
    }


    //check if ignition controls are valid
    if(Tuareg.ignition_controls.flags.valid == false)
    {
        //collect diagnostic information
        //ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //nothing to do
        return;
    }


    //check if the crank is at the ignition base position
    if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos)
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);

        //compensate timing for execution delay
        age_us= decoder_get_position_data_age_us();
        corr_timing_us= subtract_VU32(Tuareg.ignition_controls.ignition_timing_us, age_us);

        //check if sequential mode has been commanded
        if(Tuareg.ignition_controls.flags.dynamic_controls == true)
        {
            //check if sufficient information for this mode is available
            if(Tuareg.pDecoder->outputs.phase_valid == false)
            {
                //register ERROR
                return;
            }

            ///sequential mode

            //coil #1
            if(Tuareg.pDecoder->phase == PHASE_CYL1_COMP)
            {
                scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us, true);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER);
            }

            //coil #2
            if(Tuareg.pDecoder->phase == PHASE_CYL1_EX)
            {
                scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us, true);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2SCHED_UNPOWER);
            }

        }
        else
        {
            ///batch mode

            //coil #1 and #2
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us, true);
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us, true);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_UNPOWER);
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_UNPOWER);
        }

    }

    /*
    check if the crank is at the ignition dwell position (for dynamic controls not available) -> batch style
    */
    if((Tuareg.ignition_controls.flags.dynamic_controls == false) && (Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.dwell_pos))
    {
        //coil #1 and #2
        set_ignition_ch1(ACTOR_POWERED);
        set_ignition_ch2(ACTOR_POWERED);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_POWER);
    }
}


/*
The ignition system will set up the scheduler channels for dwell in dynamic mode
*/
void Tuareg_ignition_irq_handler()
{
    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_IRQ3H_CALLS);

    /**
    check vital and operational preconditions
    */
//    if( (Tuareg.flags.run_inhibit == true) || (Tuareg.flags.standstill == true) || (Tuareg.ignition_controls.flags.valid == false) || (Tuareg.ignition_controls.flags.dynamic_controls == false) )                                                                                                                                                                                                                                  )
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_PRECOND_FAIL);

        //acknowledge irq flags
        Tuareg.flags.ign1_irq_flag= false;
        Tuareg.flags.ign2_irq_flag= false;

        //nothing to do
        return;
    }


    //check if sequential mode has been requested and sufficient information for this mode is available
    if((Tuareg.ignition_controls.flags.dynamic_controls == true) && (Tuareg.ignition_controls.flags.sequential_mode == true))
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

        //coil #1
        if((Tuareg.flags.ignition_coil_1 == false) && (Tuareg.flags.ign1_irq_flag == true) && (Tuareg.pDecoder->phase == PHASE_CYL1_COMP))
        {
            //acknowledge irq flag
            Tuareg.flags.ign1_irq_flag= false;

            //allocate scheduler
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_us, false);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1SCHED_POWER);
        }

        //coil #2
        if((Tuareg.flags.ignition_coil_2 == false) && (Tuareg.flags.ign2_irq_flag == true) && (Tuareg.pDecoder->phase == PHASE_CYL1_EX))
        {
            //acknowledge irq flag
            Tuareg.flags.ign2_irq_flag= false;

            //allocate scheduler
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_us, false);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_IRQ3H_IGN2SCHED_POWER);
        }
    }
    else
    {
        /*
        batch mode
        */

        //coil #1 and #2
        scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_us, false);
        scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_us, false);

        //acknowledge irq flags
        Tuareg.flags.ign1_irq_flag= false;
        Tuareg.flags.ign2_irq_flag= false;

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN2_POWER);
    }

}

