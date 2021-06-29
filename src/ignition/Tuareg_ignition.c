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
        Tuareg.flags.limited_op= true;
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
        Tuareg.flags.limited_op= true;
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

    //bring up vital scheduler
    init_Vital_Scheduler();

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
    volatile scheduler_activation_parameters_t scheduler_parameters;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);

    /**
    check vital preconditions
    */
    if(Tuareg.flags.run_inhibit == true)
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //turn off all ignition actors
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


    /**
    check if dynamic mode is active and the crank is at the ignition base position
    -> this is the main operation scenario
    */
    if((Tuareg.ignition_controls.flags.dynamic_controls == true) && (Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos))
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);

        /*
        prepare scheduler activation parameters
        - first action is ignition -> turn off coil
        - second action is dwell -> power coil
        - use 2 intervals
        - no realloc completion
        - interval 1 -> corrected ignition timing
        - interval 2 -> dwell timing
        */
        scheduler_parameters.flags.action1_power= false;
        scheduler_parameters.flags.action2_power= true;
        scheduler_parameters.flags.interval2_enabled= true;
        scheduler_parameters.flags.complete_cycle_realloc= false;

        scheduler_parameters.interval2_us= Tuareg.ignition_controls.dwell_timing_us;
        scheduler_parameters.interval1_us= subtract_VU32(Tuareg.ignition_controls.ignition_timing_us, decoder_get_position_data_age_us());


        //check if sequential mode has been commanded
        if((Tuareg.ignition_controls.flags.sequential_mode == true) && (Tuareg.pDecoder->outputs.phase_valid == false))
        {
            //register ERROR
            return;
        }

        //coil #1
        if((Tuareg.pDecoder->phase == PHASE_CYL1_COMP) || (Tuareg.ignition_controls.flags.sequential_mode == false))
        {
            scheduler_set_channel(SCHEDULER_CH_IGN1, &scheduler_parameters);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER);
        }

        //coil #2
        if((Tuareg.pDecoder->phase == PHASE_CYL1_EX) || (Tuareg.ignition_controls.flags.sequential_mode == false))
        {
            scheduler_set_channel(SCHEDULER_CH_IGN2, &scheduler_parameters);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2SCHED_UNPOWER);
        }

        //all done
        return;

    }


    /**
    Everything other than dynamic mode is fallback -> batch style
    use cases: cranking, default ignition controls
    operating scheme: immediate spark/dwell triggering on position update
    no scheduler allocation
    */
    if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos)
    {
        scheduler_reset_channel(SCHEDULER_CH_IGN1);
        scheduler_reset_channel(SCHEDULER_CH_IGN2);

        //coil #1 and #2
        set_ignition_ch1(ACTOR_UNPOWERED);
        set_ignition_ch2(ACTOR_UNPOWERED);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_UNPOWER);
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_UNPOWER);

    }
    else if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.dwell_pos)
    {
        scheduler_reset_channel(SCHEDULER_CH_IGN1);
        scheduler_reset_channel(SCHEDULER_CH_IGN2);

        //coil #1 and #2
        set_ignition_ch1(ACTOR_POWERED);
        set_ignition_ch2(ACTOR_POWERED);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_POWER);
    }
}
