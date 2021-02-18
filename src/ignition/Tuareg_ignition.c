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

//#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"



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

    if(result != EXEC_OK)
    {
        load_essential_Ignition_Config();

        Tuareg.Errors.ignition_config_error= true;

        print(DEBUG_PORT, "\r\nWARNING Ignition essential Config has been loaded");
    }

    if(Ignition_Setup.Version != IGNITION_REQUIRED_CONFIG_VERSION)
    {
        Tuareg.Errors.ignition_config_error= true;

        print(DEBUG_PORT, "\r\nWARNING Ignition Config version does not match");
    }
    else
    {
        Tuareg.Errors.ignition_config_error= false;
    }

    print(DEBUG_PORT, "\r\nINFO Ignition Config has been loaded");


    //init hw part
    init_ignition_hw();

    //provide ignition controls for startup
    Tuareg_update_ignition_controls();
}


/**
emits the control events to actor (scheduler / coil) layer
*/
void Tuareg_ignition_update_crankpos_handler()
{
    VU32 age_us, corr_timing_us;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);

    //check vital precondition
    if(Tuareg.actors.ignition_inhibit == true)
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //turn off all powered coils
        set_ignition_ch1(ACTOR_UNPOWERED);
        set_ignition_ch2(ACTOR_UNPOWERED);

        //nothing to do
        return;
    }

    //check preconditions
    if((Tuareg.ignition_controls.state.valid == false)  || (Tuareg.ignition_controls.state.rev_limiter == true) ||
       (Tuareg.pDecoder->outputs.position_valid == false) || (Tuareg.pDecoder->outputs.timeout == true))
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        //nothing to do
        return;
    }

    //check if the crank is at the ignition base position
    if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos)
    {

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);

        //compensate timing for execution delay
        //age_us= decoder_get_data_age_us();
        //corr_timing_us= subtract_VU32(Tuareg.ignition_controls.ignition_timing_us, age_us);
        corr_timing_us= Tuareg.ignition_controls.ignition_timing_us;

        //check if sequential mode has been requested and sufficient information for this mode is available
        if((Tuareg.ignition_controls.state.dynamic_controls == true) && (Tuareg.ignition_controls.state.sequential_mode == true) && (Tuareg.pDecoder->outputs.phase_valid == true))
        {

            /*
            sequential mode
            */

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
            /*
            batch mode
            */

            //coil #1
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us, true);

            //coil #2
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us, true);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_UNPOWER);
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_UNPOWER);
        }

    }

    /*
    check if the crank is at the ignition dwell position
    */
    else if((Tuareg.ignition_controls.state.dynamic_controls == false) && (Tuareg.ignition_controls.state.sequential_mode == false) && (Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.dwell_pos))
    {
        set_ignition_ch1(ACTOR_POWERED);
        set_ignition_ch2(ACTOR_POWERED);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_POWER);
    }
}


/*
The ignition system will set up the scheduler channels for dwell in dynamic mode

BIG QUESTION:
shal irq3 be triggered on every spark? or only on coil #1?


*/
void Tuareg_ignition_irq_handler()
{
    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_IRQ3H_CALLS);

    //check preconditions
    if((Tuareg.actors.ignition_inhibit == true) ||
        (Tuareg.ignition_controls.state.valid == false) || (Tuareg.ignition_controls.state.rev_limiter == true) || (Tuareg.ignition_controls.state.dynamic_controls == false) ||
        (Tuareg.pDecoder->outputs.position_valid == false) || (Tuareg.pDecoder->outputs.timeout == true) )
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_PRECOND_FAIL);

        //acknowledge irq flags
        Tuareg.actors.ign1_irq_flag= false;
        Tuareg.actors.ign2_irq_flag= false;

        //nothing to do
        return;
    }


    //check if sequential mode has been requested and sufficient information for this mode is available
    if((Tuareg.ignition_controls.state.dynamic_controls == true) && (Tuareg.ignition_controls.state.sequential_mode == true) && (Tuareg.pDecoder->outputs.phase_valid == true))
    {
        /*
        sequential mode
        */

        //coil #1
        if((Tuareg.actors.ignition_coil_1 == false) && (Tuareg.actors.ign1_irq_flag == true) && (Tuareg.pDecoder->phase == PHASE_CYL1_COMP))
        {
            //acknowledge irq flag
            Tuareg.actors.ign1_irq_flag= false;

            //allocate scheduler
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_sequential_us, false);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1SCHED_POWER);
        }

        //coil #2
        if((Tuareg.actors.ignition_coil_2 == false) && (Tuareg.actors.ign2_irq_flag == true) && (Tuareg.pDecoder->phase == PHASE_CYL1_EX))
        {
            //acknowledge irq flag
            Tuareg.actors.ign2_irq_flag= false;

            //allocate scheduler
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_sequential_us, false);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_IRQ3H_IGN2SCHED_POWER);
        }
    }
    else
    {
        /*
        batch mode
        */

        //coil #1
        scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_batch_us, false);

        //coil #2
        scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_batch_us, false);

        //acknowledge irq flags
        Tuareg.actors.ign1_irq_flag= false;
        Tuareg.actors.ign2_irq_flag= false;

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN2_POWER);
    }

}

