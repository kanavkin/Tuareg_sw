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
#include "conversion.h"
#include "table.h"
#include "eeprom.h"

#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"



/**
How the Tuareg Ignition system works:

- With default or cranking ignition controls dwell shall begin immediately when the decoder indicates that the crank position is at dwell_pos.
- With default or cranking ignition controls ignition shall occur immediately when the decoder indicates that the crank position is at ignition_pos.

- With dynamic ignition controls the scheduler shall start coil dwell after the delay indicated by dwell_timing_sequential_us or dwell_timing_batch_us right after the spark has fired
- With dynamic ignition controls the scheduler shall trigger the spark after the delay indicated by ignition_timing_us after the decoder indicates that the crank position is at ignition_pos.

*/


/**
emits the control events to actor (scheduler / coil) layer
*/
void Tuareg_ignition_update_crankpos_handler()
{
    VU32 age_us, corr_timing_us;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);


    //check preconditions
    if((Tuareg.actors.ignition_inhibit == true) ||
       (Tuareg.ignition_controls.state.valid == false)  || (Tuareg.ignition_controls.state.rev_limiter == true) ||
       (Tuareg.decoder->state.position_valid == false) || (Tuareg.decoder->state.timeout == true))
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_PRECOND_FAIL);

        return;
    }


    //check if the crank is at the ignition base position
    if(Tuareg.decoder->crank_position == Tuareg.ignition_controls.ignition_pos)
    {

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);


        //compensate timing for execution delay
        //age_us= decoder_get_data_age_us();
        //corr_timing_us= subtract_VU32(Tuareg.ignition_controls.ignition_timing_us, age_us);
        corr_timing_us= Tuareg.ignition_controls.ignition_timing_us;


        //check if sequential mode has been requested and sufficient information for this mode is available
        if((Tuareg.ignition_controls.state.sequential_mode == true) && (Tuareg.decoder->state.phase_valid == true))
        {
            /*
            sequential mode
            */

            //coil #1
            if(Tuareg.decoder->phase == PHASE_CYL1_COMP)
            {
                scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER);
            }

            //coil #2
            if(Tuareg.decoder->phase == PHASE_CYL1_EX)
            {
                scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us);

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
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us);

            //coil #2
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_UNPOWER);
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_UNPOWER);
        }

    }

    /*
    check if the crank is at the ignition dwell position
    */
    if((Tuareg.ignition_controls.state.dynamic_controls == false) && (Tuareg.decoder->crank_position == Tuareg.ignition_controls.dwell_pos))
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
*/
void Tuareg_ignition_irq_handler()
{
    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_IRQ3H_CALLS);

    //check preconditions
    if((Tuareg.actors.ignition_inhibit == true) ||
        (Tuareg.ignition_controls.state.valid == false) || (Tuareg.ignition_controls.state.rev_limiter == true) || (Tuareg.ignition_controls.state.dynamic_controls == false) ||
        (Tuareg.decoder->state.position_valid == false) || (Tuareg.decoder->state.timeout == true) )
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_PRECOND_FAIL);

        return;
    }


    //check if sequential mode has been requested and sufficient information for this mode is available
    if((Tuareg.ignition_controls.state.sequential_mode == true) && (Tuareg.decoder->state.phase_valid == true))
    {
        /*
        sequential mode
        */

        //coil #1
        if((Tuareg.actors.ignition_coil_1 == false) && (Tuareg.actors.ignition_scheduler_1 == false) && (Tuareg.decoder->phase == PHASE_CYL1_COMP))
        {
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_sequential_us);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1SCHED_POWER);
        }

        //coil #2
        if((Tuareg.actors.ignition_coil_2 == false) && (Tuareg.actors.ignition_scheduler_2 == false) && (Tuareg.decoder->phase == PHASE_CYL1_EX))
        {
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_sequential_us);

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
        scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_batch_us);

        //coil #2
        scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_POWERED, Tuareg.ignition_controls.dwell_timing_batch_us);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN1_POWER);
        ignition_diag_log_event(IGNDIAG_IRQ3H_IGN2_POWER);
    }

}

