#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "decoder_hw.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fuel_hw.h"
#include "fuel_logic.h"

#include "dash_hw.h"
#include "dash_logic.h"
#include "act_hw.h"
#include "act_logic.h"


#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"

#include "module_test.h"

/**
emits the control events to actor (scheduler / coil) layer
*/
void Tuareg_trigger_ignition_actors(volatile crank_position_t CrankPosition, volatile engine_phase_t Phase, volatile ignition_control_t * pIgnitionControls)
{
    VU32 age_us, corr_timing_us;

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_TRIG_IGN_CALLS);

    if((Phase < PHASE_UNDEFINED) && (pIgnitionControls->state.dynamic == true) )
    {
        /**
        normal operation with engine phase information available
        */

        /*
        handle ignition events
        */
        if(CrankPosition == pIgnitionControls->ignition_pos)
        {
            //compensate timing for execution delay
            age_us= decoder_get_data_age_us();
            corr_timing_us= subtract_VU32(pIgnitionControls->ignition_timing_us, age_us);

            //check coil setup
            if(configPage13.coil_setup == COILS_SEPARATE)
            {
                ///ignition event for cylinder #1 or #2
                if(Phase == PHASE_CYL1_COMP)
                {
                    scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us);

                    //collect diagnostic information
                    tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN1);
                }
                else
                {
                    scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us);

                    //collect diagnostic information
                    tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN2);
                }

            }
            else
            {
                ///ignition event for cylinder #1 and #2
                scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us);
                scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us);

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN1);
                tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN2);
            }

        }

        /*
        handle dwell events
        */
        if(CrankPosition == pIgnitionControls->dwell_pos_phased)
        {
            //check coil setup
            if(configPage13.coil_setup == COILS_SEPARATE)
            {
                ///dwell event for cylinder #1 or #2
                if(Phase == pIgnitionControls->dwell_phase_cyl1)
                {
                    set_ignition_ch1(ACTOR_POWERED);

                    //collect diagnostic information
                    tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL1);
                }
                else
                {
                    set_ignition_ch2(ACTOR_POWERED);

                    //collect diagnostic information
                    tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL2);
                }
            }
            else
            {
                ///dwell event for cylinder #1 and #2
                set_ignition_ch1(ACTOR_POWERED);
                set_ignition_ch2(ACTOR_POWERED);

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL1);
                tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL2);
            }
        }

    }
    else
    {
        /**
        no engine phase information available
        */

        /*
        handle ignition events
        */
        if(CrankPosition == pIgnitionControls->ignition_pos)
        {
            //compensate timing for execution delay
            age_us= decoder_get_data_age_us();
            corr_timing_us= subtract_VU32(pIgnitionControls->ignition_timing_us, age_us);

            //ignition event for cylinder #1 and #2
            scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, corr_timing_us);
            scheduler_set_channel(SCHEDULER_CH_IGN2, ACTOR_UNPOWERED, corr_timing_us);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN1);
            tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN2);
        }

        /*
        handle dwell events
        */
        if(CrankPosition == pIgnitionControls->dwell_pos_unphased)
        {
            //dwell event for cylinder #1 and #2
            set_ignition_ch1(ACTOR_POWERED);
            set_ignition_ch2(ACTOR_POWERED);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL1);
            tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL2);
        }

    } //phased/unphased

}


/**
calculates the ignition timing according to the current system state and run mode

The fact that the crank decoder will not provide any crank velocity information for the first 2..3 crank revolutions after getting sync will not upset
the ignition logic. Without a suitable engine_rpm figure, cranking_ignition_timing will be used
*/
void Tuareg_update_ignition_controls()
{
    exec_result_t result;

//was void Tuareg_update_ignition_timing()

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_IGNITIONCALC_CALLS);

    switch(Tuareg.Runmode)
     {

    case TMODE_CRANKING:
    case TMODE_RUNNING:
    case TMODE_STB:

        if(Tuareg.process.crank_rpm < configPage13.dynamic_min_rpm)
        {
            cranking_ignition_controls(&(Tuareg.ignition_controls));
        }
        else if(Tuareg.process.crank_rpm > configPage13.max_rpm)
        {
            revlimiter_ignition_controls(&(Tuareg.ignition_controls));
        }
        else
        {
            result= calculate_dynamic_ignition_controls( &(Tuareg.process), &(Tuareg.ignition_controls));

            if(result != EXEC_OK)
            {
                //handle failure
                default_ignition_controls(&(Tuareg.ignition_controls));
            }
        }

        break;


    default:

        /**
        possible scenario:
        - engine has been killed by kill switch and the crank shaft is still rotating
        - TMODE_LIMP
        */

        default_ignition_controls(&(Tuareg.ignition_controls));
        break;
     }
}

