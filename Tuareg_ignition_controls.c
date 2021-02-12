#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "decoder_hw.h"
#include "decoder_logic.h"
#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "ignition_config.h"
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



/****************************************************************************************************************************************
*   Ignition controls update
****************************************************************************************************************************************/


/**
calculates the ignition timing according to the current system state and run mode

fallback strategy is to use default_ignition_controls
*/
void Tuareg_update_ignition_controls()
{
    exec_result_t result;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_CALLS);


    if((Tuareg.decoder->outputs.rpm_valid == false) || (Tuareg.decoder->outputs.timeout == true) || (Tuareg.Runmode == TMODE_LIMP) || (Tuareg.Runmode == TMODE_STB))
    {
        default_ignition_controls();
        return;
    }

    if(Tuareg.decoder->crank_rpm > Ignition_Setup.max_rpm)
    {
        revlimiter_ignition_controls();

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_REVLIM);

        return;
    }

    if((Tuareg.decoder->crank_rpm < Ignition_Setup.dynamic_min_rpm) || (Tuareg.Runmode == TMODE_CRANKING))
    {
        cranking_ignition_controls();
        return;
    }

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN);

    //proceed with dynamic ignition
    result= dynamic_ignition_controls();

    if(result != EXEC_OK)
    {
        //handle failure
        default_ignition_controls();

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN_FAIL);
    }

}


/****************************************************************************************************************************************
*   Ignition controls update helper functions
****************************************************************************************************************************************/


/**
provides a ignition timing which will allow limited engine operation e.g. while LIMP HOME
*/
inline void default_ignition_controls()
{
    //ignition
    Tuareg.ignition_controls.ignition_advance_deg= DEFAULT_IGNITION_ADVANCE_DEG;
    Tuareg.ignition_controls.ignition_timing_us= 0;
    Tuareg.ignition_controls.ignition_pos= DEFAULT_IGNITION_POSITION;

    //dwell
    Tuareg.ignition_controls.dwell_timing_sequential_us =0;
    Tuareg.ignition_controls.dwell_timing_batch_us =0;
    Tuareg.ignition_controls.dwell_sequential_us =0;
    Tuareg.ignition_controls.dwell_pos= DEFAULT_DWELL_POSITION;
    Tuareg.ignition_controls.dwell_batch_us= DEFAULT_REPORTED_DWELL_US;

    //status data
    Tuareg.ignition_controls.state.all_flags= 0;
    Tuareg.ignition_controls.state.default_controls= true;
    Tuareg.ignition_controls.state.valid= true;

}

/**
provides a late ignition timing for cranking
*/
inline void cranking_ignition_controls()
{
    //ignition
    Tuareg.ignition_controls.ignition_advance_deg= CRANKING_REPORTED_IGNITION_ADVANCE_DEG;
    Tuareg.ignition_controls.ignition_timing_us= 0;
    Tuareg.ignition_controls.ignition_pos= Ignition_Setup.cranking_ignition_position;

    //dwell
    Tuareg.ignition_controls.dwell_timing_sequential_us =0;
    Tuareg.ignition_controls.dwell_timing_batch_us =0;
    Tuareg.ignition_controls.dwell_sequential_us =0;
    Tuareg.ignition_controls.dwell_pos= DEFAULT_DWELL_POSITION;
    Tuareg.ignition_controls.dwell_batch_us= CRANKING_REPORTED_DWELL_US;

    //status data
    Tuareg.ignition_controls.state.all_flags= 0;
    Tuareg.ignition_controls.state.cranking_controls= true;
    Tuareg.ignition_controls.state.valid= true;

}


/**
rev limiter function activated
*/
inline void revlimiter_ignition_controls()
{
    //suspend ignition
    Tuareg.ignition_controls.ignition_advance_deg= REVLIMITER_REPORTED_IGNITION_ADVANCE_DEG;
    Tuareg.ignition_controls.ignition_timing_us= 0;
    Tuareg.ignition_controls.ignition_pos= CRK_POSITION_UNDEFINED;

    //dwell
    Tuareg.ignition_controls.dwell_timing_sequential_us =0;
    Tuareg.ignition_controls.dwell_timing_batch_us =0;
    Tuareg.ignition_controls.dwell_sequential_us =0;
    Tuareg.ignition_controls.dwell_pos= CRK_POSITION_UNDEFINED;
    Tuareg.ignition_controls.dwell_batch_us= REVLIMITER_REPORTED_DWELL_US;

    //status data
    Tuareg.ignition_controls.state.all_flags= 0;
    Tuareg.ignition_controls.state.rev_limiter= true;
    Tuareg.ignition_controls.state.valid= true;
}


/**
calculates the ignition timing for the next engine cycle at a given rpm

dynamic ignition function activated

preconditions:
- Tuareg.decoder.state.rpm_valid := true
- Process table valid
*/
inline exec_result_t dynamic_ignition_controls()
{
    /// TODO (oli#1#):test ignition calculation

    VU32 Ign_advance_deg, Dwell_target_us, dwell_avail_us;
    process_position_t ignition_POS;
    exec_result_t result;


    //set status data
    Tuareg.ignition_controls.state.all_flags= 0;
    Tuareg.ignition_controls.state.dynamic_controls= true;


    /**
    select ignition advance and dwell
    */
    if( (Tuareg.decoder->crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) && (Tuareg.process.CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) )
    {
        //cold idle function activated
        Tuareg.ignition_controls.state.cold_idle= true;

        Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
        Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;

    }
    else
    {
        ///get ignition advance from table

        //set status bit
        Tuareg.ignition_controls.state.advance_tps= true;

        //get target ignition advance angle
        //Ign_advance_deg= table3D_getValue(&ignitionTable_TPS, pImage->crank_rpm, pImage->TPS_deg);

        /// TODO (oli#3#): tps readout not stable yet
        Ign_advance_deg= getValue_ignAdvTable_TPS(Tuareg.decoder->crank_rpm, 30);


        ///get dwell from table

        /// TODO (oli#1#): dwell logic hacked! shall be replaced by a proper target dwell calculation/table soon!

        //get target dwell duration
        if(Tuareg.decoder->crank_rpm < 2000)
        {
            Dwell_target_us = 10000;
        }
        else
        {
            Dwell_target_us = Ignition_Setup.dynamic_dwell_target_us;
        }

    }

    /**
    check for sequential / batch mode capabilities
    */
    if((Tuareg.decoder->outputs.phase_valid == true) && (Ignition_Setup.coil_setup == COILS_SEPARATE))
    {
        Tuareg.ignition_controls.state.sequential_mode= true;
    }

    /******************************************************
    * prepare the ignition control object
    *****************************************************/

    //export static data
    Tuareg.ignition_controls.ignition_pos= Ignition_Setup.dynamic_ignition_base_position;
    Tuareg.ignition_controls.ignition_advance_deg= Ign_advance_deg;

    //prepare transfer object for process table lookup
    ignition_POS.crank_pos= Ignition_Setup.dynamic_ignition_base_position;
    ignition_POS.phase= PHASE_CYL1_COMP;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&ignition_POS);

    ASSERT_EXEC_OK(result);

    //ignition_timing_us reflects the scheduler delay to set up
    Tuareg.ignition_controls.ignition_timing_us= calc_rot_duration_us( subtract_VU32(ignition_POS.base_PA, Ign_advance_deg), Tuareg.decoder->crank_period_us);


    /**
    dwell
    the minimum delay is the spark duration
    sequential mode: delay := T720 - spark duration - target dwell duration
    batch mode: delay := T360 - spark duration - target dwell duration
    */
    Tuareg.ignition_controls.dwell_timing_sequential_us= Ignition_Setup.spark_duration_us;
    dwell_avail_us= subtract_VU32( 2* Tuareg.decoder->crank_period_us, Ignition_Setup.spark_duration_us);
    Tuareg.ignition_controls.dwell_timing_sequential_us += subtract_VU32( dwell_avail_us, Dwell_target_us);
    Tuareg.ignition_controls.dwell_sequential_us= subtract_VU32( 2* Tuareg.decoder->crank_period_us, Tuareg.ignition_controls.dwell_timing_sequential_us);


    Tuareg.ignition_controls.dwell_timing_batch_us= Ignition_Setup.spark_duration_us;
    dwell_avail_us= subtract_VU32( Tuareg.decoder->crank_period_us, Ignition_Setup.spark_duration_us);
    Tuareg.ignition_controls.dwell_timing_batch_us += subtract_VU32( dwell_avail_us, Dwell_target_us);
    Tuareg.ignition_controls.dwell_batch_us= subtract_VU32( Tuareg.decoder->crank_period_us, Tuareg.ignition_controls.dwell_timing_batch_us);


    //enable controls
    Tuareg.ignition_controls.state.valid= true;

    //finish
    return result;
}


