#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
//#include "decoder_logic.h"
#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"
#include "ignition_hw.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"

#include "ignition_config.h"

//#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"


const U32 cDynamic_min_rpm= 500;



/****************************************************************************************************************************************
*   Ignition controls update
****************************************************************************************************************************************/


/**
calculates the ignition timing according to the current system state and run mode

fallback strategy is to use default_ignition_controls
*/
void Tuareg_update_ignition_controls()
{
    volatile ignition_controls_t * pTarget= &(Tuareg.ignition_controls);

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_CALLS);

    //check operational preconditions
    if((Tuareg.flags.run_inhibit == true) || (Tuareg.flags.standby == true))
    {
        //do not provide valid ignition controls
        default_ignition_controls(pTarget);
        pTarget->flags.valid= false;
        return;
    }

    //check if rev limiter has been triggered
    if(Tuareg.flags.rev_limiter == true)
    {
        revlimiter_ignition_controls(pTarget);

        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_REVLIM);

        return;
    }

    //check preconditions for ignition controls calculations
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.flags.limited_op == true) || (Tuareg.errors.ignition_config_error == true))
    {
        default_ignition_controls(pTarget);
        return;
    }

    //check if engine is cranking
    if((Tuareg.flags.cranking == true) && (Ignition_Setup.flags.cranking_controls_enabled == true))
    {
        cranking_ignition_controls(pTarget);
        return;
    }

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN);

    //check if dynamic ignition is enabled
    if(Ignition_Setup.flags.dynamic_controls_enabled == true)
    {
        dynamic_ignition_controls(pTarget);

        //check if dynamic ignition calculation has succeeded
        if(pTarget->flags.valid == false)
        {
            //handle failure
            default_ignition_controls(pTarget);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN_FAIL);
        }
    }
    else
    {
        default_ignition_controls(pTarget);
    }

}


/****************************************************************************************************************************************
*   Ignition controls update helper functions
****************************************************************************************************************************************/


/**
provides a ignition timing which will allow limited engine operation e.g. while LIMP HOME
*/
void default_ignition_controls(volatile ignition_controls_t * pTarget)
{
    //ignition
    pTarget->ignition_advance_deg= DEFAULT_IGNITION_ADVANCE_DEG;
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= DEFAULT_IGNITION_POSITION;

    //dwell
    pTarget->dwell_timing_us =0;
    pTarget->dwell_pos= DEFAULT_DWELL_POSITION;
    pTarget->dwell_us= DEFAULT_REPORTED_DWELL_US;

    //status data
    pTarget->flags.all_flags= 0;
    pTarget->flags.valid= true;
}


/**
rev limiter function activated
*/
void revlimiter_ignition_controls(volatile ignition_controls_t * pTarget)
{
    //suspend ignition
    pTarget->ignition_advance_deg= 0;
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= CRK_POSITION_UNDEFINED;

    //dwell
    pTarget->dwell_timing_us =0;
    pTarget->dwell_us= 0;
    pTarget->dwell_pos= CRK_POSITION_UNDEFINED;

    //status data
    pTarget->flags.all_flags= 0;
    pTarget->flags.valid= true;
}

void cranking_ignition_controls(volatile ignition_controls_t * pTarget)
{
    //ignition
    Tuareg.ignition_controls.ignition_advance_deg= CRANKING_REPORTED_IGNITION_ADVANCE_DEG;
    Tuareg.ignition_controls.ignition_timing_us= 0;
    Tuareg.ignition_controls.ignition_pos= Ignition_Setup.cranking_ignition_position;

    //dwell
    Tuareg.ignition_controls.dwell_timing_us =0;
    Tuareg.ignition_controls.dwell_us= CRANKING_REPORTED_DWELL_US;
    Tuareg.ignition_controls.dwell_pos= Ignition_Setup.cranking_dwell_position;

    //status data
    pTarget->flags.all_flags= 0;
    pTarget->flags.valid= true;
 }



/**
calculates the ignition timing for the next engine cycle at a given rpm

dynamic ignition function activated

preconditions:
- Tuareg.pDecoder.state.rpm_valid := true
- Process table valid
- ignition config error not present
*/
void dynamic_ignition_controls(volatile ignition_controls_t * pTarget)
{
    VU32 Ign_advance_deg, Dwell_target_us, dwell_avail_us;
    process_position_t ignition_POS;
    exec_result_t result;

    //initialize flags
    pTarget->flags.all_flags= 0;

    //check precondition - min rpm
    if(Tuareg.pDecoder->crank_rpm < cDynamic_min_rpm)
    {
        return;
    }

    /**
    select ignition advance and dwell
    */
    if( (Tuareg.pDecoder->crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) && (Tuareg.process.CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) && (Ignition_Setup.flags.cold_idle_enabled == true) )
    {
        //cold idle function activated
        pTarget->flags.cold_idle= true;

        Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
        Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;

    }
    else
    {
        //get target ignition advance angle - TPS default value will be sufficient, in case
        Ign_advance_deg= getValue_ignAdvTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);

        ///get dwell from table
        Dwell_target_us= getValue_ignDwellTable(Tuareg.pDecoder->crank_rpm);
    }

    /**
    check for sequential / batch mode capabilities
    */
    pTarget->flags.sequential_mode= ((Tuareg.pDecoder->outputs.phase_valid == true) && (Ignition_Setup.flags.second_coil_installed == true));

    /******************************************************
    * prepare the ignition control object
    *****************************************************/

    //export static data
    pTarget->ignition_pos= Ignition_Setup.dynamic_ignition_base_position;
    pTarget->ignition_advance_deg= Ign_advance_deg;

    //prepare transfer object for process table lookup
    ignition_POS.crank_pos= Ignition_Setup.dynamic_ignition_base_position;
    ignition_POS.phase= PHASE_CYL1_COMP;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&ignition_POS);

    ASSERT_EXEC_OK_VOID(result);

    //ignition_timing_us reflects the scheduler delay to set up
    pTarget->ignition_timing_us= calc_rot_duration_us( subtract_VU32(ignition_POS.base_PA, Ign_advance_deg), Tuareg.pDecoder->crank_period_us);


    /**
    dwell
    */
    if(pTarget->flags.sequential_mode == true)
    {
        //sequential mode: delay := T720 - spark duration - target dwell duration
        dwell_avail_us= subtract_VU32( 2* Tuareg.pDecoder->crank_period_us, Ignition_Setup.spark_duration_us);
    }
    else
    {
        //batch mode: delay := T360 - spark duration - target dwell duration
        dwell_avail_us= subtract_VU32( Tuareg.pDecoder->crank_period_us, Ignition_Setup.spark_duration_us);
    }

    //the minimum delay is the spark duration
    pTarget->dwell_timing_us= Ignition_Setup.spark_duration_us + subtract_VU32( dwell_avail_us, Dwell_target_us);
    pTarget->dwell_us= subtract_VU32( 2* Tuareg.pDecoder->crank_period_us, pTarget->dwell_timing_us);

    //dwell crank position not needed in dynamic mode
    pTarget->dwell_pos= CRK_POSITION_UNDEFINED;

    //enable controls
    pTarget->flags.dynamic_controls= true;
    pTarget->flags.valid= true;
}


