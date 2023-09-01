#include <Tuareg_platform.h>
#include <Tuareg.h>

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
    if((Tuareg.pDecoder->flags.rpm_valid == false) || (Tuareg.flags.limited_op == true) || (Tuareg.errors.ignition_config_error == true))
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

const U32 cIgn_max_advance_deg= 70;
const U32 cIgn_min_advance_deg= 0;
const U32 cIgn_max_Dwell_target_us= 10000;
const U32 cIgn_min_Dwell_target_us= 0;

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
    if( (Tuareg.pDecoder->crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) &&
       (Tuareg.errors.sensor_CLT_error == false) && (Tuareg.process.CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) &&
       (Ignition_Setup.flags.cold_idle_enabled == true) )
    {
        //cold idle function activated
        pTarget->flags.cold_idle= true;

        Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
        Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;

    }
    else
    {
        /**
        TPS is required to look up ignition advance
        The ignition advance, associated with its default value will be selected to operate the engine.
        This could lead to engine damage - limit power output!
        */
        if(Tuareg.errors.sensor_TPS_error == true)
        {
            Limp(TID_IGNITION_CONFIG, IGNITION_LOC_TPS_ERROR);
        }
/// TODO (oli#1#09/01/23): implement control set

        //get target ignition advance angle - TPS default value will be sufficient, in case
        //Ign_advance_deg= getValue_ignAdvTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);
        Ign_advance_deg= Tuareg.Tuareg_controls.IgnAdv;

        ///get dwell from table
        Dwell_target_us= getValue_ignDwellTable(Tuareg.pDecoder->crank_rpm);
    }


    /**
    ignition parameters validity check
    */
    if((Ign_advance_deg > cIgn_max_advance_deg) || (Ign_advance_deg < cIgn_min_advance_deg))
    {
        Limp(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_ADVANCE);
    }

    if((Dwell_target_us < cIgn_min_Dwell_target_us) || (Dwell_target_us > cIgn_max_Dwell_target_us))
    {
        Limp(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_DWELL);
    }

    /**
    check for sequential / batch mode capabilities
    */
    pTarget->flags.sequential_mode= ((Tuareg.pDecoder->flags.phase_valid == true) && (Ignition_Setup.flags.second_coil_installed == true) && (Ignition_Setup.flags.sequential_mode_enabled == true));

    /******************************************************
    * prepare the ignition control object
    *****************************************************/

    //export static data
    pTarget->ignition_pos= Ignition_Setup.dynamic_ignition_base_position;
    pTarget->ignition_advance_deg= Ign_advance_deg;

    //prepare transfer object for process table lookup
    ignition_POS.crank_pos= Ignition_Setup.dynamic_ignition_base_position;
    ignition_POS.phase= PHASE_CYL1_COMP;
    ignition_POS.previous_cycle= false;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&ignition_POS);

    ASSERT_EXEC_OK_VOID(result);

    //ignition_timing_us reflects the scheduler delay to set up
    pTarget->ignition_timing_us= calc_rot_duration_us( subtract_U32(ignition_POS.base_PA, Ign_advance_deg), Tuareg.pDecoder->crank_period_us);


    /**
    dwell
    ignition timing is implemented via 2 interval scheduler allocation
    1. interval: base position -- ignition
    2. interval ignition -- dwell

    -> the minimum delay is the spark duration ->  dwell timing > spark_duration!
    */
    if(pTarget->flags.sequential_mode == true)
    {
        //sequential mode: delay := T720 - target dwell duration
        dwell_avail_us= subtract_U32( 2* Tuareg.pDecoder->crank_period_us, Ignition_Setup.spark_duration_us);

        pTarget->dwell_timing_us= Ignition_Setup.spark_duration_us + subtract_U32( dwell_avail_us, Dwell_target_us);

        pTarget->dwell_us= subtract_U32( 2* Tuareg.pDecoder->crank_period_us, pTarget->dwell_timing_us);
    }
    else
    {
        //batch mode: delay := T360 - target dwell duration
        dwell_avail_us= subtract_U32( Tuareg.pDecoder->crank_period_us, Ignition_Setup.spark_duration_us);

        pTarget->dwell_timing_us= Ignition_Setup.spark_duration_us + subtract_U32( dwell_avail_us, Dwell_target_us);

        /*
        if dwell is shorter than the resulting ignition timing, the scheduler will begin dwell at the iginition base position
        */
        if(pTarget->ignition_timing_us + pTarget->dwell_timing_us < Tuareg.pDecoder->crank_period_us)
        {
            pTarget->dwell_us= subtract_U32( Tuareg.pDecoder->crank_period_us, pTarget->dwell_timing_us);
        }
        else
        {
            pTarget->dwell_us= pTarget->ignition_timing_us;
        }
    }


    //dwell crank position not needed in dynamic mode
    pTarget->dwell_pos= CRK_POSITION_UNDEFINED;

    //enable controls
    pTarget->flags.dynamic_controls= true;
    pTarget->flags.valid= true;
}


