#include <Tuareg_platform.h>
#include <Tuareg.h>


const U32 cDynamic_min_rpm= 500;

const F32 cIgn_max_Dwell_us= 10000.0;
const F32 cIgn_min_Dwell_us= 0.0;




/****************************************************************************************************************************************
*   Ignition controls update helper functions
****************************************************************************************************************************************/


/**
rev limiter function activated
*/
void clear_ignition_controls(volatile ignition_controls_t * pTarget)
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
}

void cranking_ignition_controls(volatile ignition_controls_t * pTarget)
{
    //ignition
    Tuareg.Tuareg_controls.ignition_controls.ignition_advance_deg= CRANKING_REPORTED_IGNITION_ADVANCE_DEG;
    Tuareg.Tuareg_controls.ignition_controls.ignition_timing_us= 0;
    Tuareg.Tuareg_controls.ignition_controls.ignition_pos= Ignition_Setup.cranking_ignition_position;

    //dwell
    Tuareg.Tuareg_controls.ignition_controls.dwell_timing_us =0;
    Tuareg.Tuareg_controls.ignition_controls.dwell_us= CRANKING_REPORTED_DWELL_US;
    Tuareg.Tuareg_controls.ignition_controls.dwell_pos= Ignition_Setup.cranking_dwell_position;

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
- rpm > dynamic min rpm
- ignition flags cleared
*/
void dynamic_ignition_controls(volatile ignition_controls_t * pTarget)
{
    F32 Ign_advance_deg, Trigger_advance_deg, Timing_target_us, Dwell_target_us, Dwell_target_timing_us;

    /**
    select ignition advance and dwell
    */
    if( (Tuareg.pDecoder->crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) &&
       (Tuareg.errors.sensor_CLT_error == false) && (Tuareg.process.CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) &&
       (Ignition_Setup.flags.cold_idle_enabled == true) )
    {
        //cold idle function activated
        pTarget->flags.cold_idle= true;

        /**
        configured cold idle ignition advance angle validity check
        */
        if(Ignition_Setup.cold_idle_ignition_advance_deg > cMax_Adv_val)
        {
            //clamp to max
            Ign_advance_deg= cMax_Adv_val;

            //log error
            Syslog_Error(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_ADVANCE);
        }
        else if(Ignition_Setup.cold_idle_ignition_advance_deg < cMin_Adv_val)
        {
            //clamp to min
            Ign_advance_deg= cMin_Adv_val;

            //log error
            Syslog_Error(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_ADVANCE);
        }
        else
        {
            Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
        }

        //use configured cold idle dwell value
        Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;
    }
    else
    {
        //use validated target ignition advance angle from control set
        Ign_advance_deg= Tuareg.Tuareg_controls.IgnAdv_deg;

        //get dwell from table
        Dwell_target_us= getValue_ignDwellTable(Tuareg.pDecoder->crank_rpm);
    }





    /**
    dwell validity check
    */
    if(Dwell_target_us > cIgn_max_Dwell_us)
    {
        //clamp to max
        Dwell_target_us= cIgn_max_Dwell_us;

        //log error
        Syslog_Error(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_DWELL);
    }
    else if(Dwell_target_us < cIgn_min_Dwell_us)
    {
        //clamp to min
        Dwell_target_us= cIgn_min_Dwell_us;

        //log error
        Syslog_Error(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_DWELL);
    }


    /**
    check for sequential / batch mode capabilities
    */
    pTarget->flags.sequential_mode= ((Tuareg.pDecoder->flags.phase_valid == true) && (Ignition_Setup.flags.second_coil_installed == true) && (Ignition_Setup.flags.sequential_mode_enabled == true));

    //export static data
    pTarget->ignition_pos= Ignition_Setup.dynamic_ignition_base_position;
    pTarget->ignition_advance_deg= Ign_advance_deg;


    /******************************************************
    * calculate ignition timing
    *****************************************************/


    //look up the advance angle of the trigger point
    Trigger_advance_deg= Tuareg_Setup.trigger_advance_map[Ignition_Setup.dynamic_ignition_base_position];

    //check if inserting a delay is necessary
    if(Trigger_advance_deg > Ign_advance_deg)
    {
        Timing_target_us= calc_rot_duration_us( Trigger_advance_deg - Ign_advance_deg, Tuareg.pDecoder->crank_period_us);

        //subtract VR introduced delay
        if(Timing_target_us > Tuareg_Setup.decoder_delay_us)
        {
            pTarget->ignition_timing_us= Timing_target_us - Tuareg_Setup.decoder_delay_us;
        }
        else
        {
            pTarget->ignition_timing_us= 0.0;
        }
    }
    else
    {
        pTarget->ignition_timing_us= 0.0;
    }



    /**
    dwell
    ignition timing is implemented via 2 interval scheduler allocation
    1. interval: base position -- ignition
    2. interval ignition -- dwell

    -> the minimum delay is the spark duration ->  dwell timing > spark_duration!
    -> the dwell timing begins when the sceduler turns off the coil for spark
    */
    if(pTarget->flags.sequential_mode == true)
    {
        //sequential mode: delay := T720 -> max dwell duration
        Dwell_target_timing_us= 2* Tuareg.pDecoder->crank_period_us - Dwell_target_us;

        //check if the spark duration time is achieved
        if(Dwell_target_timing_us < Ignition_Setup.spark_duration_us)
        {
            //set dwell beginning right after the spark has extinguished
            pTarget->dwell_timing_us= Ignition_Setup.spark_duration_us;

            //report the resulting dwell time
            pTarget->dwell_us= 2* Tuareg.pDecoder->crank_period_us - Ignition_Setup.spark_duration_us;
        }
        else
        {
            pTarget->dwell_timing_us= Dwell_target_timing_us;
            pTarget->dwell_us= Dwell_target_us;
        }

    }
    else
    {
        //sequential mode: delay := T360 -> max dwell duration
        Dwell_target_timing_us= Tuareg.pDecoder->crank_period_us - Dwell_target_us;

        //check if the spark duration time is achieved
        if(Dwell_target_timing_us < Ignition_Setup.spark_duration_us)
        {
            //set dwell beginning right after the spark has extinguished
            pTarget->dwell_timing_us= Ignition_Setup.spark_duration_us;

            //report the resulting dwell time
            pTarget->dwell_us= Tuareg.pDecoder->crank_period_us - Ignition_Setup.spark_duration_us;
        }
        else
        {
            pTarget->dwell_timing_us= Dwell_target_timing_us;
            pTarget->dwell_us= Dwell_target_us;
        }
    }

    //dwell crank position not needed in dynamic mode
    pTarget->dwell_pos= CRK_POSITION_UNDEFINED;

    //enable controls
    pTarget->flags.dynamic_controls= true;
    pTarget->flags.valid= true;
}


/****************************************************************************************************************************************
*   Ignition controls update
****************************************************************************************************************************************/


/**
calculates the ignition timing according to the current system state and run mode

fallback strategy is to use default_ignition_controls
*/
void Tuareg_update_ignition_controls(volatile ignition_controls_t * pTarget)
{
    //clear current ignition controls
    clear_ignition_controls(pTarget);

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_CALLS);

    //check controls
    if(Tuareg.Tuareg_controls.Flags.valid == false)
    {
        //invalid ignition controls
        return;
    }

    //check if engine is cranking
    if((Tuareg.flags.cranking == true) || (Tuareg.pDecoder->crank_rpm < cDynamic_min_rpm))
    {
        cranking_ignition_controls(pTarget);
        return;
    }

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN);

    //calculate dynamic ignition controls
    dynamic_ignition_controls(pTarget);
}
