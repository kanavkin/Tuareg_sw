#include <Tuareg_platform.h>
#include <Tuareg.h>


const U32 cIgnition_min_dyn_rpm= 500;

const F32 cIgn_max_Dwell_us= 10000.0;
const F32 cIgn_min_Dwell_us= 0.0;




/****************************************************************************************************************************************
*   Ignition controls update helper functions
****************************************************************************************************************************************/

void clear_ignition_controls()
{
    //test
    memclr_boctok((void *) &(Tuareg.Controls.Ignition), sizeof(ignition_controls_t));

    /*
    //suspend ignition
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= CRK_POSITION_UNDEFINED;

    //dwell
    pTarget->dwell_timing_us =0;
    pTarget->dwell_us= 0;
    pTarget->dwell_pos= CRK_POSITION_UNDEFINED;

    //status data
    pTarget->flags.all_flags= 0;
    */
}


const U32 cCranking_Reported_Dwell_us= 10000;


void cranking_ignition_controls()
{
    //ignition
    Tuareg.Controls.Ignition.ignition_timing_us= 0;
    Tuareg.Controls.Ignition.ignition_pos= Ignition_Setup.cranking_ignition_position;

    //dwell
    Tuareg.Controls.Ignition.dwell_timing_us =0;
    Tuareg.Controls.Ignition.dwell_us= cCranking_Reported_Dwell_us;
    Tuareg.Controls.Ignition.dwell_pos= Ignition_Setup.cranking_dwell_position;

    //status data
    Tuareg.Controls.Ignition.flags.all_flags= 0;
    Tuareg.Controls.Ignition.flags.valid= true;
 }




/**
calculates the ignition timing for the next engine cycle at a given rpm

dynamic ignition function activated

preconditions:
- Tuareg.pDecoder.state.rpm_valid := true
- ignition config error not present
- rpm > dynamic min rpm
- ignition flags cleared
*/
void dynamic_ignition_controls()
{
    F32 Ign_advance_deg, Trigger_advance_deg, Timing_target_us, Dwell_target_us, Dwell_target_timing_us;

    /**
    preload the commanded target ignition advance angle and look up the corresponding dwell
    */
    Ign_advance_deg= Tuareg.Controls.IgnAdv_deg;
    Dwell_target_us= getValue_ignDwellTable(Tuareg.Decoder.crank_rpm);

    /**
    dwell validity check
    */
    if(Dwell_target_us > cIgn_max_Dwell_us)
    {
        //clamp to max
        Dwell_target_us= cIgn_max_Dwell_us;
        Limp(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_DWELL);
    }
    else if(Dwell_target_us < cIgn_min_Dwell_us)
    {
        //clamp to min
        Dwell_target_us= cIgn_min_Dwell_us;
        Limp(TID_IGNITION_CONFIG, IGNITION_LOC_DYN_CTRLS_INVALID_DWELL);
    }

    /**
    check for cold idle conditions
    */
    if( (Tuareg.Decoder.crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) &&
       (Tuareg.errors.sensor_CLT_error == false) && (Tuareg.process.CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) &&
       (Ignition_Setup.flags.cold_idle_enabled == true) )
    {

        /**
        all preconditions for cold idle function are fulfilled,
        now check if it has been configured properly
        */
        if( (Ignition_Setup.cold_idle_ignition_advance_deg >= cMin_Adv_val) &&
            (Ignition_Setup.cold_idle_ignition_advance_deg <= cMax_Adv_val) &&
            (Ignition_Setup.cold_idle_dwell_target_us >= cIgn_min_Dwell_us) &&
            (Ignition_Setup.cold_idle_dwell_target_us <= cIgn_max_Dwell_us) )
        {
            //cold idle function activated
            Tuareg.Controls.Ignition.flags.cold_idle= true;
            Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
            Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;
        }
        else
        {
            //cold idle configuration error
            Syslog_Error(TID_IGNITION_CONFIG, IGNITION_LOC_COLD_IDLE_CONFIG_INVALID);
        }

    }


    /**
    check for sequential / batch mode capabilities
    */
    Tuareg.Controls.Ignition.flags.sequential_mode= ((Tuareg.Decoder.flags.phase_valid == true) && (Ignition_Setup.flags.second_coil_installed == true) && (Ignition_Setup.flags.sequential_mode_enabled == true));

    //export static data
    Tuareg.Controls.Ignition.ignition_pos= Ignition_Setup.dynamic_ignition_base_position;


    /******************************************************
    * calculate ignition timing
    *****************************************************/


    //look up the advance angle of the trigger point
    Trigger_advance_deg= Tuareg_Setup.trigger_advance_map[Ignition_Setup.dynamic_ignition_base_position];

    //check if inserting a delay is necessary
    if(Trigger_advance_deg > Ign_advance_deg)
    {
        Timing_target_us= calc_rot_duration_us( Trigger_advance_deg - Ign_advance_deg, Tuareg.Decoder.crank_period_us);

        //subtract VR introduced delay
        if(Timing_target_us > Tuareg_Setup.decoder_delay_us)
        {
            Tuareg.Controls.Ignition.ignition_timing_us= Timing_target_us - Tuareg_Setup.decoder_delay_us;
        }
        else
        {
            Tuareg.Controls.Ignition.ignition_timing_us= 0.0;
        }
    }
    else
    {
        Tuareg.Controls.Ignition.ignition_timing_us= 0.0;
    }



    /**
    dwell
    ignition timing is implemented via 2 interval scheduler allocation
    1. interval: base position -- ignition
    2. interval ignition -- dwell

    -> the minimum delay is the spark duration ->  dwell timing > spark_duration!
    -> the dwell timing begins when the sceduler turns off the coil for spark
    */
    if(Tuareg.Controls.Ignition.flags.sequential_mode == true)
    {
        //sequential mode: delay := T720 -> max dwell duration
        Dwell_target_timing_us= 2* Tuareg.Decoder.crank_period_us - Dwell_target_us;

        //check if the spark duration time is achieved
        if(Dwell_target_timing_us < Ignition_Setup.spark_duration_us)
        {
            //set dwell beginning right after the spark has extinguished
            Tuareg.Controls.Ignition.dwell_timing_us= Ignition_Setup.spark_duration_us;

            //report the resulting dwell time
            Tuareg.Controls.Ignition.dwell_us= 2* Tuareg.Decoder.crank_period_us - Ignition_Setup.spark_duration_us;
        }
        else
        {
            Tuareg.Controls.Ignition.dwell_timing_us= Dwell_target_timing_us;
            Tuareg.Controls.Ignition.dwell_us= Dwell_target_us;
        }

    }
    else
    {
        //sequential mode: delay := T360 -> max dwell duration
        Dwell_target_timing_us= Tuareg.Decoder.crank_period_us - Dwell_target_us;

        //check if the spark duration time is achieved
        if(Dwell_target_timing_us < Ignition_Setup.spark_duration_us)
        {
            //set dwell beginning right after the spark has extinguished
            Tuareg.Controls.Ignition.dwell_timing_us= Ignition_Setup.spark_duration_us;

            //report the resulting dwell time
            Tuareg.Controls.Ignition.dwell_us= Tuareg.Decoder.crank_period_us - Ignition_Setup.spark_duration_us;
        }
        else
        {
            Tuareg.Controls.Ignition.dwell_timing_us= Dwell_target_timing_us;
            Tuareg.Controls.Ignition.dwell_us= Dwell_target_us;
        }
    }

    //dwell crank position not needed in dynamic mode
    Tuareg.Controls.Ignition.dwell_pos= CRK_POSITION_UNDEFINED;

    //enable controls
    Tuareg.Controls.Ignition.flags.dynamic_controls= true;
    Tuareg.Controls.Ignition.flags.valid= true;
}


/****************************************************************************************************************************************
* Ignition controls update
* calculates the ignition timing according to the current system state and run mode
****************************************************************************************************************************************/

void Tuareg_update_ignition_controls()
{
    //clear current ignition controls
    clear_ignition_controls();

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_CALLS);

    //check controls
    if(Tuareg.Controls.Flags.valid == false)
    {
        //invalid ignition controls
        return;
    }

    //check if engine is cranking
    if(Tuareg.flags.cranking == true)
    {
        cranking_ignition_controls();
        return;
    }

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_UPDIGNCTRL_DYN);

    //calculate dynamic ignition controls
    dynamic_ignition_controls();
}
