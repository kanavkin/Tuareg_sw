#include <Tuareg_platform.h>
#include <Tuareg.h>

/******************************************************************************************************************************
evaluates the data gathered from the cylinder identification sensor:
the resulting cis signal shall confirm the currently expected phase

BIG FAT WARNING:
Phase calculation relies on the timer value growing between lobe begin and end detection.
The decoder timer always gets reset on Position B2!
******************************************************************************************************************************/
void decoder_update_cis()
{
    engine_phase_t detected_phase;
    U32 lobe_angle_deg =0, lobe_interval_us =0;

    #ifdef DECODER_CIS_DEBUG
    decoder_cis_debug_next_cycle();
    #endif // DECODER_CIS_DEBUG

    //collect diagnostic information
    decoder_diag_log_event(DDIAG_CISUPD_CALLS);

    //precondition check
    if((Decoder.out.flags.period_valid == false) ||
       ((Decoder.cis.flags.lobe_begin_detected == true) && (Decoder.cis.flags.lobe_end_detected == false)) ||
       ((Decoder.cis.flags.lobe_begin_detected == false) && (Decoder.cis.flags.lobe_end_detected == true)) ||
       (Decoder.cis.flags.failure == true) )
    {
        //collect additional state information
        Decoder.cis.flags.period_valid= Decoder.out.flags.period_valid;
        Decoder.cis.flags.preconditions_ok= false;
        Decoder.cis.flags.triggered= false;
        Decoder.cis.flags.phase_match= false;
        Decoder.cis.flags.phase_valid= false;

        //phase information invalid
        Decoder.out.phase= PHASE_UNDEFINED;
        Decoder.out.flags.phase_valid= false;
        Tuareg.errors.sensor_CIS_error= true;
        Decoder.cis.sync_counter= 0;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PRECOND_FAIL);

        #ifdef DECODER_CIS_DEBUG
        decoder_update_cis_debug(lobe_interval_us, lobe_angle_deg);
        #endif // DECODER_CIS_DEBUG

        return;
    }
    else
    {
        //collect additional state information
        Decoder.cis.flags.preconditions_ok= true;
    }


    /*******************************************
    signal validation -> the cis has been triggered if the cam lobe has been detected for more then cis_min_angle_deg
    *******************************************/

    //default estimation: sensor has not been triggered
    detected_phase= opposite_phase(Decoder_Setup.cis_triggered_phase);
    Decoder.cis.flags.triggered= false;

    //check if the cis has detected a rising and a falling signal edge
    if( (Decoder.cis.flags.lobe_begin_detected == true) && (Decoder.cis.flags.lobe_end_detected == true))
    {
        lobe_interval_us= Decoder_hw.timer_period_us * subtract_U32(Decoder.cis.lobe_end_timestamp, Decoder.cis.lobe_begin_timestamp);

        lobe_angle_deg= calc_rot_angle_deg(lobe_interval_us, Decoder.out.crank_period_us);

        //check if lobe interval is longer than the defined minimum
        if(lobe_angle_deg > Decoder_Setup.cis_min_angle_deg)
        {
            //signal edges have been detected AND the interval is valid
            detected_phase= Decoder_Setup.cis_triggered_phase;

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CISUPD_TRIGGERED);

            //collect additional state information
            Decoder.cis.flags.triggered= true;
        }
    }


    /*******************************************
    evaluation -> the cis signal shall confirm the currently expected phase
    *******************************************/

    //check if the resulting cis signal confirms the currently expected phase
    if(Decoder.out.phase == detected_phase)
    {
        /**
        GOOD
        */

        //collect additional state information
        Decoder.cis.flags.phase_match= true;

        //check if the required amount of valid samples has been detected
        if(Decoder.cis.sync_counter < Decoder_Setup.cis_sync_thres)
        {
            Decoder.cis.sync_counter += 1;
        }
        else
        {
            //stable cis signal detected
            Decoder.out.flags.phase_valid= true;
            Tuareg.errors.sensor_CIS_error= false;
            Decoder.cis.flags.phase_valid= true;
        }

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_PASS);
    }
    else
    {
        /**
        BAD
        */
        Decoder.cis.sync_counter =0;
        Decoder.out.flags.phase_valid= false;
        Tuareg.errors.sensor_CIS_error= true;

        //expected engine phase information requires updating
        Decoder.out.phase= detected_phase;

        //collect additional state information
        Decoder.cis.flags.phase_match= false;
        Decoder.cis.flags.phase_valid= false;


        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_FAIL);
    }

    #ifdef DECODER_CIS_DEBUG
    decoder_update_cis_debug(lobe_interval_us, lobe_angle_deg);
    #endif // DECODER_CIS_DEBUG

}



/******************************************************************************************************************************
cylinder identification sensor handling - irq handler
called from decoder_hw module when the corresponding EXTI has been triggered
******************************************************************************************************************************/
void decoder_cis_handler()
{
    U32 timestamp;

    //precondition check
    if((Decoder_hw.state.timer_continuous_mode == false) || (Decoder.cis.flags.failure == true))
    {
        Decoder.cis.flags.failure= true;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISHDL_PRECOND_FAIL);

        return;
    }

    //get decoder timestamp
    timestamp= decoder_get_timestamp();

    //save decoder timestamp with respect to cam sensing
    if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_begin_sensing)
    {
        Decoder.cis.lobe_begin_timestamp= timestamp;
        Decoder.cis.flags.lobe_begin_detected= true;

        //invert cam sensing
        decoder_set_cis_sensing(Decoder_Setup.lobe_end_sensing);

        //notify high speed log
        highspeedlog_register_cis_lobe_begin();

        //collect diagnostic information
        decoder_diag_log_event(DDIAG_LOBE_BEG);

    }
    else if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_end_sensing)
    {
        Decoder.cis.lobe_end_timestamp= timestamp;
        Decoder.cis.flags.lobe_end_detected= true;

        /*
        tolerate CIS signal bouncing:
        - the first detected lobe begin is considered as actual begin
        - the last detected lobe end is considered as the actual lobe end
        -> allow multiple lobe end events
        */

        //notify high speed log
        highspeedlog_register_cis_lobe_end();

        //collect diagnostic information
        decoder_diag_log_event(DDIAG_LOBE_END);

        //count detected lobe end events
        Decoder.cis.detected_lobe_ends += 1;

    }
    else
    {
        //collect diagnostic information
        decoder_diag_log_event(DDIAG_INVALID_TRIG);
    }


    //update cis noise filter
    update_cam_noisefilter(timestamp);
}



/******************************************************************************************************************************
cylinder identification sensor handling - noise filter
******************************************************************************************************************************/
void decoder_cis_noisefilter_handler()
{
    decoder_unmask_cis_irq();
}



/******************************************************************************************************************************
cylinder identification sensor handling - high level functions
called from decoder logic module when the crank position has reached the cis enable / disable position
******************************************************************************************************************************/
void enable_cis()
{
    //set sensing for lobe beginning
    decoder_set_cis_sensing(Decoder_Setup.lobe_begin_sensing);

    //reset state data
    Decoder.cis.flags.failure= false;
    Decoder.cis.flags.lobe_begin_detected= false;
    Decoder.cis.flags.lobe_end_detected= false;
    Decoder.cis.detected_lobe_ends= 0;

    //collect diagnostic information
    decoder_diag_log_event(DDIAG_ENA_CIS);

    //enable irq
    decoder_unmask_cis_irq();
}


void disable_cis()
{
    //disable irq
    decoder_mask_cis_irq();

    //check the gathered cis data for phase information
    decoder_update_cis();
}











