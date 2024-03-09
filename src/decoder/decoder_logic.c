#include <Tuareg_platform.h>
#include <Tuareg.h>

volatile Tuareg_decoder_t Decoder;


/**
how the decoder works:

* the engine phase switches when the first position after TDC (CRK_POSITION_C1) is reached

* describe how the timer modes work

-   add description!!!
*/



/******************************************************************************************************************************
helper functions
******************************************************************************************************************************/

 void reset_timeout_counter()
{
    Tuareg.Decoder.flags.standstill= false;
    Decoder.timeout_count= 0;
}


void reset_timing_output()
{
    Tuareg.Decoder.flags.period_valid= false;
    Tuareg.Decoder.flags.rpm_valid= false;
    Tuareg.Decoder.flags.accel_valid= false;

    Tuareg.Decoder.crank_period_us= 0;
    Tuareg.Decoder.crank_rpm= 0;
    Tuareg.Decoder.crank_acceleration= 0;
}


void reset_position_data()
{
    Tuareg.Decoder.crank_position= CRK_POSITION_UNDEFINED;
    Tuareg.Decoder.phase= PHASE_UNDEFINED;

    Tuareg.Decoder.flags.phase_valid= false;
    Tuareg.Decoder.flags.position_valid= false;

    Tuareg.errors.sensor_CIS_error= true;
}


void reset_internal_data()
{
    reset_position_data();
    reset_timing_output();

    Decoder.last_crank_rpm= 0;
    Decoder.last_crank_acceleration= 0;
}


void decoder_set_state(decoder_internal_state_t NewState)
{
    VitalAssert(NewState < DSTATE_COUNT, TID_DECODER_LOGIC, DECODER_LOC_SETSTATE_ERROR);

    //collect debug information
    #ifdef DECODER_EVENT_DEBUG
    if((Decoder.state == DSTATE_SYNC) && (NewState != DSTATE_SYNC))
    {
        register_sync_lost_debug_event();
    }
    #endif // DECODER_EVENT_DEBUG

    Decoder.state= NewState;
}


/******************************************************************************************************************************
decoder helper functions - sync checker
******************************************************************************************************************************/

/*
evaluate key/gap ratio to keep trigger wheel sync state
*/
bool check_sync_ratio()
{
    VU32 sync_ratio, sync_interval, key_interval;

    if((Decoder_hw.current_timer_value == 0) || (Decoder_hw.prev1_timer_value == 0) || (Decoder_hw.prev2_timer_value == 0) || (Decoder_hw.captured_positions_cont < 7) || (Decoder_hw.state.timer_continuous_mode == false))
    {
        decoder_diag_log_event(DDIAG_SYNCHK_SYN_FAIL);
        return false;
    }

    sync_interval= subtract_U32(Decoder_hw.current_timer_value, Decoder_hw.prev2_timer_value);
    key_interval= subtract_U32(Decoder_hw.prev1_timer_value, Decoder_hw.prev2_timer_value);

    //calculate key/gap ratio in percent
    sync_ratio= divide_U32(100 * key_interval, sync_interval);

    //check the key/gap ratio against the sync interval
    if( (sync_ratio < Decoder_Setup.sync_ratio_min_pct) || (sync_ratio > Decoder_Setup.sync_ratio_max_pct) )
    {
        decoder_diag_log_event(DDIAG_SYNCHK_SYN_FAIL);
        return false;
    }

    //check succeeded
    decoder_diag_log_event(DDIAG_SYNCHK_SYN_PASS);
    return true;
}




//evaluate key/gap ratio to get trigger wheel sync in timer discontinuous mode to get SYNC
bool check_sync_ratio_async()
{
    VU32 sync_ratio;


    if((Decoder_hw.current_timer_value == 0) || (Decoder_hw.prev1_timer_value == 0) || (Decoder_hw.captured_positions_cont != 1) || (Decoder_hw.state.timer_continuous_mode == true))
    {
        decoder_diag_log_event(DDIAG_SYNCHK_ASYN_FAIL);
        return false;
    }

    //calculate key/gap ratio in percent
    sync_ratio= (100 * Decoder_hw.prev1_timer_value) / (Decoder_hw.prev1_timer_value + Decoder_hw.current_timer_value);

    //check the key/gap ratio against the sync interval
    if( (sync_ratio < Decoder_Setup.sync_ratio_min_pct) || (sync_ratio > Decoder_Setup.sync_ratio_max_pct) )
    {
        decoder_diag_log_event(DDIAG_SYNCHK_ASYN_FAIL);
        return false;
    }

    //check succeeded
    decoder_diag_log_event(DDIAG_SYNCHK_ASYN_PASS);
    return true;
}



/******************************************************************************************************************************
decoder helper functions - timing data calculation
******************************************************************************************************************************/

const U32 cDecoder_min_valid_period= 6000;
const U32 cDecoder_min_valid_rpm= 100;
const U32 cDecoder_max_valid_rpm= 10000;

void update_timing_data()
{
    U32 period_us, rpm;
    F32 accel;

    //set invalid output data, but keep last_crank_rpm
    reset_timing_output();

    #ifdef DECODER_TIMING_DEBUG
    decoder_timing_debug_next_cycle();
    #endif // DECODER_TIMING_DEBUG

    /**
    precondition check

    the first timer period captured after timer start will be invalid -> discard it
    (recognised when prev1 and prev2 are zero)
    */
    if( (Decoder_hw.state.timer_continuous_mode == false) || (Decoder_hw.captured_positions_cont != 1) || (Decoder_hw.prev1_timer_value == 0) || (Decoder_hw.prev2_timer_value == 0))
    {
        //save debug information
        #ifdef DECODER_TIMING_DEBUG
        decoder_update_timing_debug();
        #endif // DECODER_TIMING_DEBUG

        //exit with invalid outputs
        return;
    }

    //the timer value after an reset in continuous mode reflects T360
    period_us= Decoder_hw.current_timer_value * Decoder_hw.timer_period_us;

    //check if the timer value can be a valid crank period
    if(period_us < cDecoder_min_valid_period)
    {
        //in the next cycle there will be no valid last_crank_rpm
        Decoder.last_crank_rpm= 0;
        Decoder.last_crank_acceleration= 0;

        //save debug information
        #ifdef DECODER_TIMING_DEBUG
        decoder_update_timing_debug();
        #endif // DECODER_TIMING_DEBUG

        return;
    }

    //calculate crank rpm based on the validated period
    rpm= calc_rpm(period_us);

    if((rpm < cDecoder_min_valid_rpm) || (rpm > cDecoder_max_valid_rpm))
    {
        //why period was valid?
        Tuareg.Decoder.crank_period_us= 0;
        Tuareg.Decoder.flags.period_valid= false;
        Decoder.last_crank_rpm= 0;
        Decoder.last_crank_acceleration= 0;

        //save debug information
        #ifdef DECODER_TIMING_DEBUG
        decoder_update_timing_debug();
        #endif // DECODER_TIMING_DEBUG

        //log
        Syslog_Error(TID_DECODER_LOGIC, DECODER_LOC_UPDTIM_RPM_INVALID);

        return;
    }

    //export validated data
    Tuareg.Decoder.crank_period_us= period_us;
    Tuareg.Decoder.flags.period_valid= true;
    Tuareg.Decoder.crank_rpm= rpm;
    Tuareg.Decoder.flags.rpm_valid= true;
    Tuareg.Decoder.flags.standstill= false;


    /**
    calculate the difference in rpm based on the former valid rpm figure
    delta_rpm > 0 --> accel
    delta_rpm < 0 --> decel

    a := dn/dt -> a := 1000000 * delta_n / T360
    */
    if((Decoder.last_crank_rpm > cDecoder_min_valid_rpm) && (Decoder.last_crank_rpm < cDecoder_max_valid_rpm))
    {
        //period_us has already been validated in precondition check
        divide_float(1000000.0 * ((F32) rpm - (F32) Decoder.last_crank_rpm), (F32) period_us, &accel);

        //apply the ema filter
        Tuareg.Decoder.crank_acceleration= update_ema_filter(Decoder_Setup.accel_filter_coeff, &(Decoder.last_crank_acceleration), accel);

        //mark output data valid
        Tuareg.Decoder.flags.accel_valid= true;
    }

    //store last current engine speed for the calculation from next cycle
    Decoder.last_crank_rpm= rpm;

    //save debug information
    #ifdef DECODER_TIMING_DEBUG
    decoder_update_timing_debug();
    #endif // DECODER_TIMING_DEBUG
}


/******************************************************************************************************************************
decoder logic initialization
******************************************************************************************************************************/
void init_decoder_logic()
{
    //start with clean data
    reset_internal_data();

    decoder_set_state(DSTATE_INIT);

    /**
    initialize with the assumption that the engine is at standstill
    the first crank sensor event will clear the standstill flag
    Without a crank sensor event the timeout logic would not trigger.
    */
    Tuareg.Decoder.flags.standstill= true;

    //enable crank irq
    decoder_unmask_crank_irq();
}


void disable_decoder_logic()
{
    //wipe internal and output data
    reset_internal_data();

    //assume that the engine will come to standstill quickly
    Tuareg.Decoder.flags.standstill= true;
}

/******************************************************************************************************************************
crankshaft position sensor - irq handler
******************************************************************************************************************************/
void decoder_crank_handler()
{
    //we just saw a trigger condition
    reset_timeout_counter();

    switch(Decoder.state) {

        case DSTATE_INIT:

            decoder_set_state(DSTATE_ASYNC);

            //prepare to detect a KEY begin
            decoder_set_crank_pickup_sensing(Decoder_Setup.key_begin_sensing);

            /**
            this is the first impulse captured in this cycle
            it is not certain which crank sensing applied until now and it is not certain if the decoder timer has been running
            the timer will be in discontinuous mode
            */
            decoder_start_timer();

            //clean up
            reset_internal_data();

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_INIT);

            break;


        case DSTATE_ASYNC:

            /**
            1. synchronization step, we are at the beginning of a key
            cur := x, prev1 := x
            */
            decoder_set_crank_pickup_sensing(Decoder_Setup.key_end_sensing);
            decoder_set_state(DSTATE_ASYNC_KEY);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_ASYNC);

            break;


        case DSTATE_ASYNC_KEY:

            /**
            2. synchronization step, we are at the end of a key
            cur := key duration, prev1 := x
            */

            decoder_set_crank_pickup_sensing(Decoder_Setup.key_begin_sensing);
            decoder_set_state(DSTATE_ASYNC_GAP);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_ASYNC_KEY);

            break;


        case DSTATE_ASYNC_GAP:

            /**
            3. synchronization step, we are at the end of a gap (beginning of a key)
            cur := gap duration, prev := key duration (from DSTATE_ASYNC_KEY)
            */
            decoder_set_crank_pickup_sensing(Decoder_Setup.key_end_sensing);

            if( check_sync_ratio_async() == true )
            {
                /// got SYNC
                decoder_set_state(DSTATE_SYNC);
                Tuareg.Decoder.crank_position= CRK_POSITION_B1;

                //switch decoder timer mode to continuous
                decoder_request_timer_reset();
                decoder_set_timer_continuous_mode_on();

                //collect debug information
                #ifdef DECODER_EVENT_DEBUG
                register_got_sync_debug_event();
                #endif // DECODER_EVENT_DEBUG

            }
            else
            {
                //then check the next key
                decoder_set_state(DSTATE_ASYNC_KEY);

                //clean up
                decoder_set_timer_continuous_mode_off();
                reset_internal_data();
            }

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_ASYNC_GAP);

            break;


        case DSTATE_SYNC:

            // update crank_position
            Tuareg.Decoder.crank_position= crank_position_after(Tuareg.Decoder.crank_position);

            // update crank sensing
            decoder_set_crank_pickup_sensing(SENSING_INVERT);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_SYNC);

            /**
            per-position decoder housekeeping actions:
            crank_position is the crank position that has been reached at the beginning of this interrupt
            */
            switch(Tuareg.Decoder.crank_position)
            {
                case CRK_POSITION_B1:

                    //do sync check
                    if( check_sync_ratio() == true )
                    {
                        Tuareg.Decoder.flags.position_valid= true;

                        //reset the decoder timer when position B2 will be reached
                        decoder_request_timer_reset();
                    }
                    else
                    {
                        //sync check failed!
                        reset_internal_data();

                        //prepare for the next trigger condition
                        decoder_set_state(DSTATE_INIT);

                        //notify high speed logger about error condition
                        highspeedlog_register_error();

                        //log
                        Syslog_Warning(TID_DECODER_LOGIC, DECODER_LOC_LOST_SYNC);
                    }

                    break;


                case CRK_POSITION_B2:

                    //update engine rotational speed calculation
                    update_timing_data();
                    break;


                case CRK_POSITION_C1:

                    //update engine phase right at the next crank position after TDC
                    Tuareg.Decoder.phase= opposite_phase(Tuareg.Decoder.phase);
                    break;

                default:
                    break;

            } //switch(Tuareg.Decoder.crank_position)


            /**
            CIS control
            */
            if(Tuareg.Decoder.crank_position == Decoder_Setup.cis_enable_pos)
            {
                //activate the cis to collect cam information
                enable_cis();
            }
            else if(Tuareg.Decoder.crank_position == Decoder_Setup.cis_disable_pos)
            {
                //evaluate the collected cam information
                disable_cis();
            }

            //notify high speed logger about new crank position
            highspeedlog_register_crankpos(Tuareg.Decoder.crank_position);

            /**
            finally trigger the decoder update irq -> last action here
            */
            if( (Tuareg.Decoder.flags.position_valid == true) && (Tuareg.Decoder.flags.period_valid == true) && (Tuareg.Decoder.flags.rpm_valid == true) )
            {
                trigger_decoder_irq();
            }
            break; //SYNC


        case DSTATE_TIMEOUT:

            //recover from timeout
            decoder_set_state(DSTATE_INIT);
            break;


        default:

            Fatal(TID_DECODER_LOGIC, DECODER_LOC_CRKHDL_STATE_ERROR);
            break;

        } //switch Decoder.sync_mode


        /**
        noise filter
        crank pickup irq masked after set_crank_pickup_sensing() until crank_noisefilter_handler() enables it
        */
}


/******************************************************************************************************************************
Timer for decoder control: compare event --> enable external interrupt for pickup sensor
******************************************************************************************************************************/
void decoder_crank_noisefilter_handler()
{
    decoder_unmask_crank_irq();
}


/******************************************************************************************************************************
Timer for decoder control: update event --> overflow interrupt occurs when no signal from crankshaft pickup
has been received for more than the configured interval
******************************************************************************************************************************/

const U32 cDecoderTimeout= 30;

void decoder_crank_timeout_handler()
{
    //reset decoder
    decoder_set_state(DSTATE_TIMEOUT);
    reset_internal_data();

    //report to interface
    Tuareg.Decoder.flags.standstill= true;

    //collect debug information
    #ifdef DECODER_EVENT_DEBUG
    register_timer_overflow_debug_event();
    #endif // DECODER_EVENT_DEBUG


    /**
    timer update indicates unstable engine operation
    */
    if(Decoder.timeout_count >= cDecoderTimeout)
    {
        //stopping the decoder timer will leave the crank noise filter enabled
        decoder_stop_timer();

        //so prepare for the next trigger condition
        decoder_unmask_crank_irq();

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_TIMEOUT_EVENTS);

        //collect debug information
        #ifdef DECODER_EVENT_DEBUG
        register_timeout_debug_event();
        #endif // DECODER_EVENT_DEBUG
    }
    else
    {
        Decoder.timeout_count++;
    }

}



