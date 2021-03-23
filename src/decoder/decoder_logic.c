#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"

#include "base_calc.h"

#include "uart.h"
#include "uart_printf.h"
#include "debug_port_messages.h"

#include "diagnostics.h"
#include "highspeed_loggers.h"


volatile Tuareg_decoder_t Decoder;


/**
how the decoder works:
-   add description!!!
*/


/******************************************************************************************************************************
decoder helper functions - debug actions
******************************************************************************************************************************/

/// TODO (oli#4#): redirect decoder debug outputs to syslog

void decoder_process_debug_events()
{
    if(Decoder.debug.all_flags > 0)
    {
        DebugMsg_Warning("Decoder Debug Events (sync_lost got_sync timer_overflow timeout): ");
        UART_Tx(DEBUG_PORT, (Decoder.debug.lost_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.got_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.timer_overflow? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder.debug.timeout? '1' :'0'));

        //reset flags
        Decoder.debug.all_flags= 0;
    }
}

 void register_sync_lost_debug_event()
{
    //set flag
    Decoder.debug.lost_sync= true;
}


 void register_got_sync_debug_event()
{
    //set flag
    Decoder.debug.got_sync= true;
}


 void register_timer_overflow_debug_event()
{
    //set flag
    Decoder.debug.timer_overflow= true;
}


 void register_timeout_debug_event()
{
    //set flag
    Decoder.debug.timeout= true;
}



/******************************************************************************************************************************
decoder helper functions - cycle timing
******************************************************************************************************************************/

VU32 decoder_get_position_data_age_us()
{
    VU32 now_ts, update_ts, interval_us;

    now_ts= decoder_get_timestamp();
    update_ts= Decoder_hw.current_timer_value;

    //check counting mode
    if(Decoder_hw.state.timer_continuous_mode == true)
    {
        //timer continuously counting since last position update
        interval_us= Decoder_hw.timer_period_us * subtract_VU32(now_ts, update_ts);
    }
    else
    {
        //timer has been reset on last position update
        interval_us= Decoder_hw.timer_period_us * now_ts;
    }

    return interval_us;

}


/******************************************************************************************************************************
decoder helper functions - sync checker
******************************************************************************************************************************/

//evaluate key/gap ratio to keep trigger wheel sync state
 bool check_sync_ratio()
{
    VU32 sync_ratio, sync_interval, key_interval;

    if((Decoder_hw.current_timer_value == 0) || (Decoder_hw.prev1_timer_value == 0) || (Decoder_hw.prev2_timer_value == 0) || (Decoder_hw.captured_positions_cont < 7) || (Decoder_hw.state.timer_continuous_mode == false))
    {
        decoder_diag_log_event(DDIAG_SYNCHK_SYN_FAIL);
        return false;
    }

    sync_interval= subtract_VU32(Decoder_hw.current_timer_value, Decoder_hw.prev2_timer_value);
    key_interval= subtract_VU32(Decoder_hw.prev1_timer_value, Decoder_hw.prev2_timer_value);

    //calculate key/gap ratio in percent
    sync_ratio= divide_VU32(100 * key_interval, sync_interval);


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

 void update_timing_data()
{
    VU32 period_us, rpm, delta_rpm;
  //  VF32 accel =0;


    //set invalid output data initially
    reset_timing_data();


    //precondition check
    if( (Decoder_hw.state.timer_continuous_mode == false) || (Decoder_hw.captured_positions_cont != 1))
    {
        //exit with invalid outputs
        return;
    }

    //the timer value after an reset in continuous mode reflects T360
    period_us= Decoder_hw.current_timer_value * Decoder_hw.timer_period_us;

    if(period_us < 6000)
    {
        //exit with invalid outputs
        return;
    }

    //copy valid crank period figure
    Decoder.crank_period_us= period_us;
    Decoder.outputs.period_valid= true;

    //calculate crank rpm based on the valid period figure
    rpm= calc_rpm(period_us);

    if((rpm < 100) || (rpm > 10000))
    {
        return;
    }

    //calculate the difference in rpm based on the former valid rpm figure
    if(Decoder.outputs.rpm_valid)
    {
        delta_rpm= subtract_VU32(rpm, Decoder.prev1_crank_rpm);
    }
    else
    {
        delta_rpm= 0;
    }

    //copy valid rpm figure
    Decoder.crank_rpm= rpm;
    Decoder.outputs.rpm_valid= true;

    /**
    crank acceleration
    reflects the absolute change in rpm figure since last cycle:
    a := dw/dt -> a := 6 * (rpm - prev1_rpm) / T360
    */
    //accel= divide_VF32(delta_rpm / 10, period_us/ 1000000);


    /// TODO (oli#5#): add range check and set the output flag
    //Decoder.crank_acceleration= accel;
    Decoder.crank_acceleration= 0;

}


 void reset_timing_data()
{
    Decoder.crank_period_us= 0;
    Decoder.crank_rpm= 0;
    Decoder.crank_acceleration= 0;

    Decoder.prev1_crank_rpm= 0;

    Decoder.outputs.period_valid= false;
    Decoder.outputs.rpm_valid= false;
    Decoder.outputs.accel_valid= false;
}

/******************************************************************************************************************************
cylinder identification sensor
******************************************************************************************************************************/

 void reset_timeout_counter()
{
    Decoder.timeout_count =0;
    Decoder.outputs.timeout= false;
}


 void reset_position_data()
{
    Decoder.crank_position= CRK_POSITION_UNDEFINED;
    Decoder.phase= PHASE_UNDEFINED;

    Decoder.outputs.phase_valid= false;
    Decoder.outputs.position_valid= false;
}


 void reset_internal_data()
{
    reset_position_data();
    reset_timing_data();
}



 void decoder_set_state(decoder_internal_state_t NewState)
{
    if(NewState >= DSTATE_COUNT)
    {
        //error
        Decoder.state= DSTATE_INIT;
        return;
    }

    //collect debug information
    if((Decoder.state == DSTATE_SYNC) && (NewState != DSTATE_SYNC))
    {
        register_sync_lost_debug_event();
    }

    Decoder.state= NewState;
}





/******************************************************************************************************************************
decoder logic initialization

as the decoder timer was reset to 0x00 when the interrupt handler has been called,
its value reveals the over all delay of the following operations


performance analysis revealed:
handler entry happens about 1 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
volatile Tuareg_decoder_t * init_decoder_logic()
{
    decoder_set_state(DSTATE_INIT);

    reset_internal_data();
    reset_timeout_counter();

    //calculate the decoder timeout threshold corresponding to the configured timeout value
    Decoder.decoder_timeout_thrs= ((1000UL * Decoder_Setup.timeout_s) / Decoder_hw.timer_overflow_ms) +1;

    /**
    we are now ready to process decoder events
    */

    //react to crank irq
    decoder_unmask_crank_irq();

    return &Decoder;
}



/******************************************************************************************************************************
crankshaft position sensor

as the decoder timer was reset to 0x00 when the interrupt handler has been called,
its value reveals the over all delay of the following operations


performance analysis revealed:
handler entry happens about 1 us after the trigger signal edge had occurred
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
                Decoder.crank_position= CRK_POSITION_B1;

                //switch decoder timer mode to continuous
                decoder_request_timer_reset();
                decoder_set_timer_continuous_mode_on();

                //collect debug information
                register_got_sync_debug_event();

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
            Decoder.crank_position= crank_position_after(Decoder.crank_position);

            // update crank sensing
            decoder_set_crank_pickup_sensing(SENSING_INVERT);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRKPOS_SYNC);

            /**
            per-position decoder housekeeping actions:
            crank_position is the crank position that has been reached at the beginning of this interrupt
            */
            switch(Decoder.crank_position)
            {
                case CRK_POSITION_B1:

                    //do sync check
                    if( check_sync_ratio() == true )
                    {
                        Decoder.outputs.position_valid= true;

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
                    }

                    break;

                case CRK_POSITION_B2:

                    //update engine rotational speed calculation
                    update_timing_data();

                    break;


                case CRK_POSITION_C1:

                    //update engine phase
                    if(Decoder.outputs.phase_valid == true)
                    {
                        Decoder.phase= opposite_phase(Decoder.phase);
                    }

                    break;

                default:
                    break;

            } //switch(Decoder.crank_position)


            if(Decoder.crank_position == Decoder_Setup.cis_enable_pos)
            {
                //activate the cis to collect cam information
                enable_cis();
            }
            else if(Decoder.crank_position == Decoder_Setup.cis_disable_pos)
            {
                //evaluate the collected cam information
                disable_cis();
            }

            break; //SYNC


        case DSTATE_TIMEOUT:

            //recover from timeout
            decoder_set_state(DSTATE_INIT);
            break;


        default:
            //invalid state
            decoder_set_state(DSTATE_INIT);
            break;

        } //switch Decoder.sync_mode



        /**
        finally trigger the decoder update irq -> last action here
        */
        if(Decoder.state == DSTATE_SYNC)
        {
            //notify high speed logger about new crank position
            highspeedlog_register_crankpos(Decoder.crank_position);

            trigger_decoder_irq();
        }

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
void decoder_crank_timeout_handler()
{
    //reset decoder
    decoder_set_state(DSTATE_TIMEOUT);
    reset_internal_data();

    //collect debug information
    register_timer_overflow_debug_event();


    /**
    timer update indicates unstable engine operation
    */
    if(Decoder.timeout_count >= Decoder.decoder_timeout_thrs)
    {

        Decoder.outputs.timeout= true;

        //stopping the decoder timer will leave the crank noise filter enabled
        decoder_stop_timer();

        //so prepare for the next trigger condition
        decoder_unmask_crank_irq();

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_TIMEOUT_EVENTS);

        //collect debug information
        register_timeout_debug_event();


        /**
        trigger sw irq for decoder output processing
        LAST ACTION here!
        */
        trigger_decoder_irq();
    }
    else
    {
        Decoder.timeout_count++;
    }

}


/******************************************************************************************************************************
cylinder identification sensor
******************************************************************************************************************************/
 void decoder_update_cis()
{
    engine_phase_t detected_phase;
    U32 lobe_angle_deg, lobe_interval_us;

    //precondition check
    if((Decoder.outputs.period_valid == false) ||
       ((Decoder.cis.lobe_begin_detected == true) && (Decoder.cis.lobe_end_detected == false)) ||
       ((Decoder.cis.lobe_begin_detected == false) && (Decoder.cis.lobe_end_detected == true)) ||
       (Decoder.cis.cis_failure == true) )
    {
        //phase information invalid
        Decoder.phase= PHASE_UNDEFINED;
        Decoder.outputs.phase_valid= false;

        //phase sync lost
        Decoder.cis_sync_counter =0;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PRECOND_FAIL);

        return;
    }

    /*******************************************
    signal validation
    *******************************************/

    if( (Decoder.cis.lobe_begin_detected == true) && (Decoder.cis.lobe_end_detected == true))
    {
        lobe_interval_us= Decoder_hw.timer_period_us * subtract_VU32(Decoder.lobe_end_timestamp, Decoder.lobe_begin_timestamp);

        lobe_angle_deg= calc_rot_angle_deg(lobe_interval_us, Decoder.crank_period_us);

        //check lobe span against angular interval
        if(lobe_angle_deg > Decoder_Setup.cis_min_angle_deg)
        {
            detected_phase= Decoder_Setup.cis_triggered_phase;
        }
        else
        {
            detected_phase= opposite_phase(Decoder_Setup.cis_triggered_phase);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CISUPD_INTERVAL_FAIL);
        }
    }

    /*******************************************
    evaluation
    *******************************************/

    //check if the resulting cis signal confirms the currently expected phase
    if(Decoder.phase == detected_phase)
    {
        //GOOD!

        if(Decoder.cis_sync_counter < Decoder_Setup.cis_sync_thres)
        {
            Decoder.cis_sync_counter += 1;
        }

        if(Decoder.cis_sync_counter == Decoder_Setup.cis_sync_thres)
        {
            //stable cis signal detected
            Decoder.outputs.phase_valid= true;

            //report to errors
            Tuareg.Errors.sensor_CIS_error= false;
        }

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_PASS);
    }
    else
    {
        //expected engine phase information requires updating
        Decoder.outputs.phase_valid= false;

        //report to errors
        Tuareg.Errors.sensor_CIS_error= false;

        //save the currently detected phase
        Decoder.phase= detected_phase;
        Decoder.cis_sync_counter =0;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_FAIL);
    }
}


 void decoder_cis_handler()
{
    //precondition check
    if((Decoder_hw.state.timer_continuous_mode == false) || (Decoder.cis.cis_failure == true))
    {
        Decoder.cis.cis_failure= true;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISHDL_PRECOND_FAIL);

        return;
    }

    //improvement idea: check if multiple lobe begins, ends have been detected, apply noise filter

    //save decoder timestamp with respect to cam sensing
    if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_begin_sensing)
    {
        Decoder.lobe_begin_timestamp= decoder_get_timestamp();
        Decoder.cis.lobe_begin_detected= true;

        //invert cam sensing
        decoder_set_cis_sensing(Decoder_Setup.lobe_end_sensing);

        //notify high speed log
        highspeedlog_register_cis_lobe_begin();

    }
    else if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_end_sensing)
    {
        Decoder.lobe_end_timestamp= decoder_get_timestamp();
        Decoder.cis.lobe_end_detected= true;

        //invert cam sensing
        decoder_set_cis_sensing(Decoder_Setup.lobe_begin_sensing);

        //notify high speed log
        highspeedlog_register_cis_lobe_end();
    }


    update_cam_noisefilter();
}


 void decoder_cis_noisefilter_handler()
{
    decoder_unmask_cis_irq();
}


 void enable_cis()
{
    //set sensing for lobe beginning
    decoder_set_cis_sensing(Decoder_Setup.lobe_begin_sensing);

    //trust the sensor by now
    Decoder.cis.cis_failure= false;
    Decoder.cis.lobe_begin_detected= false;
    Decoder.cis.lobe_end_detected= false;

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
