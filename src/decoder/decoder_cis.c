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


/******************************************************************************************************************************
cylinder identification sensor handling
******************************************************************************************************************************/


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
    U32 lobe_angle_deg, lobe_interval_us;

    //collect diagnostic information
    decoder_diag_log_event(DDIAG_CISUPD_CALLS);

    //precondition check
    if((Decoder.out.flags.period_valid == false) ||
       ((Decoder.cis.lobe_begin_detected == true) && (Decoder.cis.lobe_end_detected == false)) ||
       ((Decoder.cis.lobe_begin_detected == false) && (Decoder.cis.lobe_end_detected == true)) ||
       (Decoder.cis.cis_failure == true) )
    {
        //phase information invalid
        Decoder.out.phase= PHASE_UNDEFINED;
        Decoder.out.flags.phase_valid= false;

        //phase sync lost
        Decoder.cis_sync_counter =0;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PRECOND_FAIL);

        return;
    }

    /*******************************************
    signal validation:
    the cis has been triggered if the cam lobe has been detected for more then cis_min_angle_deg
    *******************************************/

    //default estimation: sensor has not been triggered
    detected_phase= opposite_phase(Decoder_Setup.cis_triggered_phase);

    //check if the cis has detected a rising and a falling signal edge
    if( (Decoder.cis.lobe_begin_detected == true) && (Decoder.cis.lobe_end_detected == true))
    {
        lobe_interval_us= Decoder_hw.timer_period_us * subtract_VU32(Decoder.lobe_end_timestamp, Decoder.lobe_begin_timestamp);

        lobe_angle_deg= calc_rot_angle_deg(lobe_interval_us, Decoder.out.crank_period_us);

        //check if lobe interval is longer than the defined minimum
        if(lobe_angle_deg > Decoder_Setup.cis_min_angle_deg)
        {
            //signal edges have been detected AND the interval is valid
            detected_phase= Decoder_Setup.cis_triggered_phase;

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CISUPD_TRIGGERED);
        }
    }

    /*******************************************
    evaluation:
    the detected cis signal shall confirm the currently expected phase
    *******************************************/

    //check if the resulting cis signal confirms the currently expected phase
    if(Decoder.out.phase == detected_phase)
    {
        //GOOD!

        if(Decoder.cis_sync_counter < Decoder_Setup.cis_sync_thres)
        {
            Decoder.cis_sync_counter += 1;
        }

        if(Decoder.cis_sync_counter == Decoder_Setup.cis_sync_thres)
        {
            //stable cis signal detected
            Decoder.out.flags.phase_valid= true;

            //report to flags
            Tuareg.errors.sensor_CIS_error= false;
        }

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_PASS);
    }
    else
    {
        //expected engine phase information requires updating
        Decoder.out.flags.phase_valid= false;

        //report to flags
        Tuareg.errors.sensor_CIS_error= true;

/// TODO (oli#3#): add syslog entry for cis


        //update the expected phase information
        Decoder.out.phase= detected_phase;
        Decoder.cis_sync_counter =0;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_CISUPD_PHASE_FAIL);
    }
}


/******************************************************************************************************************************
cylinder identification sensor handling - irq handler
called from decoder_hw module when the corresponding EXTI has been triggered
******************************************************************************************************************************/
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

    //save decoder timestamp with respect to cam sensing
    if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_begin_sensing)
    {
        Decoder.lobe_begin_timestamp= decoder_get_timestamp();
        Decoder.cis.lobe_begin_detected= true;

        //invert cam sensing
        decoder_set_cis_sensing(Decoder_Setup.lobe_end_sensing);

        //notify high speed log
        highspeedlog_register_cis_lobe_begin();

        //collect diagnostic information
        decoder_diag_log_event(DDIAG_LOBE_BEG);

    }
    else if(Decoder_hw.cis_sensing == Decoder_Setup.lobe_end_sensing)
    {
        Decoder.lobe_end_timestamp= decoder_get_timestamp();
        Decoder.cis.lobe_end_detected= true;

        //invert cam sensing
        decoder_set_cis_sensing(Decoder_Setup.lobe_begin_sensing);

        //notify high speed log
        highspeedlog_register_cis_lobe_end();

        //collect diagnostic information
        decoder_diag_log_event(DDIAG_LOBE_END);
    }
    else
    {
        //collect diagnostic information
        decoder_diag_log_event(DDIAG_INVALID_TRIG);
    }


    update_cam_noisefilter();
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

    //trust the sensor by now
    Decoder.cis.cis_failure= false;
    Decoder.cis.lobe_begin_detected= false;
    Decoder.cis.lobe_end_detected= false;

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
