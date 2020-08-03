#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "uart.h"
#include "Tuareg.h"
#include "trigger_wheel_layout.h"
#include "config.h"

#include "rotation_calc.h"
#include "conversion.h"

volatile decoder_internals_t DInternals;
volatile decoder_interface_t DInterface;


/**
how the decoder works:

-   on initialisation we enable crank pickup irq
-   with first crank pickup irq execution we start crank timer to measure time delay between
    crank pickup events
-   the timer keeps on counting till the next irq, providing a stable time base for processing delay measurement
-   because we know sensing (rising or falling edge), we can tell if a gap or a key on trigger wheel
    has been detected
-   syncronisation we get when a certain time ratio (key/gap ratio) is detected - thats the cycle beginning
-   from cycle beginning every rise/fall event takes us further in engine cycle - decoder keeps track of engine
    position and expects synchronization events after position D2
-   after each sensing event the timer gets a reset
-   a noise filter has been implemented: crank pickup signal edge detection irq gets enabled after some delay time after each
    irq execution to suppress signal noise
-   if a timer overflow occures, the engine most certainly has stalled / is not running

*/


/**
calculates detailed decoder statistics
*/
void decoder_statistics_handler(VU32 Interval)
{
   DInternals.segment_duration_deg[DInternals.crank_position]= calc_rot_angle_deg(Interval * DECODER_TIMER_PERIOD_US, DInterface.crank_T_us);

   DInternals.segment_duration_base_rpm= DInterface.engine_rpm;
}

/**
calculates detailed decoder statistics
*/
void reset_decoder_statistics()
{
    U32 item;

    for(item= 0; item < CRK_POSITION_COUNT; item++)
    {
        DInternals.segment_duration_deg[item]=0;
    }

    DInternals.segment_duration_base_rpm =0;

}



/**
writes the collected diagnostic data to the memory at pTarget
*/
void decoder_export_statistics(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < CRK_POSITION_COUNT; count++)
    {
        pTarget[count]= DInternals.segment_duration_deg[count];
    }

    pTarget[CRK_POSITION_COUNT]= DInternals.segment_duration_base_rpm;
}


/**
writes the collected diagnostic data to the memory a pTarget
*/
void decoder_export_diag(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < DDIAG_COUNT; count++)
    {
        pTarget[count]= DInternals.diag[count];
    }
}


inline void sync_lost_debug_handler()
{
    UART_Tx(DEBUG_PORT, '!');

}


inline void got_sync_debug_handler()
{
    UART_Tx(DEBUG_PORT, '+');

}


inline void decoder_timeout_debug_handler()
{

    UART_Send(DEBUG_PORT, "\r \n decoder timeout");
}



/**
evaluate key/gap ratio to get trigger wheel sync
*/
VU32 check_sync_ratio()
{
    VU32 sync_ratio, target_ratio;

    //collect diagnostic data
    DInternals.diag[DDIAG_SYNCCHECK_CALLS] += 1;

    if(DInternals.sync_buffer_key + DInternals.sync_buffer_gap == 0)
    {
        return RETURN_FAIL;
    }

    //calculate key/gap ratio in percent
    sync_ratio= (DInternals.sync_buffer_key * 100) / (DInternals.sync_buffer_key + DInternals.sync_buffer_gap);


    if( (sync_ratio >= configPage12.sync_ratio_min_pct) && (sync_ratio <= configPage12.sync_ratio_max_pct) )
    {
        /**
        strict check succeeded - its key A!
        */
        return RETURN_OK;
    }
    else
    {
        /**
        strict check failed
        apply relaxed sync check while cranking / re syncing
        */
        if(DInternals.sync_stability < configPage12.sync_stability_thrs)
        {
            //collect diagnostic data
            DInternals.diag[DDIAG_SYNCCHECK_RELAXED] += 1;

            //calculate target (mean) key/gap ratio
            target_ratio= configPage12.sync_ratio_min_pct + (configPage12.sync_ratio_max_pct - configPage12.sync_ratio_min_pct) / 2;

            // do relaxed check
            if((sync_ratio >= target_ratio / 2) && (sync_ratio <= 2* target_ratio))
            {
                /**
                relaxed check succeeded - its key A!
                */
                return RETURN_OK;
            }
        }
    }

    return RETURN_FAIL;
}



/**
calculate the angle at which the crankshaft will be, when the corresponding engine position will be detected be the decoder at the current

-> the angle the crank shaft is actually at significantly differs from the trigger wheel key position angle
-> the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

-> the crank angle wraps around at 360°
*/
void calculate_crank_position_table(VU32 Period, VU16 * Table)
{
    U32 position, angle;

    //collect diagnostic data
    DInternals.diag[DDIAG_CRANKTABLE_CALLS] += 1;

    for(position=0; position < CRK_POSITION_COUNT; position++)
    {
        //calculate angle
        angle= configPage12.trigger_position_map[position] + configPage12.decoder_offset_deg;

        //account for VR delay
        if(Period != 0)
        {
            angle += calc_rot_angle_deg(configPage12.decoder_delay_us, Period);
        }

        //wrap around crank angle
        if(angle >= 360)
        {
            angle -= 360;
        }

        //save to table
        Table[position]= angle;
    }
}

/**
crank rotational speed calculation
-> captures the key and gap duration from 2 full crank revolutions (1 engine cycle)
*/
void update_engine_speed(VU32 Interval)
{
    U32 period;

    //collect diagnostic data
    DInternals.diag[DDIAG_ROTSPEED_CALLS] += 1;

    /*
    collect data for engine rpm calculation
    (in every crank 360° revolution we collect 8 timer values -> 16 captures in 720°)
    range check: 2^4*2^16=2^20 -> fits to 32 bit variable
    */
    DInternals.cycle_timing_buffer += Interval;
    DInternals.cycle_timing_counter++;


    if(DInternals.cycle_timing_counter == 16)
    {
        /**
        crank_T_us acts as an accurate time base for following calculations

        the timer is adjusted to DECODER_TIMER_PERIOD_US interval
        cycle_timing_buffer holds the duration for 2 full crank turns in timer ticks (#T.720)
        */
        period= DInternals.cycle_timing_buffer * (DECODER_TIMER_PERIOD_US / 2);

        /**
        export engine_rpm
        rpm figure is mainly for user display and table lookup
        */
        DInterface.engine_rpm= calc_rpm(period);

        /**
        crank acceleration
        reflects the absolute change in rotational period since last cycle
        */
        if( (period != 0) && (DInterface.crank_T_us != 0) )
        {
            DInterface.crank_deltaT_us= period - DInterface.crank_T_us;
        }

        //export speed data to interface
        DInterface.crank_T_us= period;

        //reset calculation
        DInternals.cycle_timing_buffer= 0;
        DInternals.cycle_timing_counter= 0;

        //update engine position data
        calculate_crank_position_table(period, DInterface.crank_position_table_deg);
    }

}

inline void reset_position_data()
{
    DInternals.crank_position= CRK_POSITION_UNDEFINED;
    DInterface.crank_position= CRK_POSITION_UNDEFINED;
    DInternals.phase= PHASE_UNDEFINED;
    DInterface.phase= PHASE_UNDEFINED;
}

inline void reset_crank_timing_data()
{
    VU32 item;

    DInternals.cycle_timing_buffer= 0UL;
    DInternals.cycle_timing_counter= 0UL;

    DInterface.crank_T_us= 0UL;
    DInterface.crank_deltaT_us= 0UL;

    DInterface.engine_rpm= 0UL;

    //decoder statistics are based on engine rpm/period
    reset_decoder_statistics();
}

inline void reset_diag_data()
{
    VU32 item;

    for(item= 0; item < DDIAG_COUNT; item++)
    {
        DInternals.diag[item]=0;
    }
}

inline void reset_sync_stability()
{
    DInternals.sync_stability =0;
}


/**
    using hw layer
    enables pickup sensor interrupt
*/
volatile decoder_interface_t * init_decoder_logic()
{
    //interface variables
    DInternals.sync_mode= INIT;

    reset_position_data();
    reset_crank_timing_data();
    reset_diag_data();
    reset_sync_stability();

    //calculate the decoder timeout threshold corresponding to the configured timeout value
    DInternals.decoder_timeout_thrs= ((1000 * configPage12.decoder_timeout_s) / DECODER_TIMER_OVERFLOW_MS);


    /**
    we are now ready to process decoder events
    */

    //react to crank irq
    decoder_unmask_crank_irq();

    return &DInterface;
}







/******************************************************************************************************************************
crankshaft position sensor

as the decoder timer was reset to 0x00 when the interrupt handler has been called,
its value reveals the over all delay of the following operations


performance analysis revealed:
handler entry happens about 1 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
inline void decoder_logic_crank_handler(VU32 Interval)
{
    //reset timeout counter, we just saw a trigger condition
    DInternals.timeout_count= 0UL;

    //collect diagnostic data
    DInternals.diag[DDIAG_CRANKHANDLER_CALLS] += 1;

    switch(DInternals.sync_mode) {

        case INIT:

            // this is the first impulse ever captured -> no timing info available
            decoder_start_timer();
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);

            DInternals.sync_mode= ASYNC_KEY;

            reset_position_data();
            reset_crank_timing_data();

            //collect diagnostic data
            DInternals.diag[DDIAG_CRANKPOS_INIT] += 1;

            break;


        case ASYNC_KEY:

            /* we are at the end of a key -> key duration captured
            in ASYNC mode we try to find segment A */
            reset_crank_timing_data();
            reset_position_data();

            DInternals.sync_buffer_key= Interval;
            decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            DInternals.sync_mode= ASYNC_GAP;

            //collect diagnostic data
            DInternals.diag[DDIAG_CRANKPOS_ASYNC_KEY] += 1;

            break;


        case ASYNC_GAP:

            // we are at the beginning of a key -> timer captured gap duration, next int will be on key end anyways
            DInternals.sync_buffer_gap= Interval;
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);

            reset_crank_timing_data();
            reset_position_data();

            if( check_sync_ratio() == RETURN_OK )
            {
                // it was key A -> we have SYNC now!
                DInternals.sync_mode= SYNC;
                DInternals.crank_position= CRK_POSITION_B1;
                DInterface.crank_position= DInternals.crank_position;

                //collect diagnostic data
                DInternals.diag[DDIAG_ASYNC_SYNC_TR] += 1;

                //run desired debug action
                got_sync_debug_handler();

            }
            else
            {
                //any other key
                DInternals.sync_mode= ASYNC_KEY;
            }

            //collect diagnostic data
            DInternals.diag[DDIAG_CRANKPOS_ASYNC_GAP] += 1;

            break;


        case SYNC:

            //update crank_position
            DInternals.crank_position++;

            //wrap around
            if(DInternals.crank_position == CRK_POSITION_COUNT)
            {
                DInternals.crank_position= 0;
            }

            //export to interface
            DInterface.crank_position= DInternals.crank_position;

            // update crank sensing
            decoder_set_crank_pickup_sensing(INVERT);

            /*
            update rotational speed if necessary
            and update the crank position table
            */
            update_engine_speed(Interval);

            //collect diagnostic data
            DInternals.diag[DDIAG_CRANKPOS_SYNC] += 1;

            //count the synced position for stability examination
            if(DInternals.sync_stability < configPage12.sync_stability_thrs)
            {
                DInternals.sync_stability++;
            }


            /**
            per-position decoder housekeeping actions:
            crank_position is the crank position that was reached at the beginning of this interrupt
            */
            switch(DInternals.crank_position) {

            case CRK_POSITION_A2:

                // store key length for sync check
                DInternals.sync_buffer_key= Interval;
                break;


            case CRK_POSITION_B1:

                //do sync check
                DInternals.sync_buffer_gap= Interval;

                if( check_sync_ratio() == RETURN_OK )
                {
                    //sync check failed! -> prepare for the next trigger condition
                    DInternals.crank_position= CRK_POSITION_UNDEFINED;
                    DInternals.sync_mode= ASYNC_KEY;
                    decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);

                    //engine phase measurement is upset too
                    reset_position_data();
                    reset_crank_timing_data();
                    reset_sync_stability();

                    decoder_mask_cis_irq();

                    //collect diagnostic data
                    DInternals.diag[DDIAG_SYNC_ASYNC_TR] += 1;

                    //run desired debug actions
                    sync_lost_debug_handler();
                }
                break;

            default:
                //any other position
                break;

            } //switch DInternals.crank_position


            /**
            cylinder identification sensor handling
            running only in sync with crank, if we lose sync in POSITION_B1 sync check:
            -> the irq enable/disable branches will not be executed as crank_position defaults to UNDEFINED
            -> the c.i.s. irq is masked already
            */
            if(DInternals.crank_position == CYLINDER_SENSOR_ENA_POSITION)
            {
               // #warning TODO (oli#1#): DEBUG
                //enabling c.i.s irq will clear its pending flag
                decoder_unmask_cis_irq();
            }
            else if(DInternals.crank_position == CYLINDER_SENSOR_DISA_POSITION)
            {
                //disable c.i.s irq
                decoder_mask_cis_irq();

                //no c.i.s trigger event has been detected -> cylinder #2 working or broken sensor
                if(DInternals.phase == CYL1_WORK)
                {
                    //sensor gives valid data
                    DInternals.phase= CYL2_WORK;
                }
                else if(DInternals.phase == CYL2_WORK)
                {
                    //no alternating signal -> sensor error
                    DInternals.phase= PHASE_UNDEFINED;

                    //collect diagnostic data
                    DInternals.diag[DDIAG_PHASED_UNDEFINED_TR] += 1;
                }
            }

            //collect diagnostic data
            if(DInternals.phase == PHASE_UNDEFINED)
            {
                DInternals.diag[DDIAG_CRANKPOS_CIS_UNDEFINED] += 1;
            }
            else
            {
                DInternals.diag[DDIAG_CRANKPOS_CIS_PHASED] += 1;
            }


            //collect statistics data
            decoder_statistics_handler(Interval);

            break;

        } //switch DInternals.sync_mode


        /**
        finally trigger the sw irq 2 for decoder output processing
        (ca. 3.2us after trigger event)
        the irq will be triggered anyways to react on sync failures!

        shall be the last action in irq!
        */
        if(DInternals.sync_mode == SYNC)
        {
            //collect diagnostic data
            DInternals.diag[DDIAG_TRIGGER_IRQ_SYNC] += 1;
            DInternals.diag[DDIAG_TRIGGER_IRQ_DELAY]= decoder_get_data_age_us();

            trigger_decoder_irq();
        }



        /**
        noise filter
        mask crank pickup irq, timer 9 compare will enable it again
        crank pickup irq is disabled after each set_crank_pickup_sensing() call
        */

}


/******************************************************************************************************************************
Timer for decoder control: compare event --> enable external interrupt for pickup sensor
 ******************************************************************************************************************************/
inline void decoder_logic_timer_compare_handler()
{
    decoder_unmask_crank_irq();
}


/******************************************************************************************************************************
Timer for decoder control: update event --> overflow interrupt occurs when no signal from crankshaft pickup
has been received for more than the configured interval
 ******************************************************************************************************************************/
inline void decoder_logic_timer_update_handler()
{
    /**
    timer update indicates unstable engine operation
    */
    reset_sync_stability();

    if(DInternals.timeout_count >= DInternals.decoder_timeout_thrs)
    {
        //reset decoder
        DInternals.sync_mode= INIT;

        reset_position_data();
        reset_crank_timing_data();

        //prepare for the next trigger condition
        decoder_stop_timer();
        decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);

        //shut down c.i.s. irq
        decoder_mask_cis_irq();
        DInternals.phase= PHASE_UNDEFINED;

        //run desired debug action
        decoder_timeout_debug_handler();

        //prepare for the next trigger condition
        decoder_unmask_crank_irq();

        //reset timeout_count and give sync a chance
        DInternals.timeout_count =0;

        //collect diagnostic data
        DInternals.diag[DDIAG_TIMEOUT_EVENTS] += 1;

        //trigger sw irq for decoder output processing - LAST ACTION here!
        trigger_decoder_irq();
    }
    else
    {
        DInternals.timeout_count++;

        //collect diagnostic data
        DInternals.diag[DDIAG_TIMER_UPDATE_EVENTS] += 1;
    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/
inline void decoder_logic_cam_handler()
{

    //collect diagnostic data
    DInternals.diag[DDIAG_CISHANDLER_CALLS] += 1;

    /**
    a trigger condition has been seen on c.i.s -> cylinder #1 working or sensor error
    first trigger condition allows us to leave UNDEFINED state
    */
    if((DInternals.phase == CYL2_WORK) || (DInternals.phase == PHASE_UNDEFINED))
    {
        //alternating signal -> valid
        DInternals.phase= CYL1_WORK;
    }
    else if(DInternals.phase == CYL1_WORK)
    {
        //invalid signal
        DInternals.phase= PHASE_UNDEFINED;

        //collect diagnostic data
        DInternals.diag[DDIAG_PHASED_UNDEFINED_TR] += 1;
    }

    //disable c.i.s irq until next turn
    decoder_mask_cis_irq();
}



