#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "uart.h"
#include "Tuareg.h"
#include "trigger_wheel_layout.h"
#include "config.h"

#include "base_calc.h"
#include "conversion.h"
#include "diagnostics.h"

#include "debug.h"

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
/*
void decoder_statistics_handler(VU32 Interval)
{
   DInternals.segment_duration_deg[DInternals.crank_position]= calc_rot_angle_deg(Interval * DECODER_TIMER_PERIOD_US, DInterface.crank_T_us);

   DInternals.segment_duration_base_rpm= calc_rpm(DInterface.crank_T_us);
}
*/
/**
calculates detailed decoder statistics
*/
/*
void reset_decoder_statistics()
{
    U32 item;

    for(item= 0; item < CRK_POSITION_COUNT; item++)
    {
        DInternals.segment_duration_deg[item]=0;
    }

    DInternals.segment_duration_base_rpm =0;

}
*/


/**
writes the collected diagnostic data to the memory at pTarget
*/
/*
void decoder_export_statistics(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < CRK_POSITION_COUNT; count++)
    {
        pTarget[count]= DInternals.segment_duration_deg[count];
    }

    pTarget[CRK_POSITION_COUNT]= DInternals.segment_duration_base_rpm;
}
*/

/**
writes the collected diagnostic data to the memory a pTarget
*/
/*
void decoder_export_diag(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < DDIAG_COUNT; count++)
    {
        pTarget[count]= DInternals.diag[count];
    }
}
*/

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
inline VU32 check_sync_ratio()
{
    VU32 sync_ratio;

    //collect diagnostic data
    decoder_diag_log_event(DDIAG_SYNCCHECK_CALLS);

    if((DInternals.sync_buffer_key > 0) && (DInternals.sync_buffer_gap > 0))
    {
        //calculate key/gap ratio in percent
        sync_ratio= (DInternals.sync_buffer_key * 100) / (DInternals.sync_buffer_key + DInternals.sync_buffer_gap);


        if( (sync_ratio >= configPage12.sync_ratio_min_pct) && (sync_ratio <= configPage12.sync_ratio_max_pct) )
        {
            /**
            check succeeded - its key A!
            */
            decoder_diag_log_event(DDIAG_SYNCCHECK_SUCCESS);

            return RETURN_OK;
        }

    }

    decoder_diag_log_event(DDIAG_SYNCCHECK_FAILED);

    return RETURN_FAIL;
}



/**
calculate the angle at which the crankshaft will be, when the corresponding engine position will be detected be the decoder at the current

-> the angle the crank shaft is actually at significantly differs from the trigger wheel key position angle
-> the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

-> the crank angle wraps around at 360°
-> returns the crank base angles when crank speed is unknown
*/
void update_crank_position_table(volatile crank_position_table_t * Table)
{
    VU32 position, angle;

    //collect diagnostic data
    decoder_diag_log_event(DDIAG_CRANKTABLE_CALLS);

    for(position=0; position < CRK_POSITION_COUNT; position++)
    {
        //get trigger position base angle
        angle= configPage12.trigger_position_map.a_deg[position];

        //calculate effective crank angle
        angle += configPage12.decoder_offset_deg;

        //calculate VR introduced delay
        angle += calc_rot_angle_deg(configPage12.decoder_delay_us, DInternals.crank_period_us);

        //wrap around crank angle
        angle= angle % 360;

        //save to table
        Table->a_deg[position]= angle;
    }
}

/**
crank rotational speed calculation
-> captures the interval duration from 2 full crank revolutions (1 engine cycle)
*/
void update_engine_speed(VU32 Interval)
{
    VU32 period;

    //collect diagnostic data
    decoder_diag_log_event(DDIAG_ROTSPEED_CALLS);

    /**
    collect data for engine rpm calculation
    (in every crank 360° revolution we collect 8 timer values -> 16 captures in 720°)
    range check: 2^4*2^16=2^20 -> fits to 32 bit variable
    */
    DInternals.cycle_timing_buffer += Interval;
    DInternals.cycle_timing_counter++;


    if(DInternals.cycle_timing_counter == 2* CRK_POSITION_COUNT)
    {
        /**
        crank_T_us acts as an accurate time base for following calculations

        the timer is adjusted to DECODER_TIMER_PERIOD_US interval
        cycle_timing_buffer holds the duration for 2 full crank turns in timer ticks (#T.720)
        */
        period= DInternals.cycle_timing_buffer * (DECODER_TIMER_PERIOD_US / 2);


        /**
        crank acceleration
        reflects the absolute change in rotational period since last cycle

        not yet implemented

        if( (period != 0) && (DInterface.crank_T_us != 0) )
        {
            DInterface.crank_deltaT_us= period - DInterface.crank_T_us;
        }
        */

        //save the calculated value
        DInternals.crank_period_us= period;

        //reset counters
        DInternals.cycle_timing_buffer= 0;
        DInternals.cycle_timing_counter= 0;
    }
}

inline void reset_position_data()
{
    DInternals.crank_position= CRK_POSITION_UNDEFINED;
}

inline void reset_crank_timing_data()
{
    DInternals.sync_buffer_key= 0;
    DInternals.sync_buffer_gap= 0;
    DInternals.cycle_timing_buffer= 0;
    DInternals.cycle_timing_counter= 0;
    DInternals.crank_period_us= 0;
}

inline void decoder_set_state(decoder_state_t NewState)
{
    if(NewState >= DSTATE_COUNT)
    {
        //error
        DInternals.state= DSTATE_INIT;
        return;
    }

    DInternals.state=NewState;
}

inline void decoder_update_interface()
{
    DInterface.crank_position= DInternals.crank_position;
    DInterface.crank_period_us= DInternals.crank_period_us;
}

inline void reset_timeout_counter()
{
    DInternals.timeout_count =0;
}


/*
inline void reset_diag_data()
{
    VU32 item;

    for(item= 0; item < DDIAG_COUNT; item++)
    {
        DInternals.diag[item]=0;
    }
}
*/

/*
inline void reset_sync_stability()
{
    DInternals.sync_stability =0;
}
*/


/******************************************************************************************************************************
decoder logic initialization

as the decoder timer was reset to 0x00 when the interrupt handler has been called,
its value reveals the over all delay of the following operations


performance analysis revealed:
handler entry happens about 1 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
volatile decoder_interface_t * init_decoder_logic()
{
    //interface variables
    decoder_set_state(DSTATE_INIT);

    reset_position_data();
    reset_crank_timing_data();
    decoder_update_interface();
    reset_timeout_counter();

    //calculate the decoder timeout threshold corresponding to the configured timeout value
    DInternals.decoder_timeout_thrs= ((1000UL * configPage12.decoder_timeout_s) / DECODER_TIMER_OVERFLOW_MS) +1;


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
void decoder_logic_crank_handler(VU32 Interval)
{
    //we just saw a trigger condition
    reset_timeout_counter();

    //collect diagnostic data
    decoder_diag_log_event(DDIAG_CRANKHANDLER_CALLS);

    switch(DInternals.state) {

        case DSTATE_INIT:

            /**
            this is the first impulse captured in this cycle

            it is not certain which crank sensing applied until now
            it is not certain if the decoder timer has been running

            decoder_set_crank_pickup_sensing() will keep the crank irq masked until decoder_logic_timer_compare_handler() enables it
            */
            decoder_start_timer(configPage12.crank_noise_filter);

            //prepare to detect a KEY begin
            decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            decoder_set_state(DSTATE_ASYNC);

            reset_position_data();
            reset_crank_timing_data();

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRANKPOS_INIT);

            break;

        case DSTATE_ASYNC:

            /**
            1. synchronization step
            this is the beginning of a KEY
            decoder timer and crank sensing are set up and working now
            decoder_set_crank_pickup_sensing() will keep the crank irq masked until decoder_logic_timer_compare_handler() enables it
            */
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);
            decoder_set_state(DSTATE_ASYNC_KEY);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRANKPOS_ASYNC);

            break;


        case DSTATE_ASYNC_KEY:

            /**
            2. synchronization step
            we are at the end of a key -> key duration captured
            in ASYNC mode we try to find segment A
            */
            DInternals.sync_buffer_key= Interval;

            decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            decoder_set_state(DSTATE_ASYNC_GAP);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRANKPOS_ASYNC_KEY);

            break;


        case DSTATE_ASYNC_GAP:

            /**
            we are at the beginning of a key -> next int will be on key end
            timer captured gap duration
            */
            DInternals.sync_buffer_gap= Interval;
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);

            if( check_sync_ratio() == RETURN_OK )
            {
                /**
                it was key A -> we have SYNC now!
                */
                decoder_set_state(DSTATE_SYNC);
                DInternals.crank_position= CRK_POSITION_B1;

                //collect diagnostic data
                decoder_diag_log_event(DDIAG_ASYNC_SYNC_TR);

                //run desired debug action
                got_sync_debug_handler();

            }
            else
            {
                //on any other key
                decoder_set_state(DSTATE_ASYNC_KEY);

                reset_position_data();
                reset_crank_timing_data();
            }

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRANKPOS_ASYNC_GAP);

            break;


        case DSTATE_SYNC:

            /*
            //update crank_position
            DInternals.crank_position++;

            //wrap around
            if(DInternals.crank_position == CRK_POSITION_COUNT)
            {
                DInternals.crank_position= 0;
            }
            */
            increment_crank_position(&(DInternals.crank_position));

            // update crank sensing
            decoder_set_crank_pickup_sensing(SENSING_INVERT);

            /*
            update rotational speed calculation
            */
            update_engine_speed(Interval);

            //collect diagnostic data
            decoder_diag_log_event(DDIAG_CRANKPOS_SYNC);

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

                if( check_sync_ratio() == RETURN_FAIL )
                {
                    //sync check failed!
                    reset_position_data();
                    reset_crank_timing_data();

                    //prepare for the next trigger condition
                    decoder_set_state(DSTATE_INIT);

                    //collect diagnostic data
                    decoder_diag_log_event(DDIAG_SYNC_ASYNC_TR);

                    //run desired debug actions
                    sync_lost_debug_handler();
                }
                break;

            default:
                //any other position
                break;

            } //switch DInternals.crank_position

            break; //SYNC


        default:
            //invalid state
            decoder_set_state(DSTATE_INIT);
            break;

        } //switch DInternals.sync_mode



        /**
        finally trigger the sw irq 2 for decoder output processing
        (ca. 3.2us after trigger event)

        shall be the last action in irq!
        */
        if(DInternals.state == DSTATE_SYNC)
        {
            //collect diagnostic data
            decoder_diag_log_event(DDIAG_TRIGGER_IRQ_SYNC);

            //export process data
            decoder_update_interface();

            trigger_decoder_irq();
        }

        /**
        noise filter
        crank pickup irq masked after set_crank_pickup_sensing() call until decoder_logic_timer_compare_handler() enables it
        until
        */

}


/******************************************************************************************************************************
Timer for decoder control: compare event --> enable external interrupt for pickup sensor
 ******************************************************************************************************************************/
void decoder_logic_timer_compare_handler()
{
    decoder_diag_log_event(DDIAG_TIMER_COMPARE_EVENTS);

    decoder_unmask_crank_irq();
}


/******************************************************************************************************************************
Timer for decoder control: update event --> overflow interrupt occurs when no signal from crankshaft pickup
has been received for more than the configured interval
 ******************************************************************************************************************************/
void decoder_logic_timer_update_handler()
{
    /**
    timer update indicates unstable engine operation
    */

    if(DInternals.timeout_count >= DInternals.decoder_timeout_thrs)
    {
        //reset decoder
        decoder_set_state(DSTATE_INIT);

        reset_position_data();
        reset_crank_timing_data();

        //run desired debug action
        decoder_timeout_debug_handler();

        //stopping the decoder timer will leave the crank noise filter on
        decoder_stop_timer();

        //so prepare for the next trigger condition
        decoder_unmask_crank_irq();

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_TIMEOUT_EVENTS);

        //export process data
        decoder_update_interface();

        /**
        trigger sw irq for decoder output processing
        LAST ACTION here!
        */
        trigger_decoder_irq();
    }
    else
    {
        DInternals.timeout_count++;

        //collect diagnostic data
        decoder_diag_log_event(DDIAG_TIMER_UPDATE_EVENTS);
    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/

void update_cis_sensor()
{
    /**
            cylinder identification sensor handling
            running only in sync with crank, if we lose sync in POSITION_B1 sync check:
            -> the irq enable/disable branches will not be executed as crank_position defaults to UNDEFINED
            -> the c.i.s. irq is masked already

            if(DInternals.crank_position == CYLINDER_SENSOR_ENA_POSITION)
            {
               // /// TODO (oli#1#): DEBUG
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
                    decoder_diag_log_event(DDIAG_PHASED_UNDEFINED_TR);
                }
            }

            //collect diagnostic data
            if(DInternals.phase == PHASE_UNDEFINED)
            {
                decoder_diag_log_event(DDIAG_CRANKPOS_CIS_UNDEFINED);
            }
            else
            {
                decoder_diag_log_event(DDIAG_CRANKPOS_CIS_PHASED);
            }


            //collect statistics data
            //decoder_statistics_handler(Interval);
            */








}


void decoder_logic_cam_handler()
{
    /*
    //collect diagnostic data
    decoder_diag_log_event(DDIAG_CISHANDLER_CALLS);

    //a trigger condition has been seen on c.i.s -> cylinder #1 working or sensor error
    //first trigger condition allows us to leave UNDEFINED state

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
        decoder_diag_log_event(DDIAG_PHASED_UNDEFINED_TR);
    }

    //disable c.i.s irq until next turn
    decoder_mask_cis_irq();
    */
}



