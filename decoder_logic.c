#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "uart.h"
#include "Tuareg.h"

#include "debug.h"

volatile decoder_logic_t Decoder_internals;

/**
how the decoder works:

-   on initialisation we enable crankpickup irq
-   with first crank pickup irq execution we start crank timer to measure time delay between
    crank pickup events
-   because we know sensing (rising or falling edge), we can tell if a gap or a key on trigger wheel
    has been detected
-   syncronisation we get when a certain time ratio (key/gap ratio) - thats the cycle beginning
-   from cycle beginning every rise/fall event takes us further in engine cycle - decoder keeps track of engine
    position and expects synchronization events after position D2
-   after each sensing event the timer gets a reset
-   a noise filter has been implemented: crank pickup signal edge detection irq gets enabled after some delay time after each
    irq execution to suppress signal noise
-   if a timer overflow occures, the engine most certainly has stalled / is not running

*/



/**
evaluate sync_buffer_key / sync_buffer_gap ratio
to get trigger wheel sync
*/
U32 check_for_key_a()
{
    U32 sync_ratio;

    if(Decoder_internals.sync_buffer_gap == 0)
    {
        return (U32) 0;
    }

    /*
    fixed point maths -> ratio in percent
    sync_buffer_xx is within a 1..65535 range, U32 is wide enough
    */
    sync_ratio= (Decoder_internals.sync_buffer_key * 100) / Decoder_internals.sync_buffer_gap;

    if( (sync_ratio >= SYNC_RATIO_MIN) && (sync_ratio <= SYNC_RATIO_MAX) )
    {
        //its key A!
        return 0xFFFFFFFF;
    }
    else
    {
        return (U32) 0;
    }

}


/**
    using hw layer
    enables pickup sensor interrupt
*/
volatile decoder_logic_t * init_decoder_logic()
{
    //interface variables
    Decoder_internals.sync_mode= INIT;
    Decoder_internals.crank_position= UNDEFINED_POSITION;
    Decoder_internals.phase= PHASE_UNDEFINED;

    Decoder_internals.diag_positions_cis_phased =0;
    Decoder_internals.diag_positions_cis_undefined =0;
    Decoder_internals.diag_positions_crank_async =0;
    Decoder_internals.diag_positions_crank_synced =0;

    //see explanation below
    Decoder_internals.rpm_calc_constant= 120UL * (SystemCoreClock / DECODER_TIMER_PSC);


    /**
    we are now ready to process decoder events
    */

    //react to crank irq
    decoder_unmask_crank_irq();

    return &Decoder_internals;
}



/******************************************************************************************************************************
crankshaft position sensor
 ******************************************************************************************************************************/
void decoder_logic_crank_handler(VU32 Timestamp)
{
     //DEBUG
    #warning TODO (oli#9#): Debug LED
    set_debug_led(TOGGLE);


    switch(Decoder_internals.sync_mode) {

        case INIT:

            // this is the first impulse ever captured -> no timing info available
            decoder_start_timer();
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);
            Decoder_internals.sync_mode= ASYNC_KEY;
            Decoder_internals.crank_position= UNDEFINED_POSITION;
            Decoder_internals.phase= PHASE_UNDEFINED;
            Decoder_internals.cycle_timing_buffer= 0UL;
            Decoder_internals.cycle_timing_counter= 0UL;
            break;


        case ASYNC_KEY:

            /* we are at the end of a key -> key duration captured
            in ASYNC mode we try to find segment A */
            Decoder_internals.sync_buffer_key= Timestamp;
            decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            Decoder_internals.sync_mode= ASYNC_GAP;
            Decoder_internals.crank_position= UNDEFINED_POSITION;

            //collect diagnostic data
            Decoder_internals.diag_positions_crank_async++;

            break;


        case ASYNC_GAP:

            // we are at the beginning of a key -> timer captured gap duration, next int will be on key end anyways
            Decoder_internals.sync_buffer_gap= Timestamp;
            decoder_set_crank_pickup_sensing(SENSING_KEY_END);

            if( check_for_key_a() )
            {
                // it was key A -> we have SYNC now!
                Decoder_internals.sync_mode= SYNC;
                Decoder_internals.crank_position= POSITION_B1;
                Decoder_internals.phase= PHASE_UNDEFINED;

                //prepare rpm calculation
                Decoder_internals.cycle_timing_counter= 0UL;
                Decoder_internals.cycle_timing_buffer= 0UL;
                Decoder_internals.engine_rpm= 0UL;
            }
            else
            {
                //any other key
                Decoder_internals.sync_mode= ASYNC_KEY;
                Decoder_internals.crank_position= UNDEFINED_POSITION;
            }

            //collect diagnostic data
            Decoder_internals.diag_positions_crank_async++;

            break;


        case SYNC:

            //update crank_position
            if(Decoder_internals.crank_position == POSITION_D2)
            {
                //crank cycle end -> turn around
                Decoder_internals.crank_position= POSITION_A1;
            }
            else
            {
                Decoder_internals.crank_position++;
            }

            // update crank sensing
            decoder_set_crank_pickup_sensing(INVERT);

            /*
            collect data for engine rpm calculation
            (in every crank 360° revolution we collect 8 timer values -> 16 captures in 720°)
            range check: 2^4*2^16=2^20 -> fits to 32 bit variable
            */
            Decoder_internals.cycle_timing_buffer += Timestamp;
            Decoder_internals.cycle_timing_counter++;

            //2 full crank revolutions captured?
            if(Decoder_internals.cycle_timing_counter == 16)
            {
                /**
                rpm calculation:
                n = 60 / T.360 = 120 / T.720
                (T.720 is the time for 2 revolutions)
                Decoder_internals.cycle_timing_buffer holds T.720 in timer ticks -> #T.720
                T.720 = #T.720 * T.timer
                (T.timer is the timer increment)
                T.timer = ps / f.cpu
                gives
                n = (120 * f.cpu) / ( #T.720 * ps)

                as 120, f.cpu and ps are constant, we calculate the constant part in init code
                */
                Decoder_internals.engine_rpm= Decoder_internals.rpm_calc_constant / Decoder_internals.cycle_timing_buffer;

                //reset calculation
                Decoder_internals.cycle_timing_buffer= 0;
                Decoder_internals.cycle_timing_counter= 0;
            }

            //collect diagnostic data
            Decoder_internals.diag_positions_crank_synced++;


            /**
            per-position decoder housekeeping actions:
            crank_position is the crank position that was reached at the beginning of this interrupt
            */
            switch(Decoder_internals.crank_position) {

            case POSITION_A2:

                // store key length for sync check
                Decoder_internals.sync_buffer_key= Timestamp;
                break;


            case POSITION_B1:

                //do sync check
                Decoder_internals.sync_buffer_gap= Timestamp;

                if( !check_for_key_a() )
                {
                    //sync check failed! -> prepare for the next trigger condition
                    Decoder_internals.crank_position= UNDEFINED_POSITION;
                    Decoder_internals.sync_mode= ASYNC_KEY;
                    decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);

                    //engine phase measurement is upset too
                    Decoder_internals.phase= PHASE_UNDEFINED;
                    decoder_mask_cis_irq();

                    //collect diagnostic data
                    Decoder_internals.diag_sync_lost_events++;
                }
                break;

            default:
                //any other position
                break;

            } //switch Decoder_internals.crank_position


            /**
            cylinder identification sensor handling
            running only in sync with crank, if we lose sync in POSITION_B1 sync check:
            -> the irq enable/disable branches will not be executed as crank_position defaults to UNDEFINED
            -> the c.i.s. irq is masked already
            */
            if(Decoder_internals.crank_position == CYLINDER_SENSOR_ENA_POSITION)
            {
                //enabling c.i.s irq will clear its pending flag
                decoder_unmask_cis_irq();
            }
            else if(Decoder_internals.crank_position == CYLINDER_SENSOR_DISA_POSITION)
            {
                //disable c.i.s irq
                decoder_mask_cis_irq();

                //no c.i.s trigger event has been detected -> cylinder #2 working or broken sensor
                if(Decoder_internals.phase == CYL1_WORK)
                {
                    //sensor gives valid data
                    Decoder_internals.phase= CYL2_WORK;
                }
                else if(Decoder_internals.phase == CYL2_WORK)
                {
                    //no alternating signal -> sensor error
                    Decoder_internals.phase= PHASE_UNDEFINED;

                    //collect diagnostic data
                    Decoder_internals.diag_phase_lost_events++;
                }
            }

            //collect diagnostic data
            if(Decoder_internals.phase == PHASE_UNDEFINED)
            {
                Decoder_internals.diag_positions_cis_undefined++;
            }
            else
            {
                Decoder_internals.diag_positions_cis_phased++;
            }


            /**
            finally trigger the sw irq 2 for decoder output processing
            (ca. 3.2us after trigger event)
            the irq will be triggered anyways to react on sync failures!
            */
            trigger_decoder_irq();
            break;

        } //switch Decoder_internals.sync_mode

        //reset timeout counter, we just saw a trigger condition
        Decoder_internals.timeout_count= 0UL;

        /**
        noise filter
        mask crank pickup irq, timer 9 compare will enable it again
        crank pickup irq is disabled after each set_crank_pickup_sensing() call
        */
        decoder_mask_crank_irq();

}


/******************************************************************************************************************************
Timer for decoder control: compare event --> enable external interrupt for pickup sensor
 ******************************************************************************************************************************/
void decoder_logic_timer_compare_handler()
{
    decoder_unmask_crank_irq();
}


/******************************************************************************************************************************
Timer for decoder control: update event --> overflow interrupt occurs when no signal from crankshaft pickup has been received for more then 4s
 ******************************************************************************************************************************/
void decoder_logic_timer_update_handler()
{
    //timer update occurs every 200 ms
    if(Decoder_internals.timeout_count >= DECODER_TIMEOUT)
    {
        //reset decoder
        Decoder_internals.sync_mode= INIT;
        Decoder_internals.crank_position= UNDEFINED_POSITION;

        //reset rpm calculation
        Decoder_internals.cycle_timing_buffer= 0UL;
        Decoder_internals.cycle_timing_counter= 0UL;
        Decoder_internals.engine_rpm= 0UL;

        //trigger sw irq for decoder output processing (ca. 3.2us behind trigger edge)
        trigger_decoder_irq();

        //prepare for the next trigger condition
        decoder_stop_timer();
        decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);
        decoder_unmask_crank_irq();

        //shut down c.i.s. irq
        decoder_mask_cis_irq();
        Decoder_internals.phase= PHASE_UNDEFINED;
    }
    else
    {
        Decoder_internals.timeout_count++;
    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/
void decoder_logic_cam_handler()
{
    /**
    a trigger condition has been seen on c.i.s -> cylinder #1 working or sensor error
    first trigger condition allows us to leave UNDEFINED state
    */
    if((Decoder_internals.phase == CYL2_WORK) || (Decoder_internals.phase == PHASE_UNDEFINED))
    {
        //alternating signal -> valid
        Decoder_internals.phase= CYL1_WORK;
    }
    else if(Decoder_internals.phase == CYL1_WORK)
    {
        //invalid signal
        Decoder_internals.phase= PHASE_UNDEFINED;

        //collect diagnostic data
        Decoder_internals.diag_phase_lost_events++;
    }

    //disable c.i.s irq until next turn
    decoder_mask_cis_irq();
}



