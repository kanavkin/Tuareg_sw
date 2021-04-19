/**
This is a simplified ignition system for
Yamaha XTZ 660 on STM32 platform

Note: Denso TNDF13 has inverted ignition transistor output logic
 */

/**
a little performance analysis:

    This was with ATmega328p @ 16 MHz:
        -calc_rpm() takes about 39 us
        -calc_ignition_timings() takes about 103,5 us
        -we are entering this section about 17,6 us after the corresponding
            trigger signal edge
        -all the ignition calculation is finished after about 160 us
            after the corresponding trigger signal edge of IGNITION_RECALC_POSITION
            this is about 8 deg @ 8500 rpm
        -the ignition timer is set up after about 22,2 us after POSITION_A2
            trigger signal edge, this is about 1,13 deq @ 8500 rpm (0.7 deg at average rpms)

    In STM32F103 @ 8 MHz:
        -ignition set up time (delay from pickup signal trigger to transistor signal change) is about 4us
        -accuracy: measured 37,6° advance @ 8500 rpm when 38° was set up (7,4us off)

    In STM32F103 @ 64 MHz
        -decoder EXTI triggers around 1,2 us after crank signal edge
        -decoder SW IRQ triggers around 9,2 us after crank signal edge
        ->decoder IRQ consumes around 8 us

*/

/*
TODO:
transition to 64 MHz clock speed

affected:

UARTs
-> not affected, baudrate setting through spl init function (takes care of cpu clock)

EEPROM
-> not affected, i2c clock speed setting through spl function

ADC (sensors)
-> possibly affected, adc clock prescaler set to 6 (fits 64 and 72 MHz)

DECODER
-> rpm calculation based on DECODER_CPU_CLK_MHZ value from decoder.h
-> timer prescaler setting affected
-> decoder timeout configuration (update event) affected
-> crank noise filter configuration (compare event) affected

SCHEDULER (ignition)
-> now self aligning to target scheduler period

*/



#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg.h"

#include "Tuareg_decoder.h"
#include "Tuareg_ignition.h"
#include "Tuareg_fueling.h"
#include "scheduler.h"
#include "Tuareg_console.h"

#include "diagnostics.h"
#include "debug_port_messages.h"
#include "syslog.h"


/**
process data shall be updated prior to
* ignition controls update
* fueling controls update
*/
#define PROCESS_DATA_UPDATE_POSITION CRK_POSITION_B1


/**
global status object
*/
volatile Tuareg_t Tuareg;


/// TODO (oli#5#): implement memset function?



/**
Tuareg IRQ priorities:

1    decoder:
    crank pickup (EXTI) -> EXTI0_IRQn
    crank pickup filter (timer 2) -> TIM2_IRQn NEW: TIM1_BRK_TIM9_IRQn

2   scheduler (timer 3) -> TIM3_IRQn

3   cis (part of decoder) (EXTI) -> EXTI1_IRQn

4   decoder (sw EXTI) -> EXTI2_IRQn

5   lowspeed timer (systick) -> SysTick_IRQn

6   ADC injected conversion complete -> ADC1_2_IRQn

7   ADC conversion complete (DMA transfer) -> DMA1_Channel1_IRQn

10  ignition timing recalculation (sw EXTI) -> EXTI3_IRQn

14  tunerstudio (usart 1) -> USART1_IRQn

15  debug com  (usart 6) -> USART6_IRQn -> not active,
    lowprio_scheduler (TIM1_TRG_COM_TIM11_IRQn)

*/

/**
Tuareg EXTI ressources:

EXTI0:  PORTB0  --> crank pickup signal

EXTI1:

EXTI2:  -sw-    --> decoder int

EXTI3:  -sw-    --> ignition irq

EXTI4: ?

*/


/**
allocated timers:

lowspeed_timers: derived from SysTick

decoder: timer2 (16 bit general-purpose timer) --> new: TIM9 (16 bit general-purpose timer)

scheduler: timer 3  (16 bit general-purpose timer) --> new: TIM5 (32 bit general-purpose timer)

lowprio_scheduler: TIM11 (32 bit general-purpose timer)

sensors: no timers





*/
int main(void)
{
    Tuareg_Init();

    while(1)
    {

        if(Tuareg.errors.fatal_error == true)
        {
            //FATAL mode to be implemented soon ...
            DebugMsg_Error("fatal mode");
            break;
        }

        /**
        50 Hz actions
        */
        if( Tuareg.pTimer->flags.cycle_20_ms == true)
        {
            Tuareg.pTimer->flags.cycle_20_ms= false;

            //provide MAP value for the stalled engine
            if(Tuareg.pDecoder->outputs.standstill == true)
            {
                //start MAP sensor conversion
                adc_start_injected_group(SENSOR_ADC);

                Tuareg_update_process_data();

                Tuareg_update_ignition_controls();
                Tuareg_update_fueling_controls();
            }

            //print debug messages from decoder
            decoder_process_debug_events();
        }


        /**
        10 Hz actions
        if( Tuareg.pTimer->flags.cycle_100_ms == true)
        {
            Tuareg.pTimer->flags.cycle_100_ms= false;


        }
        */


        /**
        handle console
        */
        if( (Tuareg.pTimer->flags.cycle_66_ms == true) || (UART_available() > SERIAL_BUFFER_THRESHOLD) )
        {
            Tuareg.pTimer->flags.cycle_66_ms= false;

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_TSTUDIO_CALLS);

            Tuareg_update_console();
        }

    }


    return 0;
}


/******************************************************************************************************************************
Update crankpos irq

sw generated irq when decoder has updated crank_position based on crank pickup signal or decoder timeout occurred

as the ignition / fueling timing relies on this position update event, it shall be implemented stream lined


The crank decoder will not provide any crank velocity information for the first 2..3 crank revolutions after getting sync

The irq can be entered in HALT Mode when the crank is still spinning but the RUN switch has not yet been evaluated.

performance analysis revealed:
handler entry happens about 3 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
void EXTI2_IRQHandler(void)
{
    //clear pending register
    EXTI->PR= EXTI_Line2;

    //start MAP sensor conversion
    adc_start_injected_group(SENSOR_ADC);

    /*
    //collect diagnostic information
    //tuareg_diag_log_event(TDIAG_DECODER_IRQ);

    //check for decoder timeout
    if(Tuareg.pDecoder->outputs.timeout == true)
    {
        Tuareg.flags.standstill= true;
        Tuareg.flags.cranking= false;

        //collect diagnostic information
        //tuareg_diag_log_event(TDIAG_DECODER_TIMEOUT);

        //delete ignition controls
        Tuareg_update_ignition_controls();

        //delete fuel controls
        Tuareg_update_fueling_controls();

        return;
    }
    */

    //reset decoder watchdog
    Tuareg.decoder_watchdog= 0;


    //check if process data shall be updated
    if(Tuareg.pDecoder->crank_position == PROCESS_DATA_UPDATE_POSITION)
    {
        //update process table with data supplied by decoder
        update_process_table( (Tuareg.pDecoder->outputs.period_valid == true)? (Tuareg.pDecoder->crank_period_us) : 0 );

        //update process data
        Tuareg_update_process_data();
    }

    //trigger the ignition module
    Tuareg_ignition_update_crankpos_handler();

    //trigger the fueling module
    Tuareg_fueling_update_crankpos_handler();

    ///tuareg_diag_log_event(TDIAG_DECODER_UPDATE);

}



/******************************************************************************************************************************
sw generated irq when a spark has fired
******************************************************************************************************************************/
void EXTI3_IRQHandler(void)
{
    //clear pending register
    EXTI->PR= EXTI_Line3;

    /*
    main task here is to turn on the coils for dwell in dynamic mode
    */
    Tuareg_ignition_irq_handler();


    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_IGNITION_IRQ);

}





