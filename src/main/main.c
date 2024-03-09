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
#include <Tuareg_platform.h>
#include <Tuareg.h>


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


/**
Tuareg IRQ priorities:

1    decoder:
    crank pickup (EXTI) -> EXTI0_IRQn
    crank pickup filter (timer 9) -> TIM1_BRK_TIM9_IRQn

2   scheduler (timer 5) -> TIM5_IRQn

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

decoder: TIM9 (16 bit general-purpose timer)

scheduler: TIM5 (32 bit general-purpose timer)

lowprio_scheduler: TIM11 (32 bit general-purpose timer)

sensors: no timers


STM32F410 available timers: TIM1, TIM5, TIM6, TIM9, TIM11, LPTIM1


IRQs:

    TIM1_BRK_TIM9_IRQn

    TIM1_UP_TIM10_IRQn
    TIM1_TRG_COM_TIM11_IRQn

    TIM1_CC_IRQn


    TIM5_IRQn

    TIM6_DAC_IRQn

    LPTIM1_IRQn

*/




int main(void)
{

    Tuareg_Init();


    while(1)
    {
        /**
        100 Hz actions
        */
        if( Tuareg.pTimer->flags.cycle_10_ms == true)
        {
            Tuareg.pTimer->flags.cycle_10_ms= false;

            //console - even in fatal mode!
            Tuareg_update_console();
        }

        /**
        10 Hz actions
        */
        if( Tuareg.pTimer->flags.cycle_100_ms == true)
        {
            Tuareg.pTimer->flags.cycle_100_ms= false;

            if(Tuareg.errors.fatal_error == false)
            {
                Tuareg_update_trip();
            }
        }


        /**
        1 Hz actions
        */
        if( Tuareg.pTimer->flags.cycle_1000_ms == true)
        {
            Tuareg.pTimer->flags.cycle_1000_ms= false;

            //console timer
            cli_cyclic_update();

            if(Tuareg.errors.fatal_error == false)
            {
                //update fuel consumption statistics
                Tuareg_update_consumption_data();

                //fuel pump priming
                if(Tuareg.fuel_pump_priming_remain_s > 0)
                {
                    Tuareg.fuel_pump_priming_remain_s -= 1;
                }
            }

        }
    }


    return 0;
}


/******************************************************************************************************************************
Update crankpos irq

sw generated irq when decoder has updated crank_position based on crank pickup signal or decoder timeout occurred

as the ignition / fueling timing relies on this position update event, it shall be implemented stream lined


The crank decoder will not trigger this irq if no valid rpm figure is available, especially on timeout

The irq can be entered in HALT Mode when the crank is still spinning but the RUN switch has not yet been evaluated.

performance analysis revealed:
handler entry happens about 3 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
void EXTI2_IRQHandler(void)
{
    //clear pending register
    EXTI->PR= EXTI_Line2;

    //reset decoder watchdog
    Tuareg.decoder_watchdog= 0;

    //no engine operation in Fatal state
    if(Tuareg.errors.fatal_error == true)
    {
        return;
    }

    //start MAP sensor conversion
    adc_start_injected_group(SENSOR_ADC);


    //check if essential decoder data is valid
    VitalAssert( Tuareg.Decoder.flags.position_valid == true, TID_MAIN, TUAREG_LOC_DECODER_INT_POSITION_ERROR);
    VitalAssert( Tuareg.Decoder.flags.period_valid == true, TID_MAIN, TUAREG_LOC_DECODER_INT_PERIOD_ERROR);
    VitalAssert( Tuareg.Decoder.flags.rpm_valid == true, TID_MAIN, TUAREG_LOC_DECODER_INT_RPM_ERROR);


    /**
    ignition and fueling controls calculation requires the process data to be updated
    */
    if(Tuareg.Decoder.crank_position == cTuareg_controls_update_pos)
    {
        //updates process data and ignition, fueling controls
        Tuareg_update_controls();
    }

    //trigger the ignition module
    Tuareg_ignition_update_crankpos_handler();

    //trigger the fueling module
    Tuareg_fueling_update_crankpos_handler();


    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_DECODER_UPDATE);
}

