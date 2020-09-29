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


#include "decoder_hw.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fuel_hw.h"
#include "fuel_logic.h"

#include "dash_hw.h"
#include "dash_logic.h"
#include "act_hw.h"
#include "act_logic.h"


#include "debug.h"
#include "Tuareg.h"



/**
global status object
*/
volatile Tuareg_t Tuareg;


#warning TODO (oli#1#): implement memset function



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
    U32 config_load_status;

    /**
    starting primary hw initialisation
    */
    Tuareg_set_Runmode(TMODE_HWINIT);

    //use 16 preemption priority levels
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    UART_DEBUG_PORT_Init();
    UART_Send(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
    UART_Send(DEBUG_PORT, "RC 0001");
    UART_Send(DEBUG_PORT, "\r \n config: \r \n");
    UART_Send(DEBUG_PORT, "XTZ 660 digital crank signal on GPIOB-0 \r \n");
    UART_Send(DEBUG_PORT, "\r \n XTZ 660 ignition coil signal on GPIOC-6 \r \n");

    UART_TS_PORT_Init();
    UART_Send(DEBUG_PORT, "TunerStudio interface ready \r\n");

    /**
    initialize core components
    */
    init_decoder_hw();
    init_ignition_hw();
    init_fuel_hw();
    init_dash_hw();
    init_act_hw();
    init_eeprom();

    //DEBUG
    //init_debug_pins();
    //dwt_init();

     //serial monitor
    #ifdef SERIAL_MONITOR
    UART1_Send("\r \n serial monitor loaded!");
    #endif


    #warning TODO (oli#4#): implement config item set/read logic
    configPage13.dynamic_ignition_position= CRK_POSITION_A2;


    /**
    ready to load config data from eeprom
    */
    Tuareg_set_Runmode(TMODE_CONFIGLOAD);

    //loading the config data is important to us, failure in loading forces "limp home mode"
    config_load_status= config_load();

/**
#warning TODO (oli#1#): DEBUG: limp home test
config_load_status= RETURN_FAIL;
*/

    if(config_load_status != RETURN_OK)
    {
        //save to error register
        Tuareg.Errors |= (1<< TERROR_CONFIGLOAD);

        //as we can hardly save some error logs in this system condition, print some debug messages only
        UART_Send(DEBUG_PORT, "\r \n *** FAILED to load config data !");

        Tuareg_set_Runmode(TMODE_LIMP);

        //provide default values to ensure limp home operation even without eeprom
        config_load_essentials();
    }
    else
    {
        /**
        prepare config data
        */

        //set 2D table dimension and link table data to config pages
        init_2Dtables();
        //actually sets table dimension, could be removed
        init_3Dtables();

        /**
        ready to initialize further system hardware
        */
        Tuareg_set_Runmode(TMODE_MODULEINIT);
    }

    //initialize core components and register interface access pointers
    Tuareg.sensors= init_sensors();
    Tuareg.decoder= init_decoder_logic();
    init_scheduler();
    init_lowspeed_timers();
    init_lowprio_scheduler();
    init_fuel_logic();
    init_dash_logic();
    init_act_logic();


    /**
    system initialization has been completed, but never leave LIMP mode!
    */
    if(Tuareg.Runmode != TMODE_LIMP )
    {
        Tuareg_set_Runmode(TMODE_HALT);
    }



    while(1)
    {
        //debug
        //poll_dwt_printout();

        //collect diagnostic information
        Tuareg.diag[TDIAG_MAINLOOP_ENTRY] += 1;

        /**
        15 Hz actions
        */
        if(ls_timer & BIT_TIMER_15HZ)
        {
            ls_timer &= ~BIT_TIMER_15HZ;

            //update digital sensor values
            read_digital_sensors();
        }

        /**
        1 Hz actions
        */
        if( ls_timer & BIT_TIMER_1HZ)
        {
            ls_timer &= ~BIT_TIMER_1HZ;

            //provide sensor data
            if((Tuareg.Runmode == TMODE_HALT) || (Tuareg.Runmode == TMODE_STB))
            {
#warning TODO (oli#8#): who will provide procass data in limp mode with engine halted?

                //start MAP sensor conversion
                adc_start_injected_group(SENSOR_ADC);

                Tuareg_update_process_data(&(Tuareg.process));
            }


            //calculate new system state
            Tuareg_update_Runmode();


            //print_sensor_data(DEBUG_PORT);

        }

        /**
        handle TS communication
        */
        if( (ls_timer & BIT_TIMER_10HZ) || (UART_available() > SERIAL_BUFFER_THRESHOLD) )
        //if(ls_timer & BIT_TIMER_4HZ)
        {
            ls_timer &= ~BIT_TIMER_10HZ;

            //serial monitor
            #ifdef SERIAL_MONITOR
            while( monitor_available() )
            {
                monitor_print();
            }
            #endif // SERIAL_MONITOR

            //if (UART_available() > 0)
           // {
                ts_communication();

                //collect diagnostic information
                Tuareg.diag[TDIAG_TSTUDIO_CALLS] += 1;

           // }
        }


    }

    return 0;
}





/******************************************************************************************************************************
sw generated irq when decoder has
updated crank_position based on crank pickup signal
or decoder timeout occurred!

The irq can be entered in HALT Mode when the crank is already spinning but the RUN switch has not yet been evaluated

performance analysis revealed:
handler entry happens about 3 us after the trigger signal edge had occurred
 ******************************************************************************************************************************/
void EXTI2_IRQHandler(void)
{
    //clear pending register
    EXTI->PR= EXTI_Line2;

    //start MAP sensor conversion
    adc_start_injected_group(SENSOR_ADC);

    //collect diagnostic information
    Tuareg.diag[TDIAG_DECODER_IRQ] += 1;
    Tuareg.diag[TDIAG_DECODER_AGE]= decoder_get_data_age_us();

    /**
    check if this is a decoder timeout (engine has stalled) or a regular crank position update event
    */
    switch(Tuareg.Runmode)
    {

        case TMODE_CRANKING:
        case TMODE_RUNNING:

            if(Tuareg.decoder->crank_position != CRK_POSITION_UNDEFINED)
            {
                /**
                normal engine operation
                */

                //trigger dwell or spark
                Tuareg_trigger_ignition();

            }
            else
            {
                /**
                decoder timeout
                */

                //collect diagnostic information
                Tuareg.diag[TDIAG_DECODER_TIMEOUT] += 1;

                Tuareg_set_Runmode(TMODE_STB);

            }

            break;


        case TMODE_STB:


            if(Tuareg.decoder->crank_position != CRK_POSITION_UNDEFINED)
            {
                /**
                first position detected -> cranking has begun
                */
                Tuareg_set_Runmode(TMODE_CRANKING);

            }
            else
            {
                /**
                decoder timeout
                */
                //collect diagnostic information
                Tuareg.diag[TDIAG_DECODER_TIMEOUT] += 1;
            }

            break;


        case TMODE_LIMP:

            //if((Tuareg.decoder->engine_rpm > 0) && (Tuareg.decoder->crank_position != CRK_POSITION_UNDEFINED))
            if(Tuareg.decoder->crank_position != CRK_POSITION_UNDEFINED)
            {
                //trigger dwell or spark
                Tuareg_trigger_ignition();
            }

            break;


        default:

            /**
            possible scenario:
            -engine has been killed by kill switch and the crank shaft is still rotating
            -in DIAG mode we will benefit from the updated crank data...
            */

            //collect diagnostic information
            Tuareg.diag[TDIAG_DECODER_PASSIVE] += 1;

            break;

     }

}

/******************************************************************************************************************************
sw generated irq when spark has fired
-> recalculate ignition timing
 ******************************************************************************************************************************/
void EXTI3_IRQHandler(void)
{
    //clear pending register
    EXTI->PR= EXTI_Line3;

    //collect diagnostic information
    Tuareg.diag[TDIAG_IGNITION_IRQ] += 1;

    /**
    recalculate ignition timing
    */
    Tuareg_update_process_data(&(Tuareg.process));
    Tuareg_update_ignition_timing();
}




