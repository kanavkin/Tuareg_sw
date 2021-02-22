#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"
#include "systick_timer.h"

#include "Tuareg.h"
#include "sensors.h"
#include "Tuareg_console.h"
#include "scheduler.h"
#include "Tuareg_service_functions.h"



volatile systick_mgr_t Systick_Mgr;


volatile systick_t * init_systick_timer()
{
    /**
    using AHPB/8
    SysTick provides a 1ms time base
    irq priority 5
    */
    SysTick->LOAD = (12500UL & SysTick_LOAD_RELOAD_Msk);
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    NVIC_SetPriority(SysTick_IRQn, 5);

    return &(Systick_Mgr.out);
}



void SysTick_Handler(void)
{
    Systick_Mgr.counter_10_ms++;
    Systick_Mgr.counter_20_ms++;
    Systick_Mgr.counter_33_ms++;
    Systick_Mgr.counter_66_ms++;
    Systick_Mgr.counter_100_ms++;
    Systick_Mgr.counter_250_ms++;
    Systick_Mgr.counter_1000_ms++;


    //1 ms cycle - 1 kHz
    Systick_Mgr.system_time++;
    Systick_Mgr.out.system_time= Systick_Mgr.system_time;

    //run service functions update in irq context
    if(Tuareg.Runmode == TMODE_SERVICE)
    {
        service_functions_periodic_update();
    }



    //10 ms cycle - 100 Hz
    if (Systick_Mgr.counter_10_ms == 10)
    {
        Systick_Mgr.counter_10_ms = 0;
        Systick_Mgr.out.flags.cycle_10_ms= true;

        /*
        actions to be taken in irq scope
        */

        //trigger ADC conversion for analog sensors
        adc_start_regular_group(SENSOR_ADC);
    }


    //20 ms cycle - 50 Hz
    if (Systick_Mgr.counter_20_ms == 20)
    {
        Systick_Mgr.counter_20_ms = 0;
        Systick_Mgr.out.flags.cycle_20_ms= true;

        /*
        actions to be taken in irq scope
        */

        //update digital sensor values
        read_digital_sensors();
    }


    //33 ms cycle - 30 Hz
    if (Systick_Mgr.counter_33_ms == 33)
    {
        Systick_Mgr.counter_33_ms = 0;
        Systick_Mgr.out.flags.cycle_33_ms= true;

        /*
        actions to be taken in irq scope
        */

    }


    //66 ms cycle - 15 Hz
    if (Systick_Mgr.counter_66_ms == 66)
    {
        Systick_Mgr.counter_66_ms = 0;
        Systick_Mgr.out.flags.cycle_66_ms= true;

        /*
        actions to be taken in irq scope
        */

    }


    //100 ms cycle - 10 Hz
    if (Systick_Mgr.counter_100_ms == 100)
    {
        Systick_Mgr.counter_100_ms = 0;
        Systick_Mgr.out.flags.cycle_100_ms= true;

        /*
        actions to be taken in irq scope
        */

    }


    //250 ms cycle - 4 Hz
    if (Systick_Mgr.counter_250_ms == 250)
    {
        Systick_Mgr.counter_250_ms = 0;
        Systick_Mgr.out.flags.cycle_250_ms= true;

        /*
        actions to be taken in irq scope
        */

    }


    //1000 ms cycle - 1 Hz
    if (Systick_Mgr.counter_1000_ms == 1000)
    {
        Systick_Mgr.counter_1000_ms = 0;
        Systick_Mgr.out.flags.cycle_1000_ms= true;

        /*
        actions to be taken in irq scope
        */

        //console
        cli_cyclic_update();

    }



}
