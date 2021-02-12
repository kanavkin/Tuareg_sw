#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"
#include "lowspeed_timers.h"

#include "Tuareg.h"
#include "sensors.h"
#include "Tuareg_console.h"
#include "scheduler.h"


VU32 loop20ms;
VU32 loop33ms;
VU32 loop66ms;
VU32 loop100ms;
VU32 loop250ms;
VS32 loopSec;


VU32 ls_timer;

VU32 system_time;



void init_lowspeed_timers()
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
}



void SysTick_Handler(void)
{
    loop20ms++;
    loop33ms++;
    loop66ms++;
    loop100ms++;
    loop250ms++;
    loopSec++;


    //50Hz loop
    if (loop20ms == 20)
    {
        //trigger ADC conversion for analog sensors
        adc_start_regular_group(SENSOR_ADC);

        loop20ms = 0;
        ls_timer |= BIT_TIMER_50HZ;
    }

    //15Hz loop
    if (loop66ms == 66)
    {
        loop66ms = 0;
        ls_timer |= BIT_TIMER_15HZ;
    }

    //30Hz loop
    if (loop33ms == 33)
    {
        //update digital sensor values
        read_digital_sensors();

        loop33ms = 0;
        ls_timer |= BIT_TIMER_30HZ;
    }

    //10Hz loop
    if (loop100ms == 100)
    {
        loop100ms = 0;
        ls_timer |= BIT_TIMER_10HZ;
    }

    //4Hz loop
    if (loop250ms == 250)
    {
        loop250ms = 0;
        ls_timer |= BIT_TIMER_4HZ;
    }

    //1Hz loop
    if (loopSec == 1000)
    {
        //console timer
        cli_update_secl();

        //console watchdog
        cli_update_watchdog();

        loopSec = 0;
        ls_timer |= BIT_TIMER_1HZ;
    }

    /**
    system time stamp
    T= 1ms
    */
    system_time++;

}
