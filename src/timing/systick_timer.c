#include <Tuareg_platform.h>
#include <Tuareg.h>



volatile systick_mgr_t Systick_Mgr;

const U32 cSystickLoadValue= 12500UL;

/**
see PM0214 Rev 8:

"When the processor is halted for debugging the counter does not decrement."

"The SysTick counter runs on the processor clock. If this clock signal is stopped for low
power mode, the SysTick counter stops.
Ensure software uses aligned word accesses to access the SysTick registers."

"The SysTick counter reload and current value are undefined at reset, the correct
initialization sequence for the SysTick counter is:
1. Program reload value.
2. Clear current value.
3. Program Control and Status register."

*/

volatile systick_t * init_systick_timer()
{
    /**
    using AHPB/8 -> T := 0,08 us
    SysTick irq provides a 1ms time base
    irq priority 5
    */
    SysTick->LOAD = (U32) (12500UL & SysTick_LOAD_RELOAD_Msk);
    SysTick->VAL  = (U32) 0;
    SysTick->CTRL = (U32) SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    NVIC_SetPriority(SysTick_IRQn, 5);

    return &(Systick_Mgr.out);
}



/**
The systick timer is a 24 bit down counter, T := 0,08 us
it is reset every 1 ms

fraction := (0xFFFFFF - STV_VAL) * 8 / 100
*/
VU32 get_timestamp_fraction_us()
{
    VU32 interval;

    //get the amount of timer ticks since the last timer reset
    interval= cSystickLoadValue - SysTick->VAL;

    //multiply *2 and divide by 25
    interval= (interval << 1) / 25;

    return interval;
}





void SysTick_Handler(void)
{
    //1 ms cycle - 1 kHz
    Systick_Mgr.system_time++;
    Systick_Mgr.out.system_time= Systick_Mgr.system_time;

    /**
    no further action will be triggered while the system is in initialization phase
    */
    if(Tuareg.errors.init_not_completed == true)
    {
        return;
    }

    //injector_1 watchdog
    if((Tuareg.flags.fuel_injector_1 == true) && (Tuareg.injector1_watchdog_ms < cU32max))
    {
        Tuareg.injector1_watchdog_ms += 1;
    }

    //injector_2 watchdog
    if((Tuareg.flags.fuel_injector_2 == true) && (Tuareg.injector2_watchdog_ms < cU32max))
    {
        Tuareg.injector2_watchdog_ms += 1;
    }

    //run service functions update in irq context
    if(Tuareg.flags.service_mode == true)
    {
        service_functions_periodic_update();
    }


    Systick_Mgr.counter_10_ms++;
  //  Systick_Mgr.counter_20_ms++;
  //  Systick_Mgr.counter_33_ms++;
  //  Systick_Mgr.counter_66_ms++;
    Systick_Mgr.counter_100_ms++;
  //  Systick_Mgr.counter_250_ms++;
    Systick_Mgr.counter_1000_ms++;


    //10 ms cycle - 100 Hz
    if (Systick_Mgr.counter_10_ms == 10)
    {
        Systick_Mgr.counter_10_ms = 0;
        Systick_Mgr.out.flags.cycle_10_ms= true;

        //actions to be taken in irq scope
        if(Tuareg.errors.fatal_error == false)
        {
            //trigger ADC conversion for analog sensors
            sensors_start_regular_group_conversion();

            //update digital sensor values
            update_digital_sensors();

            /// TODO (oli#3#03/03/22): what will happen when a injected conversion is commanded while a regular conversion is ongoing?

            /*
            at engine standstill no decoder irq can trigger any function
            do this timer based
            */
            if(Tuareg.pDecoder->flags.standstill == true)
            {
                //provide MAP value for the stalled engine
                sensors_start_injected_group_conversion();

                Tuareg_update_process_data();
                Tuareg_update_ignition_controls();
                Tuareg_update_fueling_controls();
            }

            //update system state based on the data gathered
            Tuareg_update_systick();
        }
    }


/*
    //20 ms cycle - 50 Hz
    if (Systick_Mgr.counter_20_ms == 20)
    {
        Systick_Mgr.counter_20_ms = 0;
        Systick_Mgr.out.flags.cycle_20_ms= true;

        //actions to be taken in irq scope
    }

    //33 ms cycle - 30 Hz
    if (Systick_Mgr.counter_33_ms == 33)
    {
        Systick_Mgr.counter_33_ms = 0;
        Systick_Mgr.out.flags.cycle_33_ms= true;

        //actions to be taken in irq scope


    }

    //66 ms cycle - 15 Hz
    if (Systick_Mgr.counter_66_ms == 66)
    {
        Systick_Mgr.counter_66_ms = 0;
        Systick_Mgr.out.flags.cycle_66_ms= true;

        //actions to be taken in irq scope

    }
*/


    //100 ms cycle - 10 Hz
    if (Systick_Mgr.counter_100_ms == 100)
    {
        Systick_Mgr.counter_100_ms = 0;
        Systick_Mgr.out.flags.cycle_100_ms= true;

        //actions to be taken in irq scope
        update_dash();
    }

/*
    //250 ms cycle - 4 Hz
    if (Systick_Mgr.counter_250_ms == 250)
    {
        Systick_Mgr.counter_250_ms = 0;
        Systick_Mgr.out.flags.cycle_250_ms= true;

        //actions to be taken in irq scope

    }
*/

    //1000 ms cycle - 1 Hz
    if (Systick_Mgr.counter_1000_ms == 1000)
    {
        Systick_Mgr.counter_1000_ms = 0;
        Systick_Mgr.out.flags.cycle_1000_ms= true;

        //actions to be taken in irq scope

        //update decoder watchdog
        if(Tuareg.decoder_watchdog < 0xFFFFFFFF)
        {
            Tuareg.decoder_watchdog += 1;
        }
    }
}
