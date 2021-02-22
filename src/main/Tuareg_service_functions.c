#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "Tuareg_decoder.h"

#include "Tuareg_ignition.h"
#include "ignition_hw.h"
#include "ignition_config.h"

#include "Tuareg_sensors.h"

#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "systick_timer.h"
#include "Tuareg_console.h"
#include "Tuareg_config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fuel_hw.h"
#include "fuel_logic.h"

#include "dash_hw.h"
#include "dash_logic.h"
#include "act_hw.h"
#include "act_logic.h"

#include "process_table.h"

#include "diagnostics.h"
#include "Tuareg.h"
#include "uart_printf.h"
#include "module_test.h"

#include "syslog.h"
#include "debug_port_messages.h"
#include "Tuareg_syslog_locations.h"

#include "Tuareg_service_functions.h"


service_mgr_t Service_mgr;


/**
These service functions assume a linear system time with 1 ms increment and cyclic updates at 1 ms!
*/


void init_service_functions()
{


}




void activate_fuel_pump(U32 Timeout_ms)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        DebugMsg_Warning("fuel pump service activation not permitted!");
        return;
    }

    if(Timeout_ms == 0)
    {
        DebugMsg_Warning("fuel pump service activation parameter invalid");
        return;
    }

    if(Timeout_ms > SERVICE_FUELPUMP_MAX_TIMEOUT)
    {
        Timeout_ms= SERVICE_FUELPUMP_MAX_TIMEOUT;
        DebugMsg_Warning("fuel pump service activation interval clipped!");
    }

    //store deactivation timestamp
    Service_mgr.fuel_pump_timeout= Tuareg.pTimer->system_time + Timeout_ms;

    //take over control
    Service_mgr.flags.fuel_pump_control= true;

    //command fuel hardware
    set_fuel_pump_powered();

}




void activate_injector1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        DebugMsg_Warning("injector 1 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        DebugMsg_Warning("injector 1 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON;
        DebugMsg_Warning("injector 1 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF;
        DebugMsg_Warning("injector 1 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;
        DebugMsg_Warning("injector 1 service activation cycles clipped!");
    }

    Service_mgr.injector1_on_ms= On_time_ms;
    Service_mgr.injector1_off_ms= Off_time_ms;
    Service_mgr.injector1_cycle_count= Cycles;

    //store toggle timestamp
    Service_mgr.injector1_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.injector1_control= true;

    //command fuel hardware
    set_injector1_powered();
}

void activate_injector2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        DebugMsg_Warning("injector 2 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        DebugMsg_Warning("injector 2 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON;
        DebugMsg_Warning("injector 2 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF;
        DebugMsg_Warning("injector 2 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;
        DebugMsg_Warning("injector 2 service activation cycles clipped!");
    }

    Service_mgr.injector2_on_ms= On_time_ms;
    Service_mgr.injector2_off_ms= Off_time_ms;
    Service_mgr.injector2_cycle_count= Cycles;

    //store toggle timestamp
    Service_mgr.injector2_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.injector2_control= true;

    //command fuel hardware
    set_injector2_powered();
}

void activate_coil1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        DebugMsg_Warning("coil 1 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        DebugMsg_Warning("coil 1 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON;
        DebugMsg_Warning("coil 1 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF;
        DebugMsg_Warning("coil 1 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;
        DebugMsg_Warning("coil 1 service activation cycles clipped!");
    }

    Service_mgr.coil1_on_ms= On_time_ms;
    Service_mgr.coil1_off_ms= Off_time_ms;
    Service_mgr.coil1_cycle_count= Cycles;

    //store toggle timestamp
    Service_mgr.coil1_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.coil1_control= true;

    //command fuel hardware
    set_coil1_powered();
}


void activate_coil2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        DebugMsg_Warning("coil 2 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        DebugMsg_Warning("coil 2 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON;
        DebugMsg_Warning("coil 2 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF;
        DebugMsg_Warning("coil 2 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;
        DebugMsg_Warning("coil 2 service activation cycles clipped!");
    }

    Service_mgr.coil2_on_ms= On_time_ms;
    Service_mgr.coil2_off_ms= Off_time_ms;
    Service_mgr.coil2_cycle_count= Cycles;

    //store toggle timestamp
    Service_mgr.coil2_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.coil2_control= true;

    //command hardware
    set_coil2_powered();
}



void service_functions_periodic_update()
{
    timestamp_t now;

    now= Tuareg.pTimer->system_time;

    //check other restrictions


    //check if fuel pump is active? needed?



    //check fuel pump timeout
    if((Service_mgr.flags.fuel_pump_control == true) && (now >= Service_mgr.fuel_pump_timeout))
    {
        Service_mgr.flags.fuel_pump_control= false;
        Service_mgr.fuel_pump_timeout= 0;

        //command fuel hardware
        set_fuel_pump_unpowered();
    }

    //update injector 1
    if((Service_mgr.flags.injector1_control == true) && (now >= Service_mgr.injector1_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.actors.fuel_injector_1 == true)
        {

            set_injector1_unpowered();

            //actor has been on -> cycle completed!
            sub_VU32(&(Service_mgr.injector1_cycle_count), 1);

            //check if the commanded amount of cycles have been executed
            if(Service_mgr.injector1_cycle_count == 0)
            {
                //reset controls
                Service_mgr.flags.injector1_control= false;
            }
            else
            {
                //store toggle timestamp
                Service_mgr.injector1_toggle= now + Service_mgr.injector1_off_ms;
            }
        }
        else
        {
            //actor has been off

            //command fuel hardware
            set_injector1_powered();

            //store toggle timestamp
            Service_mgr.injector1_toggle= now + Service_mgr.injector1_on_ms;
        }
    }


    //update injector 2
    if((Service_mgr.flags.injector2_control == true) && (now >= Service_mgr.injector2_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.actors.fuel_injector_2 == true)
        {

            set_injector2_unpowered();

            //actor has been on -> cycle completed!
            sub_VU32(&(Service_mgr.injector2_cycle_count), 1);

            //check if the commanded amount of cycles have been executed
            if(Service_mgr.injector2_cycle_count == 0)
            {
                //reset controls
                Service_mgr.flags.injector2_control= false;
            }
            else
            {
                //store toggle timestamp
                Service_mgr.injector2_toggle= now + Service_mgr.injector2_off_ms;
            }
        }
        else
        {
            //actor has been off

            //command hardware
            set_injector2_powered();

            //store toggle timestamp
            Service_mgr.injector2_toggle= now + Service_mgr.injector2_on_ms;
        }
    }


    //update coil 1
    if((Service_mgr.flags.coil1_control == true) && (now >= Service_mgr.coil1_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.actors.ignition_coil_1 == true)
        {
            set_coil1_unpowered();

            //actor has been on -> cycle completed!
            sub_VU32(&(Service_mgr.coil1_cycle_count), 1);

            //check if the commanded amount of cycles have been executed
            if(Service_mgr.coil1_cycle_count == 0)
            {
                //reset controls
                Service_mgr.flags.coil1_control= false;
            }
            else
            {
                //store toggle timestamp
                Service_mgr.coil1_toggle= now + Service_mgr.coil1_off_ms;
            }
        }
        else
        {
            //actor has been off

            //command hardware
            set_coil1_powered();

            //store toggle timestamp
            Service_mgr.coil1_toggle= now + Service_mgr.coil1_on_ms;
        }
    }


    //update coil 2
    if((Service_mgr.flags.coil2_control == true) && (now >= Service_mgr.coil2_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.actors.ignition_coil_2 == true)
        {
            set_coil2_unpowered();

            //actor has been on -> cycle completed!
            sub_VU32(&(Service_mgr.coil2_cycle_count), 1);

            //check if the commanded amount of cycles have been executed
            if(Service_mgr.coil2_cycle_count == 0)
            {
                //reset controls
                Service_mgr.flags.coil2_control= false;
            }
            else
            {
                //store toggle timestamp
                Service_mgr.coil2_toggle= now + Service_mgr.coil2_off_ms;
            }
        }
        else
        {
            //actor has been off

            //command hardware
            set_coil2_powered();

            //store toggle timestamp
            Service_mgr.coil2_toggle= now + Service_mgr.coil2_on_ms;
        }
    }



}












