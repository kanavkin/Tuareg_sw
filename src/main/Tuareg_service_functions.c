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
#include "service_syslog_locations.h"

#include "Tuareg_service_functions.h"

#define SERVICE_VERBOSE_OUTPUT


service_mgr_t Service_mgr;



/**********************************************************************************************************************
These service functions assume a linear system time with 1 ms increment and cyclic updates at 1 ms!
**********************************************************************************************************************/

void service_functions_periodic_update()
{
    timestamp_t now;

    now= Tuareg.pTimer->system_time;

    //check other restrictions needed?


    //update fuelling system
    fuel_pump_periodic_update(now);
    injector1_periodic_update(now);
    injector1_periodic_update(now);

    //update coils
    coil1_periodic_update(now);
    coil2_periodic_update(now);
}


void init_service_functions()
{


}


/**********************************************************************************************************************
fuel pump
**********************************************************************************************************************/

void activate_fuel_pump(U32 Timeout_s)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_PERMISSION);
        DebugMsg_Warning("fuel pump service activation not permitted!");
        return;
    }

    if(Timeout_s == 0)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_END_LOW);
        DebugMsg_Warning("fuel pump service activation parameter invalid");
        return;
    }

    if(Timeout_s > SERVICE_FUELPUMP_MAX_TIMEOUT_S)
    {
        Timeout_s= SERVICE_FUELPUMP_MAX_TIMEOUT_S;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_END_CLIP);
        DebugMsg_Warning("fuel pump service activation interval clipped!");
    }

    //store deactivation timestamp
    Service_mgr.fuel_pump_timeout= Tuareg.pTimer->system_time + 1000 * Timeout_s;

    //take over control
    Service_mgr.flags.fuel_pump_control= true;

    //command fuel hardware
    set_fuel_pump_powered();

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void fuel_pump_periodic_update(U32 now)
{
    //check fuel pump timeout
    if((Service_mgr.flags.fuel_pump_control == true) && (now >= Service_mgr.fuel_pump_timeout))
    {
        Service_mgr.flags.fuel_pump_control= false;
        Service_mgr.fuel_pump_timeout= 0;

        //command fuel hardware
        set_fuel_pump_unpowered();

        #ifdef SERVICE_VERBOSE_OUTPUT
        Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_END);
        #endif // SERVICE_VERBOSE_OUTPUT
    }
}



/**********************************************************************************************************************
injector 1
**********************************************************************************************************************/

void activate_injector1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_PERMISSION);
        DebugMsg_Warning("injector 1 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_PARAMETER_INVALID);
        DebugMsg_Warning("injector 1 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_ON_CLIP);
        DebugMsg_Warning("injector 1 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_OFF_CLIP);
        DebugMsg_Warning("injector 1 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_END_CLIP);
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

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void injector1_periodic_update(U32 now)
{
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

                #ifdef SERVICE_VERBOSE_OUTPUT
                Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_END);
                #endif // SERVICE_VERBOSE_OUTPUT
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
}




/**********************************************************************************************************************
injector 2
**********************************************************************************************************************/

void activate_injector2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);
        DebugMsg_Warning("injector 2 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);
        DebugMsg_Warning("injector 2 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);
        DebugMsg_Warning("injector 2 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);
        DebugMsg_Warning("injector 2 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);
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

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void injector2_periodic_update(U32 now)
{
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

                #ifdef SERVICE_VERBOSE_OUTPUT
                Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_END);
                #endif // SERVICE_VERBOSE_OUTPUT
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
}

/**********************************************************************************************************************
coil 1
**********************************************************************************************************************/

void activate_coil1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_PERMISSION);
        DebugMsg_Warning("coil 1 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_PERMISSION);
        DebugMsg_Warning("coil 1 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_ON_CLIP);
        DebugMsg_Warning("coil 1 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_OFF_CLIP);
        DebugMsg_Warning("coil 1 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_END_CLIP);
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

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void coil1_periodic_update(U32 now)
{
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

                #ifdef SERVICE_VERBOSE_OUTPUT
                Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_END);
                #endif // SERVICE_VERBOSE_OUTPUT
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
}


/**********************************************************************************************************************
coil 2
**********************************************************************************************************************/

void activate_coil2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles)
{
    //check preconditions
    if((Tuareg.actors.fueling_inhibit == true) || (Tuareg.Runmode != TMODE_SERVICE))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_PERMISSION);
        DebugMsg_Warning("coil 2 service activation not permitted!");
        return;
    }

    if((Cycles == 0) || (On_time_ms == 0) || (Off_time_ms == 0))
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_PARAMETER_INVALID);
        DebugMsg_Warning("coil 2 service activation parameters invalid");
        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_ON_CLIP);
        DebugMsg_Warning("coil 2 service activation on time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_OFF_CLIP);
        DebugMsg_Warning("coil 2 service activation off time clipped!");
    }

    if(Cycles > SERVICE_ACTOR_MAX_CYCLES)
    {
        Cycles= SERVICE_ACTOR_MAX_CYCLES;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_END_CLIP);
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

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void coil2_periodic_update(U32 now)
{
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

                #ifdef SERVICE_VERBOSE_OUTPUT
                Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_END);
                #endif // SERVICE_VERBOSE_OUTPUT
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
















