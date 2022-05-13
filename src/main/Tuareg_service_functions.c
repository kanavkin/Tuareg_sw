#include <Tuareg_platform.h>
#include <Tuareg.h>


#define SERVICE_VERBOSE_OUTPUT

//#define SERVICE_DEBUG_OUTPUT

#ifdef SERVICE_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // SERVICE_DEBUG_OUTPUT


service_mgr_t Service_mgr;



/**********************************************************************************************************************
These service functions assume a linear system time with 1 ms increment and cyclic updates at 1 ms!

API functions

**********************************************************************************************************************/

void request_service_mode()
{
    //enter service only if the engine has been halted and the crank has stopped spinning
    if((Tuareg.flags.standby == false) || (Tuareg.errors.fatal_error == true) || (Tuareg.flags.limited_op))
    {
        return;
    }

    Tuareg.flags.service_mode= true;

    set_mil(MIL_BLINK_SLOW);

    Syslog_Info(TID_SERVICE, SERVICE_LOC_SERVICE_ENABLED);

    #ifdef SERVICE_DEBUG_OUTPUT
    DebugMsg_Warning("Entering Service Mode");
    #endif // SERVICE_DEBUG_OUTPUT
}


void request_service_activation(U32 Actor, U32 On, U32 Off, U32 End)
{
    //precondition check
    if(Tuareg.flags.service_mode == false)
    {
        return;
    }

    //check if deactivation command received
    if((On == 0) && (Off == 0) && (End == 0))
    {
        switch (Actor)
        {

            case SACT_FUEL_PUMP:

                deactivate_fuel_pump();
                break;

            case SACT_COIL_1:

                deactivate_coil1();
                break;

            case SACT_COIL_2:

                deactivate_coil2();
                break;

            case SACT_INJECTOR_1:

                deactivate_injector1();
                break;

            case SACT_INJECTOR_2:

                deactivate_injector2();
                break;


        default:
          break;
      }
    }
    else
    {
        switch (Actor)
        {

            case SACT_FUEL_PUMP:

                activate_fuel_pump(End);
                break;

            case SACT_COIL_1:

                activate_coil1(On, Off, End);
                break;

            case SACT_COIL_2:

                activate_coil2(On, Off, End);
                break;

            case SACT_INJECTOR_1:

                activate_injector1(On, Off, End);
                break;

            case SACT_INJECTOR_2:

                activate_injector2(On, Off, End);
                break;

            default:
                break;
        }
    }
}

void service_functions_periodic_update()
{
    volatile timestamp_t now;

    now= Tuareg.pTimer->system_time;

    //check other restrictions needed?
    if(Tuareg.errors.fatal_error == true)
    {
        return;
    }

    //update fueling system
    fuel_pump_periodic_update(now);
    injector1_periodic_update(now);
    injector2_periodic_update(now);

    //update coils
    coil1_periodic_update(now);
    coil2_periodic_update(now);
}


void init_service_functions()
{

    #ifdef SERVICE_DEBUG_OUTPUT
    Syslog_Warning(TID_SERVICE, SERVICE_LOC_DEBUGOUTPUT_ENABLED);
    #endif // SERVICE_DEBUG_OUTPUT

}



/**********************************************************************************************************************
helper functions - fuel pump
**********************************************************************************************************************/

void activate_fuel_pump(U32 Timeout_s)
{
    //check preconditions
    if(Tuareg.flags.service_mode == false)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("fuel pump service activation not permitted!");
        #endif // SERVICE_DEBUG_OUTPUT

        return;
    }

    if(Timeout_s > SERVICE_FUELPUMP_MAX_TIMEOUT_S)
    {
        Timeout_s= SERVICE_FUELPUMP_MAX_TIMEOUT_S;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_END_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("fuel pump service activation interval clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    //store deactivation timestamp
    Service_mgr.fuel_pump_timeout= Tuareg.pTimer->system_time + 1000 * Timeout_s;

    //take over control
    Service_mgr.flags.fuel_pump_control= true;

    //command fuel hardware
    set_fuel_pump(ACTOR_POWERED);

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_FUEL_PUMP_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void deactivate_fuel_pump()
{
        Service_mgr.flags.fuel_pump_control= false;
        Service_mgr.fuel_pump_timeout= 0;

        //command fuel hardware
        set_fuel_pump(ACTOR_UNPOWERED);

        #ifdef SERVICE_VERBOSE_OUTPUT
        Syslog_Info(TID_SERVICE, SERVICE_LOC_DEACTIVATE_FUEL_PUMP);
        #endif // SERVICE_VERBOSE_OUTPUT
}


void fuel_pump_periodic_update(VU32 now)
{
    //check fuel pump timeout
    if((Service_mgr.flags.fuel_pump_control == true) && (now >= Service_mgr.fuel_pump_timeout))
    {
       deactivate_fuel_pump();
    }
}



/**********************************************************************************************************************
injector 1
**********************************************************************************************************************/

void activate_injector1(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s)
{
    //check preconditions
    if(Tuareg.flags.service_mode == false)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 1 service activation not permitted!");
        #endif // SERVICE_DEBUG_OUTPUT

        return;
    }

/**
with 8 bit protocol on, off max := 255 !!!

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_ON_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 1 service activation on time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(Off_time_ms > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_OFF_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 1 service activation off time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(On_target_s > SERVICE_INJECTOR_MAX_ONTIME)
    {
        On_target_s= SERVICE_INJECTOR_MAX_ONTIME;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_END_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 1 service activation on target clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }
*/

    if((On_time_ms == 0xFF) && (Off_time_ms == 0xFF))
    {
        /**
        static mode
        */
        Service_mgr.injector1_on_ms= 1000 * On_target_s;
        Service_mgr.injector1_off_ms= 0;
        Service_mgr.injector1_on_remain_ms= 0;

    }
    else
    {
        //rect mode
        Service_mgr.injector1_on_ms= On_time_ms;
        Service_mgr.injector1_off_ms= Off_time_ms;
        Service_mgr.injector1_on_remain_ms= 1000 * On_target_s;
    }


    //store toggle timestamp
    Service_mgr.injector1_toggle= Tuareg.pTimer->system_time + Service_mgr.injector1_on_ms;

    //take over control
    Service_mgr.flags.injector1_control= true;

    //command fuel hardware
    set_injector1(ACTOR_POWERED);

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR1_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void deactivate_injector1()
{
    set_injector1(ACTOR_UNPOWERED);
    Service_mgr.injector1_on_remain_ms= 0;
    Service_mgr.flags.injector1_control= false;

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_DEACTIVATE_INJECTOR1);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void injector1_periodic_update(VU32 now)
{
    //update injector 1
    if((Service_mgr.flags.injector1_control == true) && (now >= Service_mgr.injector1_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.flags.fuel_injector_1 == true)
        {
            set_injector1(ACTOR_UNPOWERED);

            //actor has been on -> less remaining on time
            sub_VU32(&(Service_mgr.injector1_on_remain_ms), Service_mgr.injector1_on_ms);

            //check if the commanded on time has been reached
            if(Service_mgr.injector1_on_remain_ms == 0)
            {
                deactivate_injector1();
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
            set_injector1(ACTOR_POWERED);

            //store toggle timestamp
            Service_mgr.injector1_toggle= now + Service_mgr.injector1_on_ms;
        }
    }
}




/**********************************************************************************************************************
injector 2
**********************************************************************************************************************/

void activate_injector2(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s)
{
    //check preconditions
    if(Tuareg.flags.service_mode == false)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 2 service activation not permitted!");
        #endif // SERVICE_DEBUG_OUTPUT

        return;
    }
/**
    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 2 service activation on time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(Off_time_ms > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 2 service activation off time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }
    if(On_target_s > SERVICE_INJECTOR_MAX_ONTIME)
    {
        On_target_s= SERVICE_INJECTOR_MAX_ONTIME;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_END_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("injector 2 service activation on target clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }
*/

    if((On_time_ms == 0xFF) && (Off_time_ms == 0xFF))
    {
        /**
        static mode
        */
        Service_mgr.injector2_on_ms= 1000 * On_target_s;
        Service_mgr.injector2_off_ms= 0;
        Service_mgr.injector2_on_remain_ms= 0;

    }
    else
    {
        //rect mode
        Service_mgr.injector2_on_ms= On_time_ms;
        Service_mgr.injector2_off_ms= Off_time_ms;
        Service_mgr.injector2_on_remain_ms= 1000 * On_target_s;
    }



    //store toggle timestamp
    Service_mgr.injector2_toggle= Tuareg.pTimer->system_time + Service_mgr.injector2_on_ms;

    //take over control
    Service_mgr.flags.injector2_control= true;

    //command fuel hardware
    set_injector2(ACTOR_POWERED);

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_INJECTOR2_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void deactivate_injector2()
{
    set_injector2(ACTOR_UNPOWERED);
    Service_mgr.injector2_on_remain_ms= 0;
    Service_mgr.flags.injector2_control= false;

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_DEACTIVATE_INJECTOR2);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void injector2_periodic_update(VU32 now)
{
    if((Service_mgr.flags.injector2_control == true) && (now >= Service_mgr.injector2_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.flags.fuel_injector_2 == true)
        {
            set_injector2(ACTOR_UNPOWERED);

            //actor has been on -> less remaining on time
            sub_VU32(&(Service_mgr.injector2_on_remain_ms), Service_mgr.injector2_on_ms);

            //check if the commanded on time has been reached
            if(Service_mgr.injector2_on_remain_ms == 0)
            {
                deactivate_injector2();
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
            set_injector2(ACTOR_POWERED);

            //store toggle timestamp
            Service_mgr.injector2_toggle= now + Service_mgr.injector2_on_ms;
        }
    }
}



/**********************************************************************************************************************
coil 1
**********************************************************************************************************************/

void activate_coil1(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s)
{
    //check preconditions
    if(Tuareg.flags.service_mode == false)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 1 service activation not permitted!");
        #endif // SERVICE_DEBUG_OUTPUT

        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_ON_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 1 service activation on time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(Off_time_ms > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_OFF_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 1 service activation off time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(On_target_s > SERVICE_COIL_MAX_ONTIME)
    {
        On_target_s= SERVICE_COIL_MAX_ONTIME;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_END_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 1 service activation on target clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    Service_mgr.coil1_on_ms= On_time_ms;
    Service_mgr.coil1_off_ms= Off_time_ms;
    Service_mgr.coil1_on_remain_ms= 1000 * On_target_s;

    //store toggle timestamp
    Service_mgr.coil1_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.coil1_control= true;

    //command ignition hardware
    set_ignition_ch1(ACTOR_POWERED);

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL1_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void deactivate_coil1()
{
    set_ignition_ch1(ACTOR_UNPOWERED);
    Service_mgr.coil1_on_remain_ms= 0;
    Service_mgr.flags.coil1_control= false;

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_DEACTIVATE_COIL1);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void coil1_periodic_update(VU32 now)
{
    if((Service_mgr.flags.coil1_control == true) && (now >= Service_mgr.coil1_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.flags.ignition_coil_1 == true)
        {
            set_ignition_ch1(ACTOR_UNPOWERED);

            //actor has been on -> less remaining on time
            sub_VU32(&(Service_mgr.coil1_on_remain_ms), Service_mgr.coil1_on_ms);

            //check if the commanded on time has been reached
            if(Service_mgr.coil1_on_remain_ms == 0)
            {
                deactivate_coil1();
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
            set_ignition_ch1(ACTOR_POWERED);

            //store toggle timestamp
            Service_mgr.coil1_toggle= now + Service_mgr.coil1_on_ms;
        }
    }
}


/**********************************************************************************************************************
coil 2
**********************************************************************************************************************/

void activate_coil2(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s)
{
    //check preconditions
    if(Tuareg.flags.service_mode == false)
    {
        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_PERMISSION);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 2 service activation not permitted!");
        #endif // SERVICE_DEBUG_OUTPUT

        return;
    }

    if(On_time_ms > SERVICE_ACTOR_MAX_ON_MS)
    {
        On_time_ms= SERVICE_ACTOR_MAX_ON_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_ON_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 2 service activation on time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(Off_time_ms > SERVICE_ACTOR_MAX_OFF_MS)
    {
        Off_time_ms= SERVICE_ACTOR_MAX_OFF_MS;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_OFF_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 2 service activation off time clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    if(On_target_s > SERVICE_COIL_MAX_ONTIME)
    {
        On_target_s= SERVICE_COIL_MAX_ONTIME;

        Syslog_Warning(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_END_CLIP);

        #ifdef SERVICE_DEBUG_OUTPUT
        DebugMsg_Warning("coil 2 service activation on target clipped!");
        #endif // SERVICE_DEBUG_OUTPUT
    }

    Service_mgr.coil2_on_ms= On_time_ms;
    Service_mgr.coil2_off_ms= Off_time_ms;
    Service_mgr.coil2_on_remain_ms= 1000 * On_target_s;

    //store toggle timestamp
    Service_mgr.coil2_toggle= Tuareg.pTimer->system_time + On_time_ms;

    //take over control
    Service_mgr.flags.coil2_control= true;

    //command hardware
    set_ignition_ch2(ACTOR_POWERED);

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_ACTIVATE_COIL2_BEGIN);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void deactivate_coil2()
{
    set_ignition_ch2(ACTOR_UNPOWERED);
    Service_mgr.coil2_on_remain_ms= 0;
    Service_mgr.flags.coil2_control= false;

    #ifdef SERVICE_VERBOSE_OUTPUT
    Syslog_Info(TID_SERVICE, SERVICE_LOC_DEACTIVATE_COIL2);
    #endif // SERVICE_VERBOSE_OUTPUT
}


void coil2_periodic_update(VU32 now)
{
    if((Service_mgr.flags.coil2_control == true) && (now >= Service_mgr.coil2_toggle))
    {
        //check if the actor has been powered
        if(Tuareg.flags.ignition_coil_2 == true)
        {
            set_ignition_ch2(ACTOR_UNPOWERED);

            //actor has been on -> less remaining on time
            sub_VU32(&(Service_mgr.coil2_on_remain_ms), Service_mgr.coil2_on_ms);

            //check if the commanded on time has been reached
            if(Service_mgr.coil2_on_remain_ms == 0)
            {
                deactivate_coil2();
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
            set_ignition_ch2(ACTOR_POWERED);

            //store toggle timestamp
            Service_mgr.coil2_toggle= now + Service_mgr.coil2_on_ms;
        }
    }
}
















