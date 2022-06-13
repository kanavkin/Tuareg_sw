#include <Tuareg_platform.h>
#include <Tuareg.h>

//#define TUAREG_DEBUG_OUTPUT

#ifdef TUAREG_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // TUAREG_DEBUG_OUTPUT


//#define HIL_HW


#ifdef HIL_HW
#warning built for HIL tester
#warning do not install on ECU hw
#endif // TUAREG_DEBUG_OUTPUT



const char Tuareg_Version [] __attribute__((__section__(".rodata"))) = "Tuareg V0.23.4 2022.06";


/******************************************************************************************************************************
INIT
******************************************************************************************************************************/

void Tuareg_Init()
{

    /******************************************************
    system init
    ******************************************************/

    //first action: set init state
    Tuareg.errors.init_not_completed= true;

    //engine operation not permitted until end of initialization
    Tuareg.flags.run_inhibit= true;
    Tuareg.flags.cranking= false;
    Tuareg.flags.standby= true;
    Tuareg.decoder_watchdog= 0xFFFFFFFF;

    //use 16 preemption priority levels
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_ENTER_INIT);



    /******************************************************
    initialize core components
    ******************************************************/

    //debug port
    UART_DEBUG_PORT_Init();

    //initialize systick timer to provide system timestamp
    Tuareg.pTimer= init_systick_timer();

    //logs
    Tuareg.pSyslog= Syslog_init();
    Tuareg.pHighspeedlog= highspeedlog_init();
    init_Fault_Log();

    //load main config
    Tuareg_load_config();

    /**
    vital modules
    */
    init_Sensors();
    init_Ignition();
    init_Fueling();
    Tuareg.pDecoder= init_Decoder();

    //provide initial process data
    Tuareg_update_process_data();

    /**
    hmi
    */
    Tuareg_init_console();

    #ifndef LOWPRIOSCHEDULER_WIP
    init_Lowprio_Scheduler();
    #endif // LOWPRIOSCHEDULER_WIP

    init_dash();

    #ifdef TUAREG_DEBUG_OUTPUT
    Tuareg_print_init_message();
    #endif // TUAREG_DEBUG_OUTPUT

    //begin fuel pump priming
    if(Tuareg.errors.fatal_error == false)
    {
        if(Tuareg_Setup.fuel_pump_priming_duration > 0)
        {
            Tuareg.fuel_pump_priming_remain_s= Tuareg_Setup.fuel_pump_priming_duration;
            Tuareg.flags.fuel_pump_priming= true;
            Syslog_Info(TID_TUAREG_FUELING, TUAREG_LOC_BEGIN_FUEL_PUMP_PRIMING);
        }
    }

    //init_act_hw();
    //init_act_logic();

    //DEBUG
    //init_debug_pins();
    //set_debug_pin(PIN_ON);
    //dwt_init();


    //last init action
    Tuareg.errors.init_not_completed= false;
}


void Tuareg_load_config()
{
    exec_result_t result;

    //bring up eeprom
    Eeprom_init();

    //loading the config data is essential, failure forces "limp home mode"
    result= load_Tuareg_Setup();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load config
        Tuareg.errors.tuareg_config_error= true;
        Tuareg.flags.limited_op= true;
        load_essential_Tuareg_Setup();

        Syslog_Error(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_FAIL);
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_ESSENTIALS_CONFIG_LOADED);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Tuareg config");
        DebugMsg_Warning("Tuareg essential config has been loaded");
        #endif // TUAREG_DEBUG_OUTPUT
    }
    else if(Tuareg_Setup.Version != TUAREG_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong config version
        Tuareg.errors.tuareg_config_error= true;
        Tuareg.flags.limited_op= true;
        load_essential_Tuareg_Setup();

        Syslog_Error(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_VERSION_FAIL);
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_ESSENTIALS_CONFIG_LOADED);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Error("Tuareg config loaded version does not match");
        DebugMsg_Warning("Tuareg essential config has been loaded");
        #endif // TUAREG_DEBUG_OUTPUT
    }
    else
    {
        //loaded config with correct version
        Tuareg.errors.tuareg_config_error= false;

        Syslog_Info(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_SUCCESS);
    }
}


void Tuareg_print_init_message()
{
    print(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
    print(DEBUG_PORT, "V 0.7");
}



/******************************************************************************************************************************
Update - periodic update function

called every 10 ms from systick timer (100 Hz)

this function will be executed in all system states and has to take care of all system errors


******************************************************************************************************************************/

const U32 cInj_wd_thres_ms= 100;

void Tuareg_update()
{
    //injector watchdog check
    if(((Tuareg.injector1_watchdog_ms > cInj_wd_thres_ms) || (Tuareg.injector2_watchdog_ms > cInj_wd_thres_ms)) && (Tuareg.flags.service_mode == false))
    {
        Fatal(TID_MAIN, TUAREG_LOC_INJ_WATCHDOG);
    }


    //update control flags
    Tuareg_update_run_inhibit();
    Tuareg_update_limited_op();
    Tuareg_update_rev_limiter();
    Tuareg_update_standby();

    //fuel pump
    Tuareg_update_fuel_pump_control();

    /**
    vital actors control
    (fuel pump is separated to allow fuel priming)

    while the system is in normal operating conditions the vital actors shall be deactivated in standby mode or when run inhibit is set
    in service mode vital actor control is given to the service functions
    fuel pump priming normally takes place when the run switch is off
    */
    if((Tuareg.flags.service_mode == false) && ((Tuareg.flags.standby == true) || (Tuareg.flags.run_inhibit == true)))
    {
        //keep vital actors deactivated, allow fuel pump priming
        Tuareg_deactivate_vital_actors(true);
    }

}


/******************************************************************************************************************************
periodic update helper function - run inhibit
******************************************************************************************************************************/

/**
static const bool cCrashSensorTrigger_on_high= true;
static const bool cRunSensorTrigger_on_high= true;
static const bool cSidestandSensorTrigger_on_high= true;
static const bool cHaltOnSidestand= true;

U16 overheat_thres_K= 390;
*/
const U32 cOverheat_hist_K= 20;


void Tuareg_update_run_inhibit()
{
    //check precondition - no fatal error present
    if(Tuareg.errors.fatal_error == true)
    {
        Tuareg.flags.run_inhibit= true;
        return;
    }

    /**
    check if the RUN switch, SIDESTAND sensor or CRASH sensor or the OVERHEAT protector indicate a HALT condition
    */

    //shut engine off if the RUN switch is DISENGAGED
    Tuareg.flags.run_switch_deactivated= (Digital_Sensors.run == false) ? true : false;

    //shut engine off if the CRASH sensor is engaged
    Tuareg.flags.crash_sensor_triggered= (Digital_Sensors.crash == Tuareg_Setup.flags.CrashSensor_trig_high) ? true : false;

    //shut engine off if the SIDESTAND sensor is engaged AND a gear has been selected
    Tuareg.flags.sidestand_sensor_triggered= ((Digital_Sensors.sidestand == Tuareg_Setup.flags.SidestandSensor_trig_high) && (Tuareg.process.Gear != GEAR_NEUTRAL) && (Tuareg.errors.sensor_GEAR_error == false)) ? true : false;


    /**
    check if the overheating protector indicates a HALT condition
    a failed coolant sensor will never trigger this protector
    */
    if((Tuareg.process.CLT_K > Tuareg_Setup.overheat_thres_K) && (Tuareg.errors.sensor_CLT_error == false))
    {
        Tuareg.flags.overheat_detected= true;
    }
    else if( ((Tuareg.flags.overheat_detected == true) && (Tuareg.process.CLT_K < subtract_U32(Tuareg_Setup.overheat_thres_K, cOverheat_hist_K))) ||
             (Tuareg.errors.sensor_CLT_error == true) )
    {
        Tuareg.flags.overheat_detected= false;
    }


    /**
    the run_inhibit flag indicates that engine operation is temporarily restricted
    */
    Tuareg.flags.run_inhibit=(  ((Tuareg.flags.run_switch_deactivated == true) && (Tuareg_Setup.flags.RunSwitch_override == false)) ||
                                ((Tuareg.flags.crash_sensor_triggered == true) && (Tuareg_Setup.flags.CrashSensor_override == false)) ||
                                ((Tuareg.flags.sidestand_sensor_triggered == true) && (Tuareg_Setup.flags.Sidestand_override == false)) ||
                                (Tuareg.flags.overheat_detected == true) ||
                                (Tuareg.flags.service_mode == true) );
}


/******************************************************************************************************************************
periodic update helper function - limited operation strategy

the limited_op flag indicates that only essential functionality shall be executed
once triggered limited_op will remain set until reboot
******************************************************************************************************************************/

const U32 cLoad_error_limp_thres= 100;

void Tuareg_update_limited_op()
{
    //nothing to do if already in limp mode
    //limited_op can be cleared only only by reset
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fatal_error == true))
    {
        return;
    }


    /**
    limit_op is now set together with the error flags for

    -tuareg_config_error
    -decoder_config_error
    -ignition_config_error
    -fueling_config_error
    -sensor_calibration_error
    */



    /*
    At least one method to determine engine load is required to operate the engine

    -> while booting all errors will be present
    -> non-running modes are not affected
    -> while cranking a static ignition profile and fueling is used
    */
    if((Tuareg.engine_runtime > cLoad_error_limp_thres) && (Tuareg.errors.sensor_MAP_error == true) && (Tuareg.errors.sensor_TPS_error == true))
    {
        //LIMP
        Limp(TID_TUAREG, TUAREG_LOC_LIMP_TPSMAP_ERROR);
    }
}


/******************************************************************************************************************************
periodic update helper function - rev limiter

the rev_limiter flag indicates that engine power output shall be decrease to reduce engine rpm
******************************************************************************************************************************/

const U32 cRevlimiter_hist_rpm= 200;

void Tuareg_update_rev_limiter()
{
    U32 max_rpm;

    //check which rpm limit applies
    max_rpm= (Tuareg.flags.limited_op)? Tuareg_Setup.limp_max_rpm : Tuareg_Setup.max_rpm;


    if((Tuareg.pDecoder->flags.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm > max_rpm))
    {
        Tuareg.flags.rev_limiter= true;
    }
    else if((Tuareg.pDecoder->flags.rpm_valid == false) || ((Tuareg.flags.rev_limiter == true) && (Tuareg.pDecoder->crank_rpm < subtract_U32(max_rpm, cRevlimiter_hist_rpm))))
    {
        Tuareg.flags.rev_limiter= false;
    }
}



/******************************************************************************************************************************
periodic update helper function - decoder watchdog and run time parameters

this function shall be called every 100 ms
******************************************************************************************************************************/

const U32 cMaxCrankingEntry= 20;

void Tuareg_update_standby()
{
    /**
    standby flag
    will be set after decoder watchdog timeout and reset with the first decoder position update
    */
    Tuareg.flags.standby= ((Tuareg.decoder_watchdog > Tuareg_Setup.standby_timeout_s) && (Tuareg.flags.run_inhibit == false))? true : false;

    /**
    cranking flag
    will be set only when the engine has not been running and the crank is moving slowly
    */
    if( (Tuareg.flags.run_inhibit == false) &&
        (Tuareg.engine_runtime < cMaxCrankingEntry) &&
        (Tuareg.flags.standby == false) &&
        (Tuareg.pDecoder->flags.standstill == false) &&
        //(Tuareg.pDecoder->outputs.rpm_valid == true)  &&
        (Tuareg.pDecoder->crank_rpm < Tuareg_Setup.cranking_end_rpm))
    {
        Tuareg.flags.cranking= true;
    }

    if( ((Tuareg.pDecoder->flags.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm > Tuareg_Setup.cranking_end_rpm)) ||
        (Tuareg.flags.run_inhibit == true) ||
        (Tuareg.flags.standby == true) ||
        (Tuareg.pDecoder->flags.standstill == true))
    {
        Tuareg.flags.cranking= false;
    }


    /**
    the engine run time counter begins to count when leaving crank mode
    */
    if( (Tuareg.flags.run_inhibit == false) &&
        (Tuareg.flags.standby == false) &&
        (Tuareg.pDecoder->flags.standstill == false) &&
        (Tuareg.flags.cranking == false) &&
        (Tuareg.pDecoder->flags.rpm_valid == true))
    {
        //engine is running -> increment runtime counter
        if(Tuareg.engine_runtime < cU32max)
        {
            Tuareg.engine_runtime += 1;
        }
    }
    else
    {
        Tuareg.engine_runtime= 0;
    }
}


/******************************************************************************************************************************
periodic update helper function - fuel pump control

the fuel_pump_priming flag indicates that the fuel pump shall be activated for a short period of time to maintain normal fuel
pressure
******************************************************************************************************************************/

void Tuareg_update_fuel_pump_control()
{
    //check precondition -no fatal error present
    if(Tuareg.errors.fatal_error == true)
    {
        set_fuel_pump(ACTOR_UNPOWERED);
        return;
    }

    /**
    fuel pump priming control
    */
    if((Tuareg.flags.fuel_pump_priming == true) && (Tuareg.fuel_pump_priming_remain_s == 0))
    {
        Tuareg.flags.fuel_pump_priming= false;
        Syslog_Info(TID_TUAREG_FUELING, TUAREG_LOC_END_FUEL_PUMP_PRIMING);
    }

    /**
    fuel pump control
    */

    //in service mode fuel pump control is given to the service functions
    if(Tuareg.flags.service_mode == true)
    {
        return;
    }

    //under normal operating conditions the fuel pump shall be deactivated in standby mode or when run inhibit is set and no priming is commanded
    if( ((Tuareg.flags.standby == false) && (Tuareg.flags.run_inhibit == false)) || (Tuareg.flags.fuel_pump_priming == true) )
    {
        //fuel pump shall be active
        set_fuel_pump(ACTOR_POWERED);
    }
    else
    {
        //fuel pump shall be deactivated
        set_fuel_pump(ACTOR_UNPOWERED);
    }
}


/******************************************************************************************************************************
keep vital actors deactivated

if a fatal error is present, force all actors off
in normal mode, ignoring the fuel pump is allowed
******************************************************************************************************************************/
void Tuareg_deactivate_vital_actors(bool IgnoreFuelPump)
{
    //turn off ignition system
    set_ignition_ch1(ACTOR_UNPOWERED);
    set_ignition_ch2(ACTOR_UNPOWERED);

    //turn off fueling system
    set_injector1(ACTOR_UNPOWERED);
    set_injector2(ACTOR_UNPOWERED);

    //reset scheduler
    scheduler_reset_channel(SCHEDULER_CH_IGN1);
    scheduler_reset_channel(SCHEDULER_CH_IGN2);
    scheduler_reset_channel(SCHEDULER_CH_FUEL1);
    scheduler_reset_channel(SCHEDULER_CH_FUEL2);


    //the fuel pump is normally allowed to finish its priming stage
    if((IgnoreFuelPump == false) || (Tuareg.errors.fatal_error == true))
    {
        set_fuel_pump(ACTOR_UNPOWERED);
    }

}


/******************************************************************************************************************************
periodic helper function - integrate the trip based on the estimated ground speed
called every 100 ms from systick timer (10 Hz)
******************************************************************************************************************************/
void Tuareg_update_trip()
{
    U32 trip_increment_mm;

    //s := v * t
    trip_increment_mm= Tuareg.process.ground_speed_mmps / 10;

    Tuareg.trip_mm += trip_increment_mm;

}



/******************************************************************************************************************************
periodic helper function - output update interval: 1s
called every second from systick timer
******************************************************************************************************************************/
void Tuareg_update_consumption_data()
{
        //export data
        Tuareg.fuel_consumpt_1s_ug= Tuareg.injected_mass_ug;
        Tuareg.trip_1s_mm= Tuareg.trip_mm;

        //reset counters
        Tuareg.injected_mass_ug= 0;
        Tuareg.trip_mm= 0;
}
