#include <Tuareg_platform.h>
#include <Tuareg.h>

//#define TUAREG_DEBUG_OUTPUT

#ifdef TUAREG_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // TUAREG_DEBUG_OUTPUT




#ifdef DEBUG_TARGET
#warning developer build
#warning not to be installed on a production system!
#endif



const char Tuareg_Version [] __attribute__((__section__(".rodata"))) = "Tuareg V0.26 2024.02";



/**
crank position when the process data and controls shall be updated
*/
const crank_position_t cTuareg_controls_update_pos= CRK_POSITION_B2;



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
    Tuareg.flags.run_allow= false;
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
    load_Control_Sets();

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
    print(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
    print_flash(DEBUG_PORT, Tuareg_Version);
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

    //load Tuareg main config data
    result= load_Tuareg_Config();

    //check if config has been loaded
    if((result != EXEC_OK) || (Tuareg_Setup.Version != TUAREG_REQUIRED_CONFIG_VERSION))
    {
        //failed to load config
        Tuareg.errors.tuareg_config_error= true;

        //no engine operation possible
        Fatal(TID_TUAREG, TUAREG_LOC_CONFIG_ERROR);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Tuareg config");
        #endif // TUAREG_DEBUG_OUTPUT
    }
    else
    {
        //loaded config with correct version
        Tuareg.errors.tuareg_config_error= false;

        Syslog_Info(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_SUCCESS);
    }
}


/******************************************************************************************************************************
periodic update helper function - run allow
******************************************************************************************************************************/

const U32 cOverheat_hist_K= 20;


void Tuareg_update_run_allow()
{
    //check precondition - no fatal error present
    if(Tuareg.errors.fatal_error == true)
    {
        Tuareg.flags.run_allow= false;
        return;
    }

    /**
    check if the RUN switch, SIDESTAND sensor or CRASH sensor or the OVERHEAT protector indicate a HALT condition
    */

    //shut engine off if the RUN switch is DISENGAGED
    Tuareg.flags.run_switch_deactivated= (Digital_Sensors.run == false);

    //shut engine off if the CRASH sensor is engaged
    Tuareg.flags.crash_sensor_triggered= (Digital_Sensors.crash == Tuareg_Setup.flags.CrashSensor_trig_high);

    //shut engine off if the SIDESTAND sensor is engaged AND a gear has been selected AND the indicated gear is valid
    Tuareg.flags.sidestand_sensor_triggered= ((Digital_Sensors.sidestand == Tuareg_Setup.flags.SidestandSensor_trig_high) && (Tuareg.process.Gear != GEAR_NEUTRAL) && (Tuareg.errors.sensor_GEAR_error == false));


    /**
    check if the overheating protector indicates a HALT condition
    */
    if(Tuareg.process.CLT_K > Tuareg_Setup.overheat_thres_K)
    {
        Tuareg.flags.overheat_detected= true;
    }
    else if( (Tuareg.flags.overheat_detected == true) && (Tuareg.process.CLT_K < subtract_U32(Tuareg_Setup.overheat_thres_K, cOverheat_hist_K)) )
    {
        Tuareg.flags.overheat_detected= false;
    }

    /**
    At least one method to determine engine load is required to operate the engine
    -> MAP/TPS sensor errors
    */


    /******************************************************************************************************************
    *** the run inhibit flag indicates that engine operation is temporarily restricted                              ***
    ******************************************************************************************************************/
    Tuareg.flags.run_allow=(

        ((Tuareg.flags.run_switch_deactivated == false) || (Tuareg_Setup.flags.RunSwitch_override == true)) &&
        ((Tuareg.flags.crash_sensor_triggered == false) || (Tuareg_Setup.flags.CrashSensor_override == true)) &&
        ((Tuareg.flags.sidestand_sensor_triggered == false) || (Tuareg_Setup.flags.Sidestand_override == true)) &&
        (Tuareg.flags.overheat_detected == false) &&
        (Tuareg.flags.rev_limiter == false) &&
        (Tuareg.flags.service_mode == false) &&
        ((Tuareg.errors.sensor_MAP_error == false) || (Tuareg.errors.sensor_TPS_error == false))

    );
}


/******************************************************************************************************************************
periodic update helper function - limited operation strategy

the limited_op flag indicates that only essential functionality shall be executed
limited_op can be cleared only by reset
******************************************************************************************************************************/

void Tuareg_update_limited_op()
{
    //nothing to do if already in limp mode
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fatal_error == true))
    {
        return;
    }

    /**
    TPS, MAP, IAT, CLT, VBAT sensors are required for safe engine operation
    */
    if( (Tuareg.flags.run_allow == true) && (
        (Tuareg.errors.sensor_TPS_error == true) ||
        (Tuareg.errors.sensor_MAP_error == true) ||
        (Tuareg.errors.sensor_IAT_error == true) ||
        (Tuareg.errors.sensor_CLT_error == true) ||
        (Tuareg.errors.sensor_VBAT_error == true) ))
    {
        //limit engine speed
        Limp(TID_FUELING_CONTROLS, TUAREG_LOC_LIMP_SENSOR_ERROR);
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
const U32 cCranking_End_rpm= 700;

void Tuareg_update_standby()
{
    /**
    standby flag
    will be set after decoder watchdog timeout and reset with the first decoder position update
    */
    Tuareg.flags.standby= ((Tuareg.decoder_watchdog > Tuareg_Setup.standby_timeout_s) && (Tuareg.flags.run_allow == true));
}


void Tuareg_update_cranking()
{
    /**
    cranking flag
    will be set only when the engine has not been running and the crank is moving slowly
    */
    if( (Tuareg.flags.run_allow == true) &&
        (Tuareg.process.engine_runtime < cMaxCrankingEntry) &&
        (Tuareg.flags.standby == false) &&
        (Tuareg.pDecoder->flags.standstill == false) &&
        (Tuareg.pDecoder->crank_rpm < cCranking_End_rpm))
    {
        Tuareg.flags.cranking= true;
    }

    if( ((Tuareg.pDecoder->flags.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm > cCranking_End_rpm)) ||
        (Tuareg.flags.run_allow == false) ||
        (Tuareg.flags.standby == true) ||
        (Tuareg.pDecoder->flags.standstill == true))
    {
        Tuareg.flags.cranking= false;
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
    if( ((Tuareg.flags.standby == false) && (Tuareg.flags.run_allow == true)) || (Tuareg.flags.fuel_pump_priming == true) )
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
Update - periodic update function

called every 10 ms from systick timer (100 Hz)

this function will be executed in all system states and has to take care of all system errors


******************************************************************************************************************************/

const U32 cInj_wd_thres_ms= 200;

void Tuareg_update_systick()
{
    //injector watchdog check
    if(((Tuareg.injector1_watchdog_ms > cInj_wd_thres_ms) || (Tuareg.injector2_watchdog_ms > cInj_wd_thres_ms)) && (Tuareg.flags.service_mode == false))
    {
        Fatal(TID_MAIN, TUAREG_LOC_INJ_WATCHDOG);
    }

    //update control flags
    Tuareg_update_limited_op();
    Tuareg_update_rev_limiter();
    Tuareg_update_run_allow();
    Tuareg_update_standby();
    Tuareg_update_cranking();
    Tuareg_update_runtime();

    //fuel pump
    Tuareg_update_fuel_pump_control();

    /**
    vital actors control
    (fuel pump is separated to allow fuel priming)

    while the system is in normal operating conditions the vital actors shall be deactivated in standby mode or when run allow is not set
    in service mode vital actor control is given to the service functions
    fuel pump priming normally takes place when the run switch is off
    */
    if((Tuareg.flags.service_mode == false) && ((Tuareg.flags.standby == true) || (Tuareg.flags.run_allow == false)))
    {
        //keep vital actors deactivated, allow fuel pump priming
        Tuareg_deactivate_vital_actors(true);
    }

}


