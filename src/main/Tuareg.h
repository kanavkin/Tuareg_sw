#ifndef TUAREG_H_INCLUDED
#define TUAREG_H_INCLUDED

#include <Tuareg_platform.h>

#include "Tuareg_ID.h"
#include "Tuareg_config.h"
#include "Tuareg_diag.h"
#include "Tuareg_errors.h"
#include "Tuareg_process_data.h"
#include "Tuareg_syslog_locations.h"

#include "Tuareg_decoder.h"
#include "Tuareg_ignition.h"
#include "Tuareg_fueling.h"
#include "Tuareg_sensors.h"
#include "Tuareg_console.h"
#include "Tuareg_dash.h"
#include "Tuareg_service_functions.h"
#include "Tuareg_controls.h"

#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "systick_timer.h"

#include "eeprom.h"
#include "eeprom_layout.h"
#include "process_table.h"
#include "table.h"
#include "map.h"
#include "mapset.h"
#include "ctrlset.h"

#include "highspeed_loggers.h"
#include "syslog.h"
#include "fault_log.h"
#include "diagnostics.h"

#include "debug_port_messages.h"

#include "uart.h"
#include "uart_printf.h"

#include "conversion.h"

#include "ctrlset.h"
#include "Tuareg_control_sets.h"


#define TUAREG_REQUIRED_CONFIG_VERSION 6


#define LOWPRIOSCHEDULER_WIP


extern const char Tuareg_Version [];

extern const crank_position_t cTuareg_controls_update_pos;

/**

REQ_UNITS_DEF:
every variable shall provide the name of the corresponding physical unit, if applicable
e.g. timeout_us, map_kPa, ...

*/


typedef struct _tuareg_flags_t {

    //is engine operation allowed?
    U32 run_inhibit :1;

    //some run_inhibit sources
    U32 crash_sensor_triggered :1;
    U32 run_switch_deactivated :1;
    U32 sidestand_sensor_triggered :1;
    U32 overheat_detected :1;

    //special operation conditions
    U32 service_mode :1;
    U32 limited_op :1;
    U32 rev_limiter :1;
    U32 standby :1;
    U32 cranking :1;

    //additional functions
    U32 fuel_pump_priming :1;
    U32 mil :1;

    /*
    vital actor power state
    */
    U32 ignition_coil_1 :1;
    U32 ignition_coil_2 :1;
    U32 fuel_injector_1 :1;
    U32 fuel_injector_2 :1;
    U32 fuel_pump :1;

    /*
    logging state
    */
    U32 syslog_update :1;
    U32 datalog_update :1;
    U32 highspeedlog_update :1;


} tuareg_flags_t;


typedef struct _tuareg_errors_t {

    U32 fatal_error :1;

    U32 decoder_config_error :1;
    U32 ignition_config_error :1;
    U32 tuareg_config_error :1;
    U32 fueling_config_error :1;
    U32 sensor_calibration_error :1;

    U32 sensor_O2_error :1;
    U32 sensor_TPS_error :1;
    U32 sensor_IAT_error :1;
    U32 sensor_CLT_error :1;
    U32 sensor_VBAT_error :1;
    U32 sensor_KNOCK_error :1;
    U32 sensor_BARO_error :1;
    U32 sensor_GEAR_error :1;
    U32 sensor_MAP_error :1;
    U32 sensor_CIS_error :1;

    //init state
    U32 init_not_completed :1;

    //fault log
    U32 fault_log_error :1;

} tuareg_errors_t;



/**

*/
typedef struct _Tuareg_t {

    /**
    the decoder interface is the primary source for crank position and engine phase
    its data can be considered valid at all time
    */
    decoder_output_t * pDecoder;

    /**
    access to core components
    */
    systick_t * pTimer;


    /**
    Tuareg strategy control
    */
    Tuareg_controls_t Tuareg_controls;

    /**
    current ignition timing and alignment
    */
    ignition_controls_t ignition_controls;

    /**
    current fueling parameters
    */
    fueling_control_t fueling_controls;

    /**
    state machine and health status
    */
    tuareg_flags_t flags;
    tuareg_errors_t errors;

    /**
    process data
    */
    process_data_t process;

    /**
    syslog
    */
    syslog_mgr_flags_t * pSyslog;

    /**
    high speed log
    */
    highspeedlog_flags_t * pHighspeedlog;

    /*
    system watchdogs
    */
    U32 decoder_watchdog;
    U32 injector1_watchdog_ms;
    U32 injector2_watchdog_ms;

    /*
    fuel pump priming
    */
    U32 fuel_pump_priming_remain_s;

} Tuareg_t;


/**
access to global Tuareg data
*/
extern volatile Tuareg_t Tuareg;


void Tuareg_Init();
void Tuareg_load_config();


void Tuareg_update_systick();
void Tuareg_update_run_inhibit();
void Tuareg_update_limited_op();
void Tuareg_update_rev_limiter();
void Tuareg_update_standby();
void Tuareg_update_cranking();

void Tuareg_update_runtime();

void Tuareg_update_fuel_pump_control();

void Tuareg_update_consumption_data();

void Tuareg_update_trip();


void Tuareg_deactivate_vital_actors(bool IgnoreFuelPump);

#endif // TUAREG_H_INCLUDED
