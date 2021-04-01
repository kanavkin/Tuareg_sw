#ifndef TUAREG_H_INCLUDED
#define TUAREG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_errors.h"
#include "Tuareg_config.h"

#include "Tuareg_process_data.h"

#include "table.h"
#include "sensors.h"
#include "process_table.h"

#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"

#include "Tuareg_decoder.h"

#include "systick_timer.h"

#include "Tuareg_fueling_controls.h"

#include "syslog.h"
#include "highspeed_loggers.h"

#define TUAREG_REQUIRED_CONFIG_VERSION 3


/**

REQ_UNITS_DEF:
every variable shall provide the name of the corresponding physical unit, if applicable
e.g. timeout_us, map_kPa, ...

REQ_CONFIGVALUE_DEF:
every module that uses configuration in eeprom storage shall provide default values for all essential items
to allow limp home operation if eeprom has been corrupted

*/


typedef struct _tuareg_flags_t {

    //is engine operation allowed?
    U32 run_inhibit :1;

    //some run_inhibit sources
    U32 crash_sensor_triggered :1;
    U32 run_switch_deactivated :1;
    U32 sidestand_sensor_triggered :1;
    U32 overheat_detected :1;

    //is vital actor operation allowed?
    U32 ignition_inhibit :1;
    U32 fueling_inhibit :1;


    //special operation conditions
    U32 limited_op :1;
    U32 rev_limiter :1;
    U32 cranking :1;
    U32 standstill :1;

    /*
    vital actor power state
    */

    U32 ignition_coil_1 :1;
    U32 ignition_coil_2 :1;
    U32 fuel_injector_1 :1;
    U32 fuel_injector_2 :1;
    U32 fuel_pump :1;

    /*
    ignition irq source flags
    */
    U32 ign1_irq_flag :1;
    U32 ign2_irq_flag :1;

    /*
    service mode
    */
    U32 service_mode :1;


} tuareg_flags_t;


typedef struct _tuareg_errors_t {

    VU32 fatal_error :1;

    VU32 decoder_config_error :1;
    VU32 ignition_config_error :1;
    VU32 tuareg_config_error :1;
    VU32 fueling_config_error :1;
    VU32 sensor_calibration_error :1;

    VU32 sensor_O2_error :1;
    VU32 sensor_TPS_error :1;
    VU32 sensor_IAT_error :1;
    VU32 sensor_CLT_error :1;
    VU32 sensor_VBAT_error :1;
    VU32 sensor_KNOCK_error :1;
    VU32 sensor_BARO_error :1;
    VU32 sensor_GEAR_error :1;
    VU32 sensor_MAP_error :1;
    VU32 sensor_CIS_error :1;

} tuareg_errors_t;



/**

*/
typedef struct _Tuareg_t {

    /**
    the decoder interface is the primary source for crank position and engine phase
    its data can be considered valid at all time
    */
    volatile Tuareg_decoder_t * pDecoder;

    /**
    access to core components
    */
    volatile sensor_interface_t * pSensors;
    volatile systick_t * pTimer;

    /**
    current ignition timing and alignment
    */
    volatile ignition_controls_t ignition_controls;

    /**
    current fueling parameters
    */
    volatile fueling_control_t fueling_controls;

    /**
    state machine and health status
    */
    volatile tuareg_flags_t flags;
    volatile tuareg_errors_t errors;

    /**
    process data
    */
    volatile process_data_t process;

    /**
    syslog
    */
    volatile syslog_mgr_flags_t * pSyslog;

    /**
    high speed log
    */
    volatile highspeedlog_flags_t * pHighspeedlog;

} Tuareg_t;


/**
access to global Tuareg data
*/
extern volatile Tuareg_t Tuareg;

void Tuareg_Init();
void Tuareg_print_init_message();

void Tuareg_update();

void Tuareg_stop_engine();



void Tuareg_export_diag(VU32 * pTarget);



void Tuareg_HWINIT_transition();
void Tuareg_CONFIGLOAD_transition();
void Tuareg_MODULEINIT_transition();
void Tuareg_LIMP_transition();
void Tuareg_SERVICE_transition();
void Tuareg_HALT_transition();
void Tuareg_RUNNING_transition();
void Tuareg_STB_transition();
void Tuareg_CRANKING_transition();
void Tuareg_FATAL_transition();

#endif // TUAREG_H_INCLUDED
