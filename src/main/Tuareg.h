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








typedef enum {

    //system init
    TMODE_INIT,

    //control engine with minimum sensor input available
    TMODE_LIMP,

    //perform diagnostic functions triggered by user, no engine operation
    TMODE_SERVICE,

    //engine operation prohibited due to kill switch or crash sensor
    TMODE_HALT,

    //engine stalled, system ready for start
    TMODE_STB,

    //engine startup
    TMODE_CRANKING,

    //normal engine operation
    TMODE_RUNNING,

    //provide error logs for debugging
    TMODE_FATAL

} tuareg_runmode_t;


typedef union
{
     U8 all_flags;

     struct {

        VU8 crash_sensor :1;
        VU8 run_switch :1;
        VU8 sidestand_sensor :1;
     };

} tuareg_haltsrc_t;


typedef struct {

    //rev limiter
    U32 rev_limiter :1;

    //ignition
    U32 ignition_inhibit :1;
    U32 ignition_coil_1 :1;
    U32 ignition_coil_2 :1;
    U32 ign1_irq_flag :1;
    U32 ign2_irq_flag :1;

    //fueling
    U32 fueling_inhibit :1;
    U32 fuel_injector_1 :1;
    U32 fuel_injector_2 :1;
    U32 fuel_pump :1;

} tuareg_actors_state_t;



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
    volatile ignition_control_t ignition_controls;

    /**
    current fueling parameters
    */
    volatile fueling_control_t fueling_controls;

    /**
    state machine and health status
    */
    volatile tuareg_runmode_t Runmode;
    volatile tuareg_haltsrc_t Halt_source;
    volatile tuareg_error_t Errors;


    volatile process_data_t process;

    //mirrors the actual state of vital actors
    volatile tuareg_actors_state_t actors;

    //decoder watchdog
    VU32 decoder_watchdog_ms;

    //syslog
    volatile syslog_mgr_flags_t * pSyslog;

    //high speed log
    volatile highspeedlog_flags_t * pHighspeedlog;

} Tuareg_t;


/**
access to global Tuareg data
*/
extern volatile Tuareg_t Tuareg;

void Tuareg_print_init_message();

void Tuareg_update_Runmode();
void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode);

extern void Tuareg_stop_engine();

void Tuareg_export_diag(VU32 * pTarget);
void Tuareg_update_halt_sources();


void Tuareg_Init();

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
