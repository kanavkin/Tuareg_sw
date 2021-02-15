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

/**

REQ_UNITS_DEF:
every variable shall provide the name of the corresponding physical unit, if applicable
e.g. timeout_us, map_kPa, ...

REQ_CONFIGVALUE_DEF:
every module that uses config in eeprom storage shall provide default values for all essential items
to allow limp home operation if eeprom has ben corrupted




*/

#define IGNITION_CONTROLS_UPDATE_POSITION CRK_POSITION_B2


//level at which the crash sensor reports a crash event
#define CRASH_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_CRASH)

//level at which the run sensor reports a run permission
#define RUN_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_RUN)


/*
ASENSOR_VALIDITY_THRES of consecutive valid captures an analog sensor has to provide until he is considered valid

the counter will be initialized with ASENSOR_VALIDITY_FASTINIT to provide sensor data for startup
*/
#define ASENSOR_VALIDITY_THRES 150
#define ASENSOR_VALIDITY_FASTINIT 100


/**
default sensor values

if an analog sensor is not available, use these defaults
*/
#define MAP_DEFAULT_KPA 100
#define BARO_DEFAULT_KPA 100
#define TPS_DEFAULT_DEG 45
#define O2_DEFAULT_AFR 14.5
#define IAT_DEFAULT_C 20
#define CLT_DEFAULT_C 85
#define VBAT_DEFAULT_V 14
#define KNOCK_DEFAULT 0
#define GEAR_DEFAULT 0


typedef enum {

    //boot time
    TMODE_BOOT,

    //system init
    TMODE_HWINIT,
    TMODE_CONFIGLOAD,
    TMODE_MODULEINIT,

    //control engine with minimum sensor input available
    TMODE_LIMP,

    //perform diagnostic functions triggered by user, no engine operation
    TMODE_DIAG,

    //engine operation prohibited due to kill switch or crash sensor
    TMODE_HALT,

    //engine stalled, system ready for start
    TMODE_STB,

    //engine startup
    TMODE_CRANKING,

    //normal engine operation
    TMODE_RUNNING

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
    volatile Tuareg_decoder_t * decoder;

    /**
    access to core components
    */
    volatile sensor_interface_t * sensors;

    /**
    current ignition timing and alignment
    */
    volatile ignition_control_t ignition_controls;

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

extern void reset_decoder_watchdog();
extern void update_decoder_watchdog();

#endif // TUAREG_H_INCLUDED
