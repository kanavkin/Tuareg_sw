#ifndef TUAREG_H_INCLUDED
#define TUAREG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_errors.h"
#include "Tuareg_config.h"
#include "table.h"
#include "ignition_logic.h"
#include "sensors.h"
#include "process_table.h"

/**

REQ_UNITS_DEF:
every variable shall provide the name of the corresponding physical unit, if applicable
e.g. timeout_us, map_kPa, ...

REQ_CONFIGVALUE_DEF:
every module that uses config in eeprom storage shall provide default values for all essential items
to allow limp home operation if eeprom has ben corrupted




*/

#define PROCESS_DATA_UPDATE_POSITION CRK_POSITION_B1
#define IGNITION_CONTROLS_UPDATE_POSITION CRK_POSITION_B2



/// TODO (oli#8#): find elegant solution for configuration values

//level at which the crash sensor reports a crash event
#define CRASH_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_CRASH)

//level at which the run sensor reports a run permission
#define RUN_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_RUN)

//amount of consecutive valid captures an analog senor has to provide until he is considered valid
#define ASENSOR_VALIDITY_THRES 150



//#define CYLINDER_SENSOR_POSITION POSITION_D2

//TODO: remove deprecated arduino type definitions in Tuareg_t
#include "arduino_types.h"

#define CRANK_ANGLE_MAX  720
#define CRANK_ANGLE_MAX_IGN  360
#define CRANK_ANGLE_MAX_INJ  360 // The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential


/**
This is the maximum rpm that the ECU will attempt to run at.
It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be allowed to run.
Lower number gives better performance
*/
#define MAX_RPM 9000


/**
default sensor values

if an analog sensor is not available, use these defaults
*/
#define MAP_DEFAULT_KPA 100
#define BARO_DEFAULT_KPA 100
#define TPS_DEFAULT_DEG 45
#define O2_DEFAULT_AFR 145
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
    TMODE_RUNNING,

    //run internal checks
    TMODE_MODULE_TEST

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





/**
The status struct contains the current values for all 'live' variables
In current version this is 64 bytes
*/
typedef struct _Tuareg_t {

    /**
    the decoder interface is the primary source for crank position and engine phase
    its data can be considered valid at all time
    */
    volatile decoder_interface_t * decoder;

    /**
    access to core components
    */
    volatile sensor_interface_t * sensors;

    /**
    current ignition timing and alignment
    */
    volatile ignition_control_t ignition_controls;

    /**
    statemachine and health status
    */
    volatile tuareg_runmode_t Runmode;
    volatile tuareg_haltsrc_t Halt_source;
    volatile tuareg_error_t Errors;


    volatile process_data_t process;



} Tuareg_t;


/**
access to global Tuareg data
*/
extern volatile Tuareg_t Tuareg;

void Tuareg_print_init_message();
void Tuareg_update_Runmode();
void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode);
void Tuareg_stop_engine();
void Tuareg_update_process_data();


VF32 Tuareg_update_MAP_sensor();
VF32 Tuareg_update_GEAR_sensor();
VF32 Tuareg_update_BARO_sensor();
VF32 Tuareg_update_KNOCK_sensor();
VF32 Tuareg_update_VBAT_sensor();
VF32 Tuareg_update_CLT_sensor();
VF32 Tuareg_update_IAT_sensor();
VF32 Tuareg_update_TPS_sensor();
VF32 Tuareg_update_ddt_TPS();
VF32 Tuareg_update_O2_sensor();
VF32 Tuareg_update_MAP_sensor();


void Tuareg_export_diag(VU32 * pTarget);


void Tuareg_update_halt_sources();

#endif // TUAREG_H_INCLUDED
