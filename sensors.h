#ifndef SENSORS_H_INCLUDED
#define SENSORS_H_INCLUDED


#include "stm32_libs/boctok_types.h"
#include "decoder_logic.h"


/**
digital sensor channels:

PORTB4 -> DSENSOR_SPARE2
PORTC0 -> DSENSOR_NEUTRAL
PORTC2 -> DSENSOR_RUN
PORTC3 -> DESNSOR_CRASH
PORTC5 -> DSENSOR_DEBUG
*/

/**
with a sensor readout rate of 4HZ it will report a logic "1" after 1 s
*/
#define DSENSOR_CYCLE_LEN 6
#define DSENSOR_HIGH_THRES 4


//use ADC1 for analog sensors
#define SENSOR_ADC ADC1

/**
ADC channels vs analog sensors

0   -> PORTA0  -> ASENSOR_O2
1   -> PORTA1  -> ASENSOR_TPS
2   -> PORTA2  -> ASENSOR_MAP
3   -> PORTA3  -> ASENSOR_IAT
4   -> PORTA4  -> ASENSOR_CLT
5   -> PORTA5  -> ASENSOR_VBAT
6   -> PORTA6  -> ASENSOR_KNOCK
7   -> PORTA7  -> ASENSOR_BARO
14  -> PORTC4  -> ASENSOR_SPARE
*/

/*
"wasted" ADC channels:
8  -> PORTB0    (used for Decoder - CRANK)
9  -> PORTB1    (used for Decoder - CAM)
10  -> PORTC0   (used for digital spare 1 sensor)
11  -> PORTC1   (used for VR spare channel)
12  -> PORTC2   (used for run switch)
13  -> PORTC3   (used for crash switch)
15  -> PORTC5   (used for debug switch)
*/


#define ADC_O2_CH       ADC_Channel_0
#define ADC_TPS_CH      ADC_Channel_1
#define ADC_MAP_CH      ADC_Channel_2
#define ADC_IAT_CH      ADC_Channel_3
#define ADC_CLT_CH      ADC_Channel_4
#define ADC_VBAT_CH     ADC_Channel_5
#define ADC_KNOCK_CH    ADC_Channel_6
#define ADC_BARO_CH     ADC_Channel_7
#define ADC_SPARE_CH    ADC_Channel_14



/**
generic values
TODO set more specific ones
TODO make values configurable (config page)

sensor timing considerations:
with an average buffer of 16 bit width we can fit up to 8
12 bit ADC values,
keep good balance between CPU load and accuracy!

T_upd= ASENSOR_x_AVG_THRES * 20ms * loop_count
*/
#define ASENSOR_MAP_MIN_THRES 302
#define ASENSOR_MAP_MAX_THRES 4095
#define ASENSOR_MAP_ERROR_THRES 0xFF
#define ASENSOR_MAP_AVG_THRES 16

#define ASENSOR_BARO_MIN_THRES 302
#define ASENSOR_BARO_MAX_THRES 4095
#define ASENSOR_BARO_ERROR_THRES 0xFF
#define ASENSOR_BARO_AVG_THRES 5

#define ASENSOR_TPS_MIN_THRES 1
#define ASENSOR_TPS_MAX_THRES 4095
#define ASENSOR_TPS_ERROR_THRES 0xFF
#define ASENSOR_TPS_AVG_THRES 5
#define DELTA_TPS_THRES 5

#define ASENSOR_O2_MIN_THRES 1
#define ASENSOR_O2_MAX_THRES 4095
#define ASENSOR_O2_ERROR_THRES 0xFF
#define ASENSOR_O2_AVG_THRES 5

#define ASENSOR_VBAT_MIN_THRES 1
#define ASENSOR_VBAT_MAX_THRES 4095
#define ASENSOR_VBAT_ERROR_THRES 0xFF
#define ASENSOR_VBAT_AVG_THRES 5

#define ASENSOR_IAT_MIN_THRES 1
#define ASENSOR_IAT_MAX_THRES 4095
#define ASENSOR_IAT_ERROR_THRES 0xFF
#define ASENSOR_IAT_AVG_THRES 5
#define IAT_OFFSET 40

#define ASENSOR_CLT_MIN_THRES 1
#define ASENSOR_CLT_MAX_THRES 4095
#define ASENSOR_CLT_ERROR_THRES 0xFF
#define ASENSOR_CLT_AVG_THRES 5
#define CLT_OFFSET 40

#define ASENSOR_SPARE_MIN_THRES 1
#define ASENSOR_SPARE_MAX_THRES 4095
#define ASENSOR_SPARE_ERROR_THRES 0xFF
#define ASENSOR_SPARE_AVG_THRES 5

//------- new generic structure:

#define ASENSOR_MIN_VALID 1
#define ASENSOR_MAX_VALID 4095

#define ASENSOR_ASYNC_SAMPLE_LEN 5
#define ASENSOR_SYNC_SAMPLE_LEN 16

#define ASENSOR_ERROR_THRES 0xFF


/**
sensor module internal type
the enumerators ASENSOR_ASYNC_yy address array elements in the internals struct: asensors_async_error_counter, asensors_async_integrator, asensors_async_integrator_count

their order is important and shall match the regular ADC group configuration!
*/
typedef enum {

    //ADC based sensors in regular group:
    ASENSOR_ASYNC_O2,
    ASENSOR_ASYNC_TPS,
    ASENSOR_ASYNC_IAT,
    ASENSOR_ASYNC_CLT,
    ASENSOR_ASYNC_VBAT,
    ASENSOR_ASYNC_KNOCK,
    ASENSOR_ASYNC_BARO,
    ASENSOR_ASYNC_SPARE,
    ASENSOR_ASYNC_COUNT


} asensors_async_t;

/**
sensor module internal type
the enumerators ASENSOR_SYNC_yy address array elements in the internals struct: asensors_sync_error_counter, asensors_sync_integrator, asensors_sync_integrator_count

their order is important and shall match the injected ADC group configuration!
*/
typedef enum {

    //ADC based sensors in injected group:
    ASENSOR_SYNC_MAP,
    ASENSOR_SYNC_COUNT


} asensors_sync_t;


/**
sensor module internal type
*/
typedef enum {

    //internal ADC channels
    ASENSOR_INTERNAL_TEMP,
    ASENSOR_INTERNAL_VREF,
    ASENSOR_INTERNAL_COUNT

} asensors_internal_t;

/**
interface type

provides unified access to analog sensor values and hides the technical details (internal separation into synchronous and asynchronous sensors / regular and injected ADC channels) from the user

the enumerators address array elements in the interface struct: asensors_async_error_counter, asensors_async_integrator, asensors_async_integrator_count

their order is important and shall match asensors_async_t and asensors_sync_t layout!
*/
typedef enum {

    ASENSOR_O2,
    ASENSOR_TPS,
    ASENSOR_IAT,
    ASENSOR_CLT,
    ASENSOR_VBAT,
    ASENSOR_KNOCK,
    ASENSOR_BARO,
    ASENSOR_SPARE,
    ASENSOR_MAP,
    ASENSOR_COUNT

} asensors_t;

/**
internal and interface type
choose DSENSOR_xx values so that they can act as flags in a U8 (sensors.digital_sensors)
their order is not important
*/
typedef enum {

    //digital sensors
    DSENSOR_SPARE2,
    DSENSOR_NEUTRAL,
    DSENSOR_RUN,
    DSENSOR_CRASH,
    DSENSOR_DEBUG,
    DSENSOR_COUNT

} dsensors_t;


typedef enum {

    SDIAG_READ_DSENSORS_CALLS,
    SDIAG_ADCIRQ_CALLS,
    SDIAG_ADCIRQ_INJECTEDGR_CALLS,
    SDIAG_DMAIRQ_CALLS,
    SDIAG_DMAIRQ_CH1_CALLS,

    SDIAG_COUNT

} sensors_diag_t;


/**
sensor control

ADC measurement average values, its counters and error counters we address through average[ASENSOR_yy], average_counter[ASENSOR_yy] and error_counter[ASENSOR_yy]
only MAP sensor uses map_integrator and map_integrator_count for averaging.

when modifying sensors keep all indexes of regular group sensors below index of asensor_map
*/
typedef struct {

    VU8 dsensor_cycle;
    VU8 dsensor_history[DSENSOR_CYCLE_LEN];

    VU8 asensors_async_error_counter[ASENSOR_ASYNC_COUNT];
    VU8 asensors_sync_error_counter[ASENSOR_SYNC_COUNT];

    VU16 asensors_async_integrator[ASENSOR_ASYNC_COUNT];
    VU8 asensors_async_integrator_count[ASENSOR_ASYNC_COUNT];

    //MAP sensor holds 16 samples -> U32
    VU32 asensors_sync_integrator[ASENSOR_SYNC_COUNT];
    VU8 asensors_sync_integrator_count[ASENSOR_SYNC_COUNT];


    VF32 last_TPS;

    VU8 async_loop_count;

    VU32 diag[SDIAG_COUNT];

    /**
    where DMA will drop ADC data from regular group
    */
    VU16 asensors_async_buffer[ASENSOR_ASYNC_COUNT];

} sensor_internals_t;


typedef struct {

    VF32 ddt_TPS;

    VU16 asensors_health;

    VF32 asensors[ASENSOR_COUNT];
    VU16 asensors_raw[ASENSOR_COUNT];
    VU16 asensors_valid[ASENSOR_COUNT];

    VU8 dsensors;

} sensor_interface_t;

volatile sensor_interface_t * init_sensors();
VU32 read_dsensors();
void read_digital_sensors();
float calc_inverse_lin(U32 Arg, float M, float N);
float calculate_ddt_TPS(float Last_TPS, float Current_TPS);
void reset_asensor_sync_integrator(asensors_sync_t Sensor);

extern const float cKelvin_offset;

#endif // SENSORS_H_INCLUDED
