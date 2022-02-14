#ifndef SENSORS_H_INCLUDED
#define SENSORS_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

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
14  -> PORTC4  -> ASENSOR_GEAR
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
#define ADC_GEAR_CH     ADC_Channel_14



/**
generic values

MAP sensor average configuration:

For the XTZ 850 engine, MAP value be calculated at every crank turn, based on the samples taken on every crank position update
*/
#define ASENSOR_SYNC_SAMPLE_CRK_REVS 1

#define ASENSOR_SYNC_SAMPLE_LEN (ASENSOR_SYNC_SAMPLE_CRK_REVS * CRK_POSITION_COUNT)

#define ASENSOR_ERROR_THRES 0xFF

/*********************************************************************************************************************************
analog sensor definition

asynchronuous sensors are handled by timer triggered reguler ADC group conversion, synchronuous sensors form the injected group

*********************************************************************************************************************************/
typedef enum {

    //asynchronuous sensors
    ASENSOR_O2,
    ASENSOR_TPS,
    ASENSOR_IAT,
    ASENSOR_CLT,
    ASENSOR_VBAT,
    ASENSOR_KNOCK,
    ASENSOR_BARO,
    ASENSOR_GEAR,

    //synchronuous sensors
    ASENSOR_MAP,

    //end
    ASENSOR_COUNT

} asensors_t;

#define LAST_ASYNC_ASENSOR ASENSOR_GEAR


/***********************************************************************************************************************************
holds the per analog sensor channel data

the data in out reflects the measured physical value, calculated from the last valid sample
(will never be calculated from an invalid sample, can be outdated within the valid/invalid count window)

raw is the average readout, calculated from the last valid samples
(will never be calculated from an invalid sample, can be outdated within the valid/invalid count window)
***********************************************************************************************************************************/
typedef struct {

    //health data - counting consecutive (in)valid samples
    VU32 invalid_count;
    VU32 valid_count;

    //moving average calculation
    VU32 integrator;
    VU32 integrator_count;

    //sensor data
    VF32 out;
    VU32 raw;

} asensor_data_t;


/**
holds the parameters per analog sensor channel

a sample is considered valid if it is between min_valid and max_valid
target sample length indicates, how many valid samples will form the moving average
*/
typedef struct __attribute__ ((__packed__)) {

    //linearization parameters
    VF32 M;
    VF32 N;

    //validation thresholds
    VU16 min_valid;
    VU16 max_valid;

    //update parameters
    VU8 target_sample_length;

} asensor_parameters_t;




extern volatile asensor_data_t Analog_Sensors[ASENSOR_COUNT];



void init_sensor_inputs(U32 Init_count);

void sensors_start_regular_group_conversion();
void sensors_start_injected_group_conversion();


void reset_integrator(volatile asensor_data_t * pSensorData);
void update_analog_sensor(asensors_t Sensor, U32 Sample);


#endif // SENSORS_H_INCLUDED
