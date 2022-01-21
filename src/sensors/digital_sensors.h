#ifndef DIGITAL_SENSORS_H_INCLUDED
#define DIGITAL_SENSORS_H_INCLUDED


#include "stm32_libs/boctok_types.h"


/**
digital sensor channels:

PORTB4 -> DSENSOR_SPARE2
PORTC0 -> DSENSOR_NEUTRAL
PORTC2 -> DSENSOR_RUN
PORTC3 -> DSENSOR_CRASH
PORTC5 -> DSENSOR_DEBUG
*/

/**
this is to assign integrator cells to sensors
*/
typedef enum {

    //digital sensors
    DSENSOR_SPARE2,
    DSENSOR_SIDESTAND,
    DSENSOR_RUN,
    DSENSOR_CRASH,
    DSENSOR_DEBUG,
    DSENSOR_COUNT

} dsensors_t;



/*********************************************************************************************************************************
dsensor_state_t holds the digital sensors data
take care that the order of sensors in dsensors_t matches dsensor_state_t
*********************************************************************************************************************************/
typedef union
{
     VU32 all_sensors;

    struct
    {
        VU32 spare2 :1;
        VU32 sidestand :1;
        VU32 run :1;
        VU32 crash :1;
        VU32 debug :1;
     };

} dsensors_state_t;





void init_digital_sensors();
void update_digital_sensors();


//interface
extern volatile dsensors_state_t Digital_Sensors;


#endif
