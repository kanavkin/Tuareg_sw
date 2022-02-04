#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"

#include "Tuareg_types.h"

#include "digital_sensors.h"
#include "diagnostics.h"



//integrator
VU32 DSensor_Intergrator[DSENSOR_COUNT];
VU32 DSensor_Intergrator_count;

//interface
volatile dsensors_state_t Digital_Sensors;



/*********************************************************************************************************************************
digital sensor helper functions
*********************************************************************************************************************************/
void reset_dsensor_integrator()
{
    VU32 sensor;

    for(sensor= 0; sensor < DSENSOR_COUNT; sensor++)
    {
        DSensor_Intergrator[sensor]= 0;
    }

    DSensor_Intergrator_count= 0;
}




/*********************************************************************************************************************************
digital sensor initialization
*********************************************************************************************************************************/
void init_digital_sensors()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    //GPIO run + crash + spare 1/2 + debug
    GPIO_configure(GPIOC, 0, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 2, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 3, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 5, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 1, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 4, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);

    //clean up
    reset_dsensor_integrator();
}



/*********************************************************************************************************************************
digital sensor sampling
*********************************************************************************************************************************/
void read_dsensors()
{
    //DSENSOR_SPARE2
    if(GPIOB->IDR & GPIO_IDR_IDR4)
    {
        DSensor_Intergrator[DSENSOR_SPARE2] += 1;
    }

    //DSENSOR_SIDESTAND
    if(GPIOC->IDR & GPIO_IDR_IDR0)
    {
        DSensor_Intergrator[DSENSOR_SIDESTAND] += 1;
    }

    //DSENSOR_RUN
    if(GPIOC->IDR & GPIO_IDR_IDR2)
    {
        DSensor_Intergrator[DSENSOR_RUN] += 1;
    }

    //DSENSOR_CRASH
    if(GPIOC->IDR & GPIO_IDR_IDR3)
    {
        DSensor_Intergrator[DSENSOR_CRASH] += 1;
    }

    //DSENSOR_DEBUG
    if(GPIOC->IDR & GPIO_IDR_IDR5)
    {
        DSensor_Intergrator[DSENSOR_DEBUG] += 1;
    }

    /**
    count fresh samples
    */
    DSensor_Intergrator_count += 1;

}



/*********************************************************************************************************************************
digital sensor periodic update function
*********************************************************************************************************************************/


/**
built in defaults

this update function gets called every 20 ms
a delay of 500 ms is ok

cCycleLength indicates, how many samples are taken until the new sensor state is evaluated
cHighThres defines, how many samples with a high logic state are needed to set the output value
*/
const U32 cCycleLength= 25;
const U32 cHighThres= 20;



void update_digital_sensors()
{
    VU32 sensor;

    //collect diagnostic data
    sensors_diag_log_event(SNDIAG_UPDATE_DSENSORS_CALLS);


    //check if sampling cycle end has been reached
    if(DSensor_Intergrator_count < cCycleLength)
    {
        //read digital sensors
        read_dsensors();
    }
    else
    {
        //do evaluation
        for(sensor=0; sensor < DSENSOR_COUNT; sensor++)
        {
            //check which logical level applies
            if(DSensor_Intergrator[sensor] >= cHighThres)
            {
                Digital_Sensors.all_sensors |= (1 << sensor);
            }
            else
            {
                Digital_Sensors.all_sensors &= ~(1 << sensor);
            }
        }

        //clean up
        reset_dsensor_integrator();
    }
}


