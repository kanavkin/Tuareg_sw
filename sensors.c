#include <math.h>

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"
#include "sensors.h"
#include "uart.h"
#include "conversion.h"
#include "table.h"
#include "config.h"
#include "decoder_logic.h"
#include <math.h>

volatile sensor_interface_t SInterface;
volatile sensor_internals_t SInternals;

const float cKelvin_offset= 273.15;

/**
see sensors.h for sensor layout!
*/


/**
bring up analog and digital sensors
*/
volatile sensor_interface_t * init_sensors()
{
    DMA_InitTypeDef DMA_InitStructure;

    //set ADC prescaler to 2 (source: APB2/PCLK2 @ 50 MHz) -> 25 MHz
    ADC->CCR &= ~ADC_CCR_ADCPRE;

    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    //GPIO ADC CH0..7 + Port C4
    GPIO_configure(GPIOA, 0, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 1, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 2, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 3, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 4, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 5, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 6, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 7, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 4, GPIO_MODE_ANALOG, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    //GPIO run + crash + spare 1/2 + debug
    GPIO_configure(GPIOC, 0, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 2, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 3, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOC, 5, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 1, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 4, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);


    /**
    ADC setup low fat variant:
    CR1 ->  use SCAN mode and IRQ on injected conversion end
    CR2 ->  sw external trigger for regular + injected groups,
            use DMA for regular group, data align right, independent mode,
            no continuous conversion
    */
    ADC1->CR1= ADC_CR1_SCAN | ADC_CR1_JEOCIE;
    ADC1->CR2= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;

    /* ADC calibration will be performed by hw at startup */


    /**
    ADC channels setup

    Regular group handling:
    The enumerator value of ASENSOR_  in sensors_t is used as its address in ADCBuffer[].
    The ADC converts the regular group channels in the order we set through adc_set_regular_group.
    DMA writes the conversion results to ADCBuffer[]
    -> The channel order must match the enumerator values!
    -> keep REGULAR_GROUP_LENGTH up to date!

    sensor:     O2   CLT    SPARE
                |     |       |
    group nr.   1 2 3 4 5 6 7 8
                | | | | | | | |
    ADCBuffer[] 0 1 2 3 4 5 6 7

    Injected group handling:
    MAP sensor is in the injected group
    ADC conversion result is read in irq

    ADC clock is 12MHz, conversion time will be 12.5 + sample_time * T_adc
    so total regular conversion time should be ~9us
    */
    adc_set_sample_time(ADC1, ADC_O2_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_TPS_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_MAP_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_IAT_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_CLT_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_VBAT_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_KNOCK_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_BARO_CH, SAMPLE_TIME_7_5);
    adc_set_sample_time(ADC1, ADC_SPARE_CH, SAMPLE_TIME_7_5);

    /**
    the order specified here shall match the enumerator order in asensors_async_t !
    */
    adc_set_regular_group(ADC1, 1, ADC_O2_CH);
    adc_set_regular_group(ADC1, 2, ADC_TPS_CH);
    adc_set_regular_group(ADC1, 3, ADC_IAT_CH);
    adc_set_regular_group(ADC1, 4, ADC_CLT_CH);
    adc_set_regular_group(ADC1, 5, ADC_VBAT_CH);
    adc_set_regular_group(ADC1, 6, ADC_KNOCK_CH);
    adc_set_regular_group(ADC1, 7, ADC_BARO_CH);
    adc_set_regular_group(ADC1, 8, ADC_SPARE_CH);

    //set the number of channels in the regular group in sensors.h!
    adc_set_regular_group_length(ADC1, ASENSOR_ASYNC_COUNT);

    /**
    setting up the injected group is a bit tricky:
    when we have only one conversion in group, ADC_JSQR.JL is 0 and only JSQ4 entry will
    define which channel to convert.
    Conversion result will be in ADC1->JDR1 !
    */
    ADC1->JSQR= (U32) (ADC_MAP_CH << 15);

    /*
    configure DMA2 CH0 Stream0 for ADC
    */

    //preset struct elements with default values -> lines commented out means that we keep the defaults
    DMA_StructInit(&DMA_InitStructure);

    //DMA_InitStructure.DMA_Channel= DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr= (U32)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr= (U32)SInternals.asensors_async_buffer;
    //DMA_InitStructure.DMA_DIR= DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize= ASENSOR_ASYNC_COUNT;
    //DMA_InitStructure.DMA_PeripheralInc= DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc= DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize= DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode= DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority= DMA_Priority_High;
    //DMA_InitStructure.DMA_FIFOMode= DMA_FIFOMode_Disable;
    //DMA_InitStructure.DMA_FIFOThreshold= DMA_FIFOThreshold_1QuarterFull;
    //DMA_InitStructure.DMA_MemoryBurst= DMA_MemoryBurst_Single;
    //DMA_InitStructure.DMA_PeripheralBurst= DMA_PeripheralBurst_Single;

    //fire up config
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    //enable DMA1_Channel1 and its irq
    DMA2_Stream0->CR |= (U32) (DMA_SxCR_EN | DMA_SxCR_TCIE);

    //Clear all DMA2 stream0 interrupt pending bits
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    /**
    NVIC
    */

    //DMA2 stream0 (prio 7)
    NVIC_SetPriority(DMA2_Stream0_IRQn, 7UL);
    NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    //ADC1-2 irq (prio 6)
    NVIC_SetPriority(ADC_IRQn, 6UL);
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);

    return &SInterface;
}


/**
sensor data conversion by inverse linear function
using float for maximum precision

x = ( # - n)  / m

*/
VF32 calc_inverse_lin(VU32 Arg, VF32 M, VF32 N)
{
    F32 inverse;

    //subtract N
    if( N < Arg)
    {
        inverse= Arg - N;
    }
    else
    {
        //clip result
        return 0.0;
    }

    //divide by M
    if( (M > 0.0) || (M < 0.0) )
    {
        /**
        do not ever delete by zero
        so we test if calib_M is set to something usable
        -> calibration loading success should be monitored and
        there should be default values if an error occurred while loading
        ... but who can be sure for sure... ;9
        */
        return (inverse / M);
    }
    else
    {
        //clip result
        return 0.0;
    }

}



/**
throttle transient calculation
as the change in airflow restriction past the throttle is a inverse logarithmic curve
we use a weighted measure:
ddt_TPS= (tps_new - tps_old) * (101 - tps_old)
but suppress little spikes

calculation relies on the converted TPS value (0..100)!
*/
VF32 calculate_ddt_TPS(VF32 Last_TPS, VF32 Current_TPS)
{
    F32 delta_TPS;

    delta_TPS= Current_TPS - Last_TPS;

    if( ((delta_TPS < 0.0) && (-delta_TPS < DELTA_TPS_THRES)) || ((delta_TPS >= 0.0) && (delta_TPS < DELTA_TPS_THRES)) )
    {
        return 0.0;
    }
    else
    {
        return (delta_TPS * (101.0 - Last_TPS));
    }
}



/**
digital sensors wiring dependent part
*/
VU32 read_dsensors()
{
    VU8 readout =0;

    //DSENSOR_SPARE2
    if(GPIOB->IDR & GPIO_IDR_IDR4)
    {
        readout |= (1 << DSENSOR_SPARE2);
    }

    //DSENSOR_NEUTRAL
    if(GPIOC->IDR & GPIO_IDR_IDR0)
    {
        readout |= (1 << DSENSOR_NEUTRAL);
    }

    //DSENSOR_RUN
    if(GPIOC->IDR & GPIO_IDR_IDR2)
    {
        readout |= (1 << DSENSOR_RUN);
    }

    //DSENSOR_CRASH
    if(GPIOC->IDR & GPIO_IDR_IDR3)
    {
        readout |= (1 << DSENSOR_CRASH);
    }

    //DSENSOR_DEBUG
    if(GPIOC->IDR & GPIO_IDR_IDR5)
    {
        readout |= (1 << DSENSOR_DEBUG);
    }

    return readout;
}


/**
handling digital sensors seems so easy
compared to analog ones ;)
*/
void read_digital_sensors()
{
    U32 cnt, level, sensor =0;

    //collect diagnostic data
    SInternals.diag[SDIAG_READ_DSENSORS_CALLS] += 1;

    if(SInternals.dsensor_cycle < DSENSOR_CYCLE_LEN)
    {
        //save digital sensors state to history
        SInternals.dsensor_history[SInternals.dsensor_cycle]= read_dsensors();

        SInternals.dsensor_cycle++;
    }
    else
    {
        /**
        cycle end -> do evaluation
        */

        //for every sensor in list
        for(sensor=0; sensor < DSENSOR_COUNT; sensor++)
        {
            //start counting with an empty counter
            level =0;

            //process every sample in history
            for(cnt =0; cnt < DSENSOR_CYCLE_LEN; cnt++)
            {
                /**
                count the number of samples with logic "1" state
                */
                if( ((U32) SInternals.dsensor_history[cnt]) & (1 << sensor) )
                {
                    level++;
                }

            }

            /**
            do evaluation
            high level shall be detected if at least DSENSOR_HIGH_THRES samples with logic "1" state have been collected
            */
            if(level >= DSENSOR_HIGH_THRES)
            {
                SInterface.dsensors |= (1 << sensor);
            }
            else
            {
                SInterface.dsensors &= ~(1 << sensor);
            }

        }

        //cycle end reached
        SInternals.dsensor_cycle =0;

    }
}


/**
this function can be used to reset the captured data from synchronous sensors e.g. in case of a sync lost event
*/
void reset_asensor_sync_integrator(asensors_sync_t Sensor)
{
    SInternals.asensors_sync_integrator[Sensor]= 0;
    SInternals.asensors_sync_integrator_count[Sensor]= 0;
}


/**
MAP sensor readout as injected conversion
(can be triggered synchronous to crank movement)
*/
void ADC_IRQHandler()
{
    VU32 average, sample;
    VU32 * pIntegr= NULL;
    VU8 * pCount= NULL;

    //collect diagnostic data
    SInternals.diag[SDIAG_ADCIRQ_CALLS] += 1;

    //MAP sensor handled by injected group
    if(ADC1->SR & ADC_SR_JEOC)
    {
        //collect diagnostic data
        SInternals.diag[SDIAG_ADCIRQ_INJECTEDGR_CALLS] += 1;

        //clear JEOC by write 0
        ADC1->SR &= ~(U32) ADC_SR_JEOC;

        //read ADC value
        sample= ADC1->JDR1;

        //access to sensor data
        pIntegr= &(SInternals.asensors_sync_integrator[ASENSOR_SYNC_MAP]);
        pCount= &(SInternals.asensors_sync_integrator_count[ASENSOR_SYNC_MAP]);

        //validate sample
        if( (sample >= ASENSOR_MIN_VALID) && (sample <= ASENSOR_MAX_VALID) )
        {
            /**
            valid sample
            */

            //store to average buffer
            *pIntegr += sample;
            *pCount += 1;

            //enough samples read?
            if(*pCount >= ASENSOR_SYNC_SAMPLE_LEN)
            {
                //calculate the average map value
                average= *pIntegr / *pCount;

                //calculate MAP and export to interface
                SInterface.asensors[ASENSOR_MAP]= calc_inverse_lin(average, configPage9.MAP_calib_M, configPage9.MAP_calib_N);

                //reset average buffer
                *pIntegr= 0;
                *pCount= 0;

                //mark sensor as active, reset error counter
                SInterface.asensors_health |= (1<< ASENSOR_MAP);
                SInternals.asensors_sync_error_counter[ASENSOR_SYNC_MAP] =0;

                //export raw value
                SInterface.asensors_raw[ASENSOR_MAP]= average;

                //count the amount of consecutive valid readings
                SInterface.asensors_valid_samples[ASENSOR_MAP]++;
            }

        }
        else
        {
            /**
            invalid reading, do error handling
            */

            if(SInternals.asensors_sync_error_counter[ASENSOR_SYNC_MAP] < ASENSOR_ERROR_THRES)
            {
                SInternals.asensors_sync_error_counter[ASENSOR_SYNC_MAP]++;
            }
            else
            {
                /**
                sensor temporarily disturbed!
                */

                //report to interface
                SInterface.asensors[ASENSOR_MAP] =0;
                SInterface.asensors_health &= ~(1<< ASENSOR_MAP);

                //reset average buffer
                *pIntegr= 0;
                *pCount= 0;

                //delete raw value
                SInterface.asensors_raw[ASENSOR_MAP]= 0;

                //no more consecutive valid readings
                SInterface.asensors_valid_samples[ASENSOR_MAP] =0;

            }

        }

    }

}


/**
The regular group conversion is triggered by lowspeed_timer every 20 ms (50Hz)
with 5x oversampling this gives an update interval of 100 ms
*/
/// TODO (oli#1#): is 100ms to slow?
void DMA2_Stream0_IRQHandler()
{

    U32 average, sample, sensor, result;
    VU16 * pIntegr= NULL;
    VU8 * pCount= NULL;

    //collect diagnostic data
    SInternals.diag[SDIAG_DMAIRQ_CALLS] += 1;

    //DMA1 Channel1 Transfer Complete interrupt
    if(DMA2->LISR & DMA_LISR_TCIF0)
    {
        //collect diagnostic data
        SInternals.diag[SDIAG_DMAIRQ_CH1_CALLS] += 1;

        //Clear all DMA2 stream0 interrupt pending bits
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

        /**
        update frequency shall be equal for every asensor_async -> every loop
        */

        //for every async asensor
        for(sensor =0; sensor < ASENSOR_ASYNC_COUNT; sensor++)
        {
            //read ADC input value
            sample= SInternals.asensors_async_buffer[sensor];

            //get easy access to sensor data
            pIntegr= &(SInternals.asensors_async_integrator[sensor]);
            pCount= &(SInternals.asensors_async_integrator_count[sensor]);

            if( (sample >= ASENSOR_MIN_VALID) && (sample <= ASENSOR_MAX_VALID) )
            {
                /**
                valid reading
                */

                //store to integrator
                *pIntegr += sample;
                *pCount += 1;

                //enough samples read?
                if(*pCount >= ASENSOR_ASYNC_SAMPLE_LEN)
                {
                    //calculate the average value
                    average= *pIntegr / *pCount;

                    /**
                    ADC value to process data conversion
                    */
                    switch(sensor)
                    {
                        case ASENSOR_ASYNC_O2:

                            result= calc_inverse_lin(average, configPage9.O2_calib_M, configPage9.O2_calib_N);
                            break;

                        case ASENSOR_ASYNC_TPS:

                            result= calc_inverse_lin(average, configPage9.TPS_calib_M, configPage9.TPS_calib_N);

                            //save old TPS value for ddt_TPS calculation
                            SInternals.last_TPS= SInterface.asensors[ASENSOR_TPS];

                            //throttle transient calculation
                            SInterface.ddt_TPS= calculate_ddt_TPS(SInternals.last_TPS, result);

                            break;

                        case ASENSOR_ASYNC_IAT:

                            result= calc_inverse_lin(average, configPage9.IAT_calib_M, configPage9.IAT_calib_N);
                            result += cKelvin_offset;
                            break;

                        case ASENSOR_ASYNC_CLT:

                            result= calc_inverse_lin(average, configPage9.CLT_calib_M, configPage9.CLT_calib_N);
                            result += cKelvin_offset;
                            break;

                        case ASENSOR_ASYNC_VBAT:

                            result= calc_inverse_lin(average, configPage9.VBAT_calib_M, configPage9.VBAT_calib_N);
                            break;

                        case ASENSOR_ASYNC_KNOCK:

                            result= calc_inverse_lin(average, configPage9.KNOCK_calib_M, configPage9.KNOCK_calib_N);
                            break;

                        case ASENSOR_ASYNC_BARO:

                            result= calc_inverse_lin(average, configPage9.BARO_calib_M, configPage9.BARO_calib_N);
                            break;

                        default:

                            //e.g. ASENSOR_ASYNC_SPARE
                            result= average;
                            break;

                    }

                    //reset average buffer
                    *pIntegr= 0;
                    *pCount= 0;

                    //export calculated value to interface
                    SInterface.asensors[sensor]= result;

                    //mark sensor as active, reset error counter
                    SInterface.asensors_health |= (1<< sensor);
                    SInternals.asensors_async_error_counter[sensor] =0;

                    //count the amount of consecutive valid readings, do not roll over
                    if(SInterface.asensors_valid_samples[sensor] < 0xFF)
                    {
                        SInterface.asensors_valid_samples[sensor]++;
                    }

                    //export raw value
                    SInterface.asensors_raw[sensor]= average;
                }

            }
            else
            {
                /**
                invalid reading, do error handling
                */

                if(SInternals.asensors_async_error_counter[sensor] < ASENSOR_ERROR_THRES)
                {
                    SInternals.asensors_async_error_counter[sensor]++;
                }
                else
                {
                    /**
                    sensor temporarily disturbed!
                    */

                    //report to interface
                    SInterface.asensors[sensor] =0;
                    SInterface.asensors_health &= ~(1<< sensor);

                    //reset average buffer
                    *pIntegr= 0;
                    *pCount= 0;

                    //delete raw value
                    SInterface.asensors_raw[sensor]= 0;

                    //no more consecutive valid readings
                    SInterface.asensors_valid_samples[sensor] =0;

                }

            }

        }

    }
}
