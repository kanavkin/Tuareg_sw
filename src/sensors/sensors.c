#include <math.h>

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "sensors.h"
#include "sensor_calibration.h"

#include "Tuareg.h"

#include "diagnostics.h"


volatile sensor_interface_t SInterface;
volatile sensor_internals_t SInternals;



/**
bring up analog and digital sensors
*/
volatile sensor_interface_t * init_sensor_inputs(U32 Init_count)
{
    DMA_InitTypeDef DMA_InitStructure;
    VU32 channel;

    //set ADC pre scaler to 2 (source: APB2/PCLK2 @ 50 MHz) -> 25 MHz
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
    adc_set_sample_time(ADC1, ADC_GEAR_CH, SAMPLE_TIME_7_5);

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
    adc_set_regular_group(ADC1, 8, ADC_GEAR_CH);

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

    /**
    fast init feature:

    initialize the valid sample count with its fast init value "init_count"
    this makes sensor data available earlier in case of engine startup
    */
    for(channel =0; channel < ASENSOR_COUNT; channel++)
    {
        SInterface.asensors_valid_samples[channel]= Init_count;
    }

    return &SInterface;
}


void sensors_start_regular_group_conversion()
{
    adc_start_regular_group(SENSOR_ADC);
}

void sensors_start_injected_group_conversion()
{
    adc_start_injected_group(SENSOR_ADC);
}



/**
throttle transient calculation

[ddt_TPS] := °/s

implements exponential moving average with alpha [0 .. 1]


This calculation has been tailored for a 10 ms TPS sample interval:
ddt_TPS := 1000 * delta_TPS / TPS_sample_interval_ms

*/
VF32 calculate_ddt_TPS(VF32 TPS, VF32 Last_TPS, VF32 Last_ddt_TPS)
{
    VF32 ddt_TPS, ema_ddt_TPS;

    const F32 TPS_sample_interval_ms= 10;
    const F32 alpha= 0.85;

    //calculate the current TPS change rate in °/s
    ddt_TPS= (1000.0 / TPS_sample_interval_ms) * (TPS - Last_TPS);

    // EMA: y[n]= y[n−1] * (1−α) + x[n] * α
    ema_ddt_TPS= (1 - alpha) * Last_ddt_TPS + alpha * ddt_TPS;

    return ema_ddt_TPS;
}

/**
intake pressure transient calculation

[ddt_MAP] := kPa/s

implements exponential moving average with alpha [0 .. 1]


This calculation has been tailored for an interval in us:
ddt_MAP := (1000000 / Interval_us) * delta_MAP

*/
VF32 calculate_ddt_MAP(VF32 MAP_kPa, VF32 Last_MAP_kPa, VF32 Last_ddt_MAP, VU32 Interval_us)
{
    VF32 ddt_MAP, ema_ddt_MAP;

    const F32 alpha= 0.85;

    //calculate the current MAP change rate in kPa/s
    ddt_MAP= divide_VF32(1000000, Interval_us) * (MAP_kPa - Last_MAP_kPa);

    // EMA: y[n]= y[n−1] * (1−α) + x[n] * α
    ema_ddt_MAP= (1 - alpha) * Last_ddt_MAP + alpha * ddt_MAP;

    return ema_ddt_MAP;
}


/**
smoothed out intake pressure

[avg_MAP] := kPa

implements exponential moving average with alpha [0 .. 1]
*/
VF32 calculate_average_MAP(VF32 MAP_kPa, VF32 Last_avg_MAP_kPa)
{
    VF32 ema_MAP_kPa;

    // EMA: y[n]= y[n−1] * (1−α) + x[n] * α
    ema_MAP_kPa= (1 - Sensor_Calibration.MAP_filter_alpha) * Last_avg_MAP_kPa + Sensor_Calibration.MAP_filter_alpha * MAP_kPa;

    return ema_MAP_kPa;
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

    //DSENSOR_SIDESTAND
    if(GPIOC->IDR & GPIO_IDR_IDR0)
    {
        readout |= (1 << DSENSOR_SIDESTAND);
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
this function parses the raw digital input data from DSENSORS (hardware independent part)
*/
void read_digital_sensors()
{
    U32 cnt, level, sensor =0;

    //collect diagnostic data
    sensors_diag_log_event(SNDIAG_READ_DSENSORS_CALLS);

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
(intended to be triggered synchronous to crank movement)
*/
void ADC_IRQHandler()
{
    VU32 average, sample;
    VU32 * pIntegr= NULL;
    VU32 * pCount= NULL;
    VF32 MAP_kPa, avg_MAP_kPa, ddt_MAP;

    //collect diagnostic data
    sensors_diag_log_event(SNDIAG_ADCIRQ_CALLS);

    //MAP sensor handled by injected group
    if(ADC1->SR & ADC_SR_JEOC)
    {
        //collect diagnostic data
        sensors_diag_log_event(SNDIAG_ADCIRQ_INJECTEDGR_CALLS);

        //clear JEOC by write 0
        ADC1->SR &= ~(U32) ADC_SR_JEOC;

        //read ADC value
        sample= ADC1->JDR1;

        //access to sensor data
        pIntegr= &(SInternals.asensors_sync_integrator[ASENSOR_SYNC_MAP]);
        pCount= &(SInternals.asensors_sync_integrator_count[ASENSOR_SYNC_MAP]);

        //validate sample
        if( (sample >= Sensor_Calibration.MAP_min_valid) && (sample <= Sensor_Calibration.MAP_max_valid) && (Tuareg.errors.sensor_calibration_error == false))
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

                //critical section
                __disable_irq();

                //calculate the average map value
                average= *pIntegr / *pCount;

                //calculate new MAP value
                MAP_kPa= solve_linear(average, Sensor_Calibration.MAP_calib_M, Sensor_Calibration.MAP_calib_N);

                /**
                MAP average and derivative calculation

                on transition from sensor error state
                - the map average value shall be initialised with a sane value
                - ddt MAP shall be zero

                */
                if(SInterface.asensors_valid_samples[ASENSOR_MAP] == 0)
                {
                    //the first captured, valid sample -> no calculation
                    avg_MAP_kPa= MAP_kPa;
                    ddt_MAP= 0;
                }
                else
                {
                    //last average MAP and ddt MAP values should be valid

                    //valculate average MAP
                    avg_MAP_kPa= calculate_average_MAP(MAP_kPa, SInternals.last_avg_MAP_kPa);

                    //calculate ddt_MAP based on the average pressure
                    if(Tuareg.pDecoder->outputs.period_valid == true)
                    {
                        //the interval given to calculate_ddt_MAP should reflect the actual sample interval (T720 if sampling 2 crank turns)
                        ddt_MAP= calculate_ddt_MAP(MAP_kPa, avg_MAP_kPa, SInternals.last_ddt_MAP, ASENSOR_SYNC_SAMPLE_CRK_REVS * Tuareg.pDecoder->crank_period_us);
                    }
                    else
                    {
                        ddt_MAP= 0;
                    }
                }


                //save MAP_kPa, avg_MAP_kPa, ddt_MAP values from the current cycle to be new old values in the next cycle
                SInternals.last_avg_MAP_kPa= avg_MAP_kPa;
                SInternals.last_ddt_MAP= ddt_MAP;

                //export to interface
                SInterface.asensors[ASENSOR_MAP]= MAP_kPa;
                SInterface.asensors_raw[ASENSOR_MAP]= average;
                SInterface.ddt_MAP= ddt_MAP;
                SInterface.avg_MAP_kPa= avg_MAP_kPa;

                //reset average buffer
                *pIntegr= 0;
                *pCount= 0;

                //notify valid readout
                SInternals.asensors_sync_error_counter[ASENSOR_SYNC_MAP] =0;

                //count the amount of consecutive valid readings, do not roll over
                if(SInterface.asensors_valid_samples[ASENSOR_MAP] < 0xFF)
                {
                    SInterface.asensors_valid_samples[ASENSOR_MAP]++;
                }

                //ready
                __enable_irq();

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
                SInterface.asensors[ASENSOR_MAP]= 0.0;
                SInterface.asensors_raw[ASENSOR_MAP]= 0;
                SInterface.ddt_MAP= 0.0;
                SInternals.last_ddt_MAP= 0.0;
                SInternals.last_avg_MAP_kPa= 0.0;

                //reset average buffer
                *pIntegr= 0;
                *pCount= 0;

                //no more consecutive valid readings
                SInterface.asensors_valid_samples[ASENSOR_MAP] =0;

            }

        }

    }

}


/**
The regular group conversion is triggered by systick timer every 10 ms (100Hz)
with 5x oversampling this gives an update interval of 50 ms
*/
void DMA2_Stream0_IRQHandler()
{

    VU32 average, sample, sensor, min_valid, max_valid, target_sample_len;
    VF32 result, calc;
    VU32 * pIntegr= NULL;
    VU32 * pCount= NULL;
    bool process_data_now= false;

    //collect diagnostic data
    sensors_diag_log_event(SNDIAG_DMAIRQ_CALLS);

    //DMA1 Channel1 Transfer Complete interrupt
    if(DMA2->LISR & DMA_LISR_TCIF0)
    {
        //collect diagnostic data
        sensors_diag_log_event(SNDIAG_DMAIRQ_CH1_CALLS);

        //Clear all DMA2 stream0 interrupt pending bits
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

        /**
        update frequency shall be equal for every asensor_async -> every loop
        */

        //for every async asensor
        for(sensor =0; sensor < ASENSOR_ASYNC_COUNT; sensor++)
        {
            process_data_now= false;

            //read ADC input value
            sample= SInternals.asensors_async_buffer[sensor];

            //get easy access to sensor data
            pIntegr= &(SInternals.asensors_async_integrator[sensor]);
            pCount= &(SInternals.asensors_async_integrator_count[sensor]);


            /**
            get the individual sensor readout validity thresholds and target sample length
            */
            switch(sensor)
            {
                case ASENSOR_ASYNC_O2:

                    min_valid= Sensor_Calibration.O2_min_valid;
                    max_valid= Sensor_Calibration.O2_max_valid;
                    target_sample_len= 0;
                    break;

                case ASENSOR_ASYNC_TPS:

                    min_valid= Sensor_Calibration.TPS_min_valid;
                    max_valid= Sensor_Calibration.TPS_max_valid;
                    target_sample_len= 0;
                    break;

                case ASENSOR_ASYNC_IAT:

                    min_valid= Sensor_Calibration.IAT_min_valid;
                    max_valid= Sensor_Calibration.IAT_max_valid;
                    target_sample_len= 50;
                    break;

                case ASENSOR_ASYNC_CLT:

                    min_valid= Sensor_Calibration.CLT_min_valid;
                    max_valid= Sensor_Calibration.CLT_max_valid;
                    target_sample_len= 50;
                    break;

                case ASENSOR_ASYNC_VBAT:

                    min_valid= Sensor_Calibration.VBAT_min_valid;
                    max_valid= Sensor_Calibration.VBAT_max_valid;
                    target_sample_len= 10;
                    break;

                case ASENSOR_ASYNC_KNOCK:

                    min_valid= Sensor_Calibration.KNOCK_min_valid;
                    max_valid= Sensor_Calibration.KNOCK_max_valid;
                    target_sample_len= 0;
                    break;

                case ASENSOR_ASYNC_BARO:

                    min_valid= Sensor_Calibration.BARO_min_valid;
                    max_valid= Sensor_Calibration.BARO_max_valid;
                    target_sample_len= 50;
                    break;

                case ASENSOR_ASYNC_GEAR:

                    min_valid= Sensor_Calibration.GEAR_min_valid;
                    max_valid= Sensor_Calibration.GEAR_max_valid;
                    target_sample_len= 50;
                    break;

                default:

                    min_valid= 0;
                    max_valid= 0;
                    target_sample_len= 0;
                    break;

            }

            //check if the read value is valid (validate the sensor channel with its individual threshold, if the threshold itself is valid)
            if( (sample >= min_valid) && (sample <= max_valid) && (Tuareg.errors.sensor_calibration_error == false) )
            {
                /**
                valid reading
                */

                //count the amount of consecutive valid readings, do not roll over
                if(SInterface.asensors_valid_samples[sensor] < 0xFF)
                {
                    SInterface.asensors_valid_samples[sensor]++;
                }


                //check if averaging is enabled for this channel
                if(target_sample_len > 1)
                {
                    //store to integrator
                    *pIntegr += sample;
                    *pCount += 1;

                    //check if enough samples have been collected
                    if(*pCount >= target_sample_len)
                    {
                        average= *pIntegr / *pCount;

                        //reset average buffer
                        *pIntegr= 0;
                        *pCount= 0;

                        //ready to process data
                        process_data_now= true;
                    }
                }
                else
                {
                    //no averaging
                    average= sample;
                    process_data_now= true;
                }


                /*******************************************************************
                ready?
                *******************************************************************/
                if(process_data_now == true)
                {
                    //critical section
                    __disable_irq();

                    /**
                    ADC value to process data conversion
                    */
                    switch(sensor)
                    {
                        case ASENSOR_ASYNC_O2:

                            result= solve_linear(average, Sensor_Calibration.O2_calib_M, Sensor_Calibration.O2_calib_N);
                            break;

                        case ASENSOR_ASYNC_TPS:

                            result= solve_linear(average, Sensor_Calibration.TPS_calib_M, Sensor_Calibration.TPS_calib_N);

                            //throttle transient calculation
                            calc= calculate_ddt_TPS(result, SInternals.last_TPS, SInternals.last_ddt_TPS);

                            //save old TPS / ddt_TPS values
                            SInternals.last_TPS= result;
                            SInternals.last_ddt_TPS= calc;

                            //export ddt_TPS
                            SInterface.ddt_TPS= calc;

                            break;

                        case ASENSOR_ASYNC_IAT:

                            result= solve_linear(average, Sensor_Calibration.IAT_calib_M, Sensor_Calibration.IAT_calib_N);
                            result += cKelvin_offset;
                            break;

                        case ASENSOR_ASYNC_CLT:

                            result= solve_linear(average, Sensor_Calibration.CLT_calib_M, Sensor_Calibration.CLT_calib_N);
                            result += cKelvin_offset;
                            break;

                        case ASENSOR_ASYNC_VBAT:

                            result= solve_linear(average, Sensor_Calibration.VBAT_calib_M, Sensor_Calibration.VBAT_calib_N);
                            break;

                        case ASENSOR_ASYNC_KNOCK:

                            result= solve_linear(average, Sensor_Calibration.KNOCK_calib_M, Sensor_Calibration.KNOCK_calib_N);
                            break;

                        case ASENSOR_ASYNC_BARO:

                            result= solve_linear(average, Sensor_Calibration.BARO_calib_M, Sensor_Calibration.BARO_calib_N);
                            break;

                        case ASENSOR_ASYNC_GEAR:

                            result= solve_linear(average, Sensor_Calibration.GEAR_calib_M, Sensor_Calibration.GEAR_calib_N);
                            break;

                        default:

                            //e.g. ASENSOR_ASYNC_SPARE
                            result= average;
                            break;

                    }


                    /*******************************************************************
                    export to interface
                    *******************************************************************/

                    SInterface.asensors[sensor]= result;
                    SInterface.asensors_raw[sensor]= average;

                    //mark sensor as active, reset error counter
                    SInternals.asensors_async_error_counter[sensor] =0;

                    //ready
                    __enable_irq();

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

                    //delete raw value
                    SInterface.asensors_raw[sensor]= 0;

                    //ddt_TPS calculation depends on TPS
                    if(sensor == ASENSOR_ASYNC_TPS)
                    {
                        SInterface.ddt_TPS= 0;
                        SInternals.last_TPS= 0;
                        SInternals.last_ddt_TPS= 0;
                    }

                    //reset average buffer
                    *pIntegr= 0;
                    *pCount= 0;

                    //no more consecutive valid readings
                    SInterface.asensors_valid_samples[sensor] =0;

                }

            }

        }

    }
}
