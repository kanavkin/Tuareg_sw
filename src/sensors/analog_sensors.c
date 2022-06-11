#include <math.h>

#include <Tuareg_platform.h>
#include <Tuareg.h>


/**
use the lookup table for CLT sensor
*/
#define CLT_LOOKUP

/// TODO (oli#1#): check data width (16 bit adc transfers vs. 32 bit readout)
VU16 DMA_Buffer[LAST_ASYNC_ASENSOR];


volatile asensor_data_t Analog_Sensors[ASENSOR_COUNT];


/**
an analog sensor shall be considered as temporarily disturbed, when the following amount of consecutive invalid samples have been detected

with the configured update rate (currently 100 Hz) a timeout of 3 second results
*/
const U32 cASensorErrorThres= 300;


const U32 cASensorInitCycles= 10;



VU32 Regular_Group_Init_counter= 0;
VU32 Injected_Group_Init_counter= 0;



/**
bring up analog and digital sensors
*/
void init_analog_sensors()
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
    adc_set_regular_group_length(ADC1, LAST_ASYNC_ASENSOR);

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
    DMA_StructInit(&DMA_InitStructure);

    //DMA_InitStructure.DMA_Channel= DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr= (U32)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr= (U32)DMA_Buffer;
    //DMA_InitStructure.DMA_DIR= DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize= LAST_ASYNC_ASENSOR;
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


    for(channel =0; channel < ASENSOR_COUNT; channel++)
    {
        Analog_Sensors[channel].valid_count= ASENSOR_VALIDITY_THRES;
        Analog_Sensors[channel].invalid_count= 0;
    }

    Injected_Group_Init_counter= 0;
    sensors_start_injected_group_conversion();

    while(Injected_Group_Init_counter <= cASensorInitCycles)
    {
        __asm volatile ("nop");
    }

    Regular_Group_Init_counter= 0;
    sensors_start_regular_group_conversion();

    while(Regular_Group_Init_counter <= cASensorInitCycles)
    {
        __asm volatile ("nop");
    }

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
this function can be used to reset the captured data from synchronous sensors e.g. in case of a sync lost event
*/
void reset_integrator(volatile asensor_data_t * pSensorData)
{
    pSensorData->integrator= 0;
    pSensorData->integrator_count= 0;
}


/*************************************************************************************************************************************************
helper function - analog sensor readout validation and conversion, health status update
*************************************************************************************************************************************************/
void update_analog_sensor(asensors_t Sensor, U32 Sample, bool ForceProcessing)
{
    VU32 average;
    VF32 result;
    volatile asensor_data_t * const pSensorData= &(Analog_Sensors[Sensor]);
    volatile asensor_parameters_t * const pSensorParameters= &(Sensor_Calibration.Asensor_Parameters[Sensor]);


    //export raw sample value for diagnostic purposes
    pSensorData->raw= Sample;


    //check if the input sample is invalid
    if((Sample < pSensorParameters->min_valid) || (Sample > pSensorParameters->max_valid))
    {
        //clean up
        reset_integrator(pSensorData);

        //update health statistics, value cannot roll over within a realistic up time
        pSensorData->invalid_count += 1;

        //check if the error threshold has been reached
        if(pSensorData->invalid_count > cASensorErrorThres)
        {
            //sensor disturbed, no output data available
            pSensorData->out= 0;
            pSensorData->valid_count= 0;
        }

        //no input data to process
        return;
    }

    //update integrator
    pSensorData->integrator += Sample;
    pSensorData->integrator_count += 1;


    //check if samples has to be processed
    if((pSensorData->integrator_count < pSensorParameters->target_sample_length) && (ForceProcessing == false))
    {
        //no input data to process
        return;
    }

    //calculate average
    average= divide_U32(pSensorData->integrator, pSensorData->integrator_count);

    //clean up
    reset_integrator(pSensorData);


    #ifdef CLT_LOOKUP
    if(Sensor == ASENSOR_CLT)
    {
        result= getValue_InvTableCLT(average);
    }
    else
    {
        //calculate the physical input value
        result= solve_linear(average, pSensorParameters->M, pSensorParameters->N);
    }
    #else
        //calculate the physical input value
        result= solve_linear(average, pSensorParameters->M, pSensorParameters->N);
    #endif // CLT_LOOKUP

    //export outputs
    pSensorData->out= result;

    //update health statistics
    if(pSensorData->valid_count < cU32max)
    {
        pSensorData->valid_count += 1;
    }

    pSensorData->invalid_count= 0;
}



/**
MAP sensor readout as injected conversion
(intended to be triggered synchronous to crank movement)
*/
void ADC_IRQHandler()
{
    VU32 sample;

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

        //check precondition - sensor calibration available
        if(Tuareg.errors.sensor_calibration_error == true)
        {
            //error
/// TODO (oli#1#): handle this case: LIMP
            return;
        }

        if(Injected_Group_Init_counter > cASensorInitCycles)
        {
            //update MAP sensor
            update_analog_sensor(ASENSOR_MAP, sample, false);
        }
        else
        {
            //update MAP sensor without averaging
            update_analog_sensor(ASENSOR_MAP, sample, true);

            Injected_Group_Init_counter++;

            //start the next conversion right now
            sensors_start_injected_group_conversion();
        }
    }
}


/**
The regular group conversion is triggered by systick timer every 10 ms (100Hz)
the effective update interval for each sensor depends on its configured target sample length (n * 10 ms)
*/
void DMA2_Stream0_IRQHandler()
{
    VU32 sensor;

    //collect diagnostic data
    sensors_diag_log_event(SNDIAG_DMAIRQ_CALLS);

    //DMA1 Channel1 Transfer Complete interrupt
    if(DMA2->LISR & DMA_LISR_TCIF0)
    {
        //collect diagnostic data
        sensors_diag_log_event(SNDIAG_DMAIRQ_CH1_CALLS);

        //Clear all DMA2 stream0 interrupt pending bits
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

        //check precondition - sensor calibration available
        if(Tuareg.errors.sensor_calibration_error == true)
        {
            //error
            /// TODO (oli#1#): handle this case?
            return;
        }

        if(Regular_Group_Init_counter > cASensorInitCycles)
        {
            //update every async asensor
            for(sensor= 0; sensor < LAST_ASYNC_ASENSOR +1; sensor++)
            {
                //run update function
                update_analog_sensor(sensor, DMA_Buffer[sensor], false);
            }
        }
        else
        {
            //update every async asensor without averaging
            for(sensor= 0; sensor < LAST_ASYNC_ASENSOR +1; sensor++)
            {
                //run update function
                update_analog_sensor(sensor, DMA_Buffer[sensor], true);
            }

            Regular_Group_Init_counter++;

            //start the next conversion right now
            sensors_start_regular_group_conversion();
        }





    }
}
