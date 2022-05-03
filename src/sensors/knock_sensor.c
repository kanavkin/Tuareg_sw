#include <Tuareg_platform.h>
#include <Tuareg.h>


#ifdef KNOCK_SENSOR_CODE_READY


volatile knock_sensor_controls_t Knock_sensor_ctrl;



/**
 * TPIC8101 parameters
 */
static const U32 Integrator_parameters[INTEGRATOR_PARAMETERS_SIZE] = {
    40, 45, 50, 55, 60, 65, 70, 75, 80, 90, 100, 110, 120, 130, 140, 150,
		160, 180, 200, 220, 240, 260, 280, 300, 320, 360, 400, 440, 480, 520, 560, 600
		};

static const F32 Gain_Parameters_reversed[GAIN_PARAMETERS_SIZE] = {

/* 00 */0.111, 0.118, 0.125, 0.129, 0.133, 0.138, 0.143, 0.148,
/* 08 */0.154, 0.160, 0.167, 0.174, 0.182, 0.190, 0.200, 0.211,
/* 16 */0.222, 0.236, 0.250, 0.258, 0.267, 0.276, 0.286, 0.296,
/* 24 */0.308, 0.320, 0.333, 0.348, 0.364, 0.381, 0.400, 0.421,
/* 32 */0.444, 0.471, 0.500, 0.548, 0.567, 0.586, 0.607, 0.630,
/* 40 */0.654, 0.680, 0.708, 0.739, 0.773, 0.810, 0.850, 0.895,
/* 48 */0.944, 1.000, 1.063, 1.143, 1.185, 1.231, 1.280, 1.333,
/* 56 */1.391, 1.455, 1.523, 1.600, 1.684, 1.778, 1.882, 2.0
};

static const F32 Band_parameters[BAND_PARAMETERS_SIZE] = {
    1.22, 1.26, 1.31, 1.35, 1.4, 1.45, 1.51, 1.57, 1.63, 1.71, 1.78,
		1.87, 1.96, 2.07, 2.18, 2.31, 2.46, 2.54, 2.62, 2.71, 2.81, 2.92, 3.03, 3.15, 3.28, 3.43, 3.59, 3.76, 3.95,
		4.16, 4.39, 4.66, 4.95, 5.12, 5.29, 5.48, 5.68, 5.9, 6.12, 6.37, 6.64, 6.94, 7.27, 7.63, 8.02, 8.46, 8.95, 9.5,
		10.12, 10.46, 10.83, 11.22, 11.65, 12.1, 12.6, 13.14, 13.72, 14.36, 15.07, 15.84, 16.71, 17.67, 18.76, 19.98
		};



/**
bring up the knock sensor interface

connections:

SCK     -> PB13 -> AF5: SPI2
MISO    -> PB14 -> AF5: SPI2
MOSI    -> PB15 -> AF5: SPI2
CS      -> PB12

INT/HLD -> PB11

analog out -> PA6

*/
void init_knock_sensor_interface()
{
    SPI_InitTypeDef  SPI_InitStructure;

    //clock for SPI and GPIO
    RCC->APB1ENR |=  RCC_AHB1ENR_GPIOBEN | RCC_APB1ENR_SPI2EN;

    //GPIO config
    GPIO_configure(GPIOB, 11, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_UP);
    GPIO_configure(GPIOB, 12, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);

    // SPI2 GPIO pins with AF 5
    GPIO_configure(GPIOB, 13, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 14, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 15, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_HIGH, GPIO_PULL_DOWN);

    GPIO_SetAF(GPIOB, 13, 5);
    GPIO_SetAF(GPIOB, 14, 5);
    GPIO_SetAF(GPIOB, 15, 5);


    // SPI configuration
    SPI_I2S_DeInit(SPI2);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    SPI_Init(SPI2, &SPI_InitStructure);

    //Enable USART1 global interrupt (prio 15)
    NVIC_SetPriority(SPI2_IRQn, 15UL);
    NVIC_EnableIRQ(SPI2_IRQn);

    //Enable the SPI peripheral
    SPI_Cmd(SPI2, ENABLE);

    //Enable the Rx buffer not empty interrupt
    SPI2->CR2 |= SPI_CR2_RXNEIE;

    //Enable the Tx buffer empty interrupt
    //SPI2->CR2 |= SPI_CR2_TXEIE;

}




void SPI2_SendData(U32 Data)
{
    select_Knock_CS();

    //Write in the DR register the data to be sent
    SPI2->DR= (U8) Data;

    deselect_Knock_CS();
}


U32 SPI2_ReadInputData()
{
    //Return the data in the DR register */
    return SPI2->DR;
}



void begin_Knock_integration()
{
    // INT/^HLD high
    gpio_set_pin(GPIOC, 11, PIN_ON);
}

void end_Knock_integration()
{
    // INT/^HLD low
    gpio_set_pin(GPIOC, 11, PIN_OFF);
}

void select_Knock_CS()
{
    // CS high
    gpio_set_pin(GPIOC, 12, PIN_ON);
}

void deselect_Knock_CS()
{
    // CS low
    gpio_set_pin(GPIOC, 12, PIN_OFF);
}




void update_knock_sensor_controls()
{
    update_knock_sensor_parameters();


    set_integration_time_constant(0);
    set_gain_value(0);










}


static const U32 cKnock_window_begin_aTDC= 10;
static const U32 cKnock_window_width_deg= 30;
static const F32 cKnock_target_V= 5.0;
static const F32 cKnock_signal_peak_V= 0.3;


void update_knock_sensor_parameters(knock_sensor_controls_t * pTarget)
{
    U32 interval_us, tau_target_us, tau_us, i;
    F32 a_target, a_param;

    //begin timing
    pTarget->integration_begin_timing_us= calc_rot_duration_us(Tuareg.ignition_controls.ignition_advance_deg + cKnock_window_begin_aTDC, Tuareg.pDecoder->crank_period_us );

    //integration interval
    interval_us= calc_rot_duration_us(cKnock_window_width_deg, Tuareg.pDecoder->crank_period_us );
    pTarget->integration_time_us= interval_us;

    /**
    integration time constant
    TC is typically TINT/(2*Pi*VOUT)
    */
    tau_target_us= (U32) divide_float((VF32) interval_us, 2*cPi*cKnock_target_V);

    //find a suitable integration time constant parameter
    for(i=0; i < INTEGRATOR_PARAMETERS_SIZE; i++)
    {
        tau_us= Integrator_parameters[i];

        if(tau_target_us <= tau_us)
        {
            break;
        }
    }

    //export parameters
    pTarget->tau_us= tau_us;
    pTarget->tau_index= i;

    /**
    signal gain
    A := ((V_target - 0.125) * pi * tau) / (signal_peak * 8 * tint)
    */
    a_target= divide_float(cPi * (F32) tau_us * (cKnock_target_V - 0.125), cKnock_signal_peak_V * 8 * (F32) interval_us );


    //find a suitable integration time constant parameter
    for(i=0; i < GAIN_PARAMETERS_SIZE; i++)
    {
        //Gain_Parameters_reversed holds the gain parameters in descending order
        a_param= Gain_Parameters_reversed[i];

        if(a_target > a_param)
        {
            break;
        }
    }


    //export parameters
    pTarget->gain= a_param;
    pTarget->gain_index= 63 - i;
}



VF32 calculate_knock_level(knock_sensor_controls_t * pControls, VU32 Readout)
{

    VF32 knock_level, m;

    /**
    v_out := V_in * A * 8/pi * t/tau + 0.125
    */
    m= divide_float((8 * pControls->gain * pControls->integration_time_us), (cPi * pControls->tau_us));

    knock_level= solve_linear(Readout, m, 0.125);

    return knock_level;
}





/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
    //data received
    if(SPI2->SR & SPI_SR_RXNE)
    {

    }


    /*
    if(SPI2->SR & SPI_SR_TXNE)
    {

    }
    */

}


#endif
