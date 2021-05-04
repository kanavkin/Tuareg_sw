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


#include "knock_sensor.h"

volatile knock_sensor_controls_t Knock_sensor_ctrl;

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
    update_integration_time_constant();
    update_gain();


    set_integration_time_constant(0);
    set_gain_value(0);

}




void update_integration_time_constant()
{





}



void set_integration_time_constant(U32 Index)
{
    VF32 time_constant_us =0;




    Knock_sensor_ctrl.integration_time_constant_us= time_constant_us;

}



void update_gain()
{




}


void set_gain_value(U32 Index)
{
    VF32 gain =0;



    Knock_sensor_ctrl.gain= gain;

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


