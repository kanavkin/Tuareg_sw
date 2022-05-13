/**
this module covers the dash elements HAL
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>
#include "dash_hw.h"


/******************************************************************************************************************************
dash actuator control
 ******************************************************************************************************************************/
void set_tachometer_hw(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        gpio_set_pin(GPIOC, 11, PIN_ON);

        Syslog_Info(TID_DASH_HW, 42);

    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 11, PIN_OFF);

        Syslog_Info(TID_DASH_HW, 41);
    }
}

/*
Open drain mode: A “0” in the Output register activates the N-MOS whereas a “1”
in the Output register leaves the port in Hi-Z (the P-MOS is never activated)
*/
void set_mil_hw(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        gpio_set_pin(GPIOC, 12, PIN_ON);
        Tuareg.flags.mil= true;
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 12, PIN_OFF);
        Tuareg.flags.mil= false;
    }
}



/**
    using
    -GPIOC11 for tachometer
    -GPIOC11 for user dash lamp (mil)
    connected to VNLD5090 low side driver
    with open drain control input
*/
void init_dash_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    #ifdef HIL_HW
    //active output for HIL
    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 12, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    #else
    //open drain output to driver IC
    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 12, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    #endif // HIL_HW

    set_tachometer_hw(ACTOR_UNPOWERED);
    set_mil_hw(ACTOR_UNPOWERED);
}
