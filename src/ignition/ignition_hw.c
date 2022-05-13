/**
this module covers the ignition hardware layer
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>

/******************************************************************************************************************************
ignition actuator control - helper functions
******************************************************************************************************************************/
void set_coil1_powered()
{
    //check vital preconditions
    if(Tuareg.errors.fatal_error == true)
    {
        return;
    }

    //ON
    gpio_set_pin(GPIOC, 6, PIN_ON);

    Tuareg.flags.ignition_coil_1= true;
}

void set_coil1_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 6, PIN_OFF);

    Tuareg.flags.ignition_coil_1= false;
}


void set_coil2_powered()
{
    //check vital preconditions
    if(Tuareg.errors.fatal_error == true)
    {
        return;
    }

    //ON
    gpio_set_pin(GPIOC, 7, PIN_ON);

    Tuareg.flags.ignition_coil_2= true;
}

void set_coil2_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 7, PIN_OFF);

    Tuareg.flags.ignition_coil_2= false;
}


/******************************************************************************************************************************
ignition irq control - helper function
void trigger_ignition_irq()
{
    ///trigger sw irq
    EXTI->SWIER= EXTI_SWIER_SWIER3;
}
******************************************************************************************************************************/



/******************************************************************************************************************************
ignition actuator control

a spark will be generated when switching from powered to unpowered state
high speed log will be updated on level change

 ******************************************************************************************************************************/
void set_ignition_ch1(actor_control_t level)
{
    bool old_state= Tuareg.flags.ignition_coil_1;

    if(level == ACTOR_POWERED)
    {
        set_coil1_powered();

        if(old_state == false)
        {
            highspeedlog_register_coil1_power();
        }
    }
    else
    {
        set_coil1_unpowered();

        if(old_state == true)
        {
            highspeedlog_register_coil1_unpower();
        }
    }
}


void set_ignition_ch2(actor_control_t level)
{
    bool old_state= Tuareg.flags.ignition_coil_2;

    if(level == ACTOR_POWERED)
    {
        set_coil2_powered();

        if(old_state == false)
        {
            highspeedlog_register_coil2_power();
        }
    }
    else
    {
        set_coil2_unpowered();

        if(old_state == true)
        {
            highspeedlog_register_coil2_unpower();
        }
    }
}



/******************************************************************************************************************************
ignition hardware initialization

using
    -GPIOC6 for ignition coil 1
    -GPIOC7 for ignition coil 2

    -do not use EXTI IRQ 3 after spark has fired

 ******************************************************************************************************************************/
void init_ignition_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 6, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 7, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
/*
    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);


*/
}
