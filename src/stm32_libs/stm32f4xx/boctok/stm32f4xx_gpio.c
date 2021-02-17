#include "stm32f4xx.h"
#include "../boctok_types.h"
#include "stm32f4xx_gpio.h"


void GPIO_configure(GPIO_TypeDef * Port, U32 Pin, U32 Mode, U32 OutputType, U32 Speed, U32 Pud)
{
    //GPIO port pin range 0..15!
    if(Pin > 15)
    {
        return;
        //TODO log warning
    }

    //Mode
    Port->MODER &= (U32) ~(0x03 << (Pin * 2));

    if(Mode)
    {
        Port->MODER |= (U32) (Mode << (Pin * 2));
    }


    //OutputType
    Port->OTYPER &= (U16) ~(0x01 << Pin);

    if(OutputType)
    {
        Port->OTYPER |= (U16) (OutputType << Pin);
    }


    //Speed
    Port->OSPEEDR &= (U32) ~(0x03 << (Pin * 2));

    if(Speed)
    {
        Port->OSPEEDR |= (U32) (Speed << (Pin * 2));
    }


    //Pullup Pulldown
    Port->PUPDR &= (U32) ~(0x03 << (Pin * 2));

    if(Pud)
    {
        Port->PUPDR |= (U32) (Pud << (Pin * 2));
    }

}


/**
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  Port : selects the GPIO port to be used as source for this line
  *
  * @param  Line: specifies the EXTI line to be configured.
  *           GPIOx-1 maps to line 1
  *           GPIOx-2 maps to line 2
  *           and so on
  * @retval None
  */
void SYSCFG_map_EXTI(U32 Line, U32 Port)
{
    if(Line < 4)
    {
        SYSCFG->EXTICR[0] &= (U32) ~(0x0F << (Line * 4));
        SYSCFG->EXTICR[0] |= (U32)(Port << (Line * 4));
    }
    else if(Line < 8)
    {
        SYSCFG->EXTICR[1] &= (U32) ~(0x0F << ((Line -4) * 4));
        SYSCFG->EXTICR[1] |= (U32)(Port << ((Line -4) * 4));
    }
    else if(Line < 12)
    {
        SYSCFG->EXTICR[2] &= (U32) ~(0x0F << ((Line -8) * 4));
        SYSCFG->EXTICR[2] |= (U32)(Port << ((Line -8) * 4));
    }
    else if(Line < 16)
    {
        SYSCFG->EXTICR[3] &= (U32) ~(0x0F << ((Line -12) * 4));
        SYSCFG->EXTICR[3] |= (U32)(Port << ((Line -12) * 4));
    }
}


/**
helper functions
*/
void gpio_set_pin_high(GPIO_TypeDef * Port, VU32 Pin)
{
    //on
    Port->BSRR= (U32) (1<< Pin);
}

void gpio_set_pin_low(GPIO_TypeDef * Port, VU32 Pin)
{
    //off
    Port->BSRR= (U32) (1<< (Pin + 16));
}




/**
convenient output pin modification
*/
void gpio_set_pin(GPIO_TypeDef * Port, VU32 Pin, output_pin_t Level)
{
    //GPIO port pin range 0..15!
    if(Pin > 15)
    {
        return;
        //TODO log warning
    }

    if(Level == PIN_ON)
    {
        gpio_set_pin_high(Port, Pin);
    }
    else if(Level == PIN_TOGGLE)
    {
        if(Port->ODR & (U32) (1<< Pin))
        {
            // OFF
            gpio_set_pin_low(Port, Pin);
        }
        else
        {
            //on
            gpio_set_pin_high(Port, Pin);
        }
    }
    else
    {
        gpio_set_pin_low(Port, Pin);
    }
}


/**
  * @brief  Changes the AF mapping of the specified pin.
  * @param  Port
  * @param  Pin: specifies the pin for the Alternate function where x can be (0..15).
  * @param  Function: sets the AF number (0..15)

  * @retval None
  */
void GPIO_SetAF(GPIO_TypeDef* Port, U16 Pin, U8 Function)
{
    //GPIO port pin range 0..15!
    if(Pin > 15)
    {
        return;
        //TODO log warning
    }

    //AF range 0..15!
    if(Function > 15)
    {
        return;
        //TODO log warning
    }


    if(Pin < 8)
    {
        Port->AFR[0] &= (U32) ~(0x0F << (Pin * 4));
        Port->AFR[0] |= (U32)(Function << (Pin * 4));
    }
    else
    {
        Port->AFR[1] &= (U32) ~(0x0F << ((Pin -8) * 4));
        Port->AFR[1] |= (U32)(Function << ((Pin -8) * 4));
    }

}
