#ifndef STM32_GPIO_H_INCLUDED
#define STM32_GPIO_H_INCLUDED

#include "../boctok_types.h"



/**
how GPIO on STM32 looks like:
- GPIO ports A..G
- GPIO connected to APB2 bus
- 16 pins on each port
- register accessed in word mode (32bit)

- pins 0..7  controlled by GPIOx_CRL
- pins 8..15 controlled by GPIOx_CRH




*/

/**
remember to enable clock!
*/
#define GPIO_MODE_IN        0x00
#define GPIO_MODE_OUT       0x01
#define GPIO_MODE_AF        0x02
#define GPIO_MODE_ANALOG    0x03

#define GPIO_OUT_PP         0x00
#define GPIO_OUT_OD         0x01

#define GPIO_SPEED_LOW      0x00
#define GPIO_SPEED_MID      0x01
#define GPIO_SPEED_HIGH     0x02
#define GPIO_SPEED_VERY_HIGH 0x03

#define GPIO_PULL_NONE      0x00
#define GPIO_PULL_UP        0x01
#define GPIO_PULL_DOWN      0x02

/**
AFIO EXTI mapping
*/
#define EXTI_MAP_GPIOA  0x00
#define EXTI_MAP_GPIOB  0x01
#define EXTI_MAP_GPIOC  0x02
#define EXTI_MAP_GPIOH  0x07




/**
to set port pins
*/
typedef enum {

    PIN_ON,
    PIN_OFF,
    PIN_TOGGLE

} output_pin_t;



void GPIO_configure(GPIO_TypeDef * Port, U32 Pin, U32 Mode, U32 OutputType, U32 Speed, U32 Pud);
void SYSCFG_map_EXTI(U32 line, U32 port);
void gpio_set_pin(GPIO_TypeDef * Port, VU32 Pin, output_pin_t Level);
void GPIO_SetAF(GPIO_TypeDef* Port, U16 Pin, U8 Function);

#endif // STM32_GPIO_H_INCLUDED
