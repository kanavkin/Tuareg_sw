#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "ignition_logic.h"
/*
typedef struct
{
  __IO uint32_t CTRL;                        // Offset: 0x00  Control Register
  __IO uint32_t CYCCNT;                        // Offset: 0x04  Cycle counter Register

} SWT_Type;

 Core Debug registers (defined in core header)
#define DWT_BASE    (0xE0001000)
#define DWT         ((SWT_Type *) DWT_BASE)
*/


void init_debug_pins();
void set_debug_pin( output_pin_t level);
void set_debug_led( output_pin_t level);

void print_full_state(volatile ignition_timing_t * intime);
void print_minimal_state(USART_TypeDef * Port, volatile ignition_timing_t * intime);


void dwt_init();
void delay_us(U32 delay);

void print_sensor_data();


#endif // DEBUG_H_INCLUDED
