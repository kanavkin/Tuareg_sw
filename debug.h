#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "ignition_logic.h"


extern VU32 Debug_DWT_Timestamp_1;
extern VU32 Debug_DWT_Timestamp_2;
extern VU32 Debug_DWT_End;

void init_debug_pins();
void set_debug_pin( output_pin_t level);
void set_debug_led( output_pin_t level);

void print_full_state(volatile ignition_timing_t * intime);
void print_minimal_state(USART_TypeDef * Port, volatile ignition_timing_t * intime);


void dwt_init();
extern void dwt_stop_cyclecounter();

extern U32 dwt_get_cyclecounter();
extern void dwt_set_begin();
extern void dwt_set_end();
void print_dwt_delay();
extern void poll_dwt_printout();

void delay_us(U32 delay);

void print_sensor_data();
void print_decoder_statistics();

#endif // DEBUG_H_INCLUDED
