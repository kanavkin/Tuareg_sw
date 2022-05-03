#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include <Tuareg_platform.h>


void set_debug_led(output_pin_t level);
void set_debug_pin(output_pin_t level);
void init_debug_pins();
void enable_sysclk_check();

void SWO_Init(U32 portBits, U32 cpuCoreFreqHz);
void SWO_PrintString(const char *s, U8 portNumber);


#endif // DEBUG_H_INCLUDED
