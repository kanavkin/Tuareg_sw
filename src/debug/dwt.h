#ifndef DWT_H_INCLUDED
#define DWT_H_INCLUDED

#include <Tuareg_platform.h>


/**
this module contains all the debug tool
that should never be integrated to a
production version
*/


void dwt_init();
extern void dwt_stop_cyclecounter();
extern U32 dwt_get_cyclecounter();
extern void dwt_set_begin();
extern void dwt_set_end();
void print_dwt_delay();
extern void poll_dwt_printout();
void delay_us(U32 delay);
U32 dwt_get_interval_us(U32 Reference);


#endif // DWT_H_INCLUDED
