#ifndef DECODER_DEBUG_H_INCLUDED
#define DECODER_DEBUG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

void decoder_process_debug_events();
void register_sync_lost_debug_event();
void register_got_sync_debug_event();
void register_timer_overflow_debug_event();
void register_timeout_debug_event();

#endif
