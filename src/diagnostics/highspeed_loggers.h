#ifndef HSPDLOG_H_INCLUDED
#define HSPDLOG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_ID.h"


#define HIGSPEEDLOG_LENGTH 6





typedef struct __attribute__ ((__packed__)) _highspeedlog_message_t {

    U32 timestamp;



} highspeedlog_message_t;





typedef struct _highspeedlog_flags_t {

    U8 syslog_new_entry :1;


} highspeedlog_flags_t;



typedef struct _highspeedlog_mgr_t {

    U32 D_ptr;

    syslog_mgr_flags_t flags;

} highspeedlog_mgr_t;


void highspeedlog_init();

void highspeedlog_register_error();

void highspeedlog_register_crankpos(volatile crank_position_t Position, VU32 timestamp);

void highspeedlog_register_cis_lobe_begin();
void highspeedlog_register_cis_lobe_end();

void highspeedlog_register_coil1_power();
void highspeedlog_register_coil1_unpower();

void highspeedlog_register_coil2_power();
void highspeedlog_register_coil2_unpower();

void highspeedlog_register_injector1_power();
void highspeedlog_register_injector1_unpower();

void highspeedlog_register_injector2_power();
void highspeedlog_register_injector2_unpower();








#endif // HSPDLOG_H_INCLUDED
