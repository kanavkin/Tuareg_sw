#ifndef SYSLOG_H_INCLUDED
#define SYSLOG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_ID.h"

#define USE_SYSLOG

#define SYSLOG_LOC_BIT_E 7
#define SYSLOG_LOC_BIT_W 6

#define SYSLOG_LENGTH 30
#define DATALOG_LENGTH 10



typedef struct __attribute__ ((__packed__)) _syslog_message_t {

    U32 timestamp;
    U8 location;
    Tuareg_ID src;

} syslog_message_t;


typedef union
{
    F32 as_float[4];
    U32 as_U32[4];
    U16 as_U16[8];
    U8  as_U8[24];

} syslog_data128_t;


typedef struct __attribute__ ((__packed__)) _syslog_datagram_t {

    U32 timestamp;
    U8 location;
    Tuareg_ID src;

    syslog_data128_t data;

} syslog_datagram_t;


typedef struct _syslog_mgr_t {

    U32 Msg_E_ptr;
    U32 Msg_ptr;

    U32 D_ptr;

} syslog_mgr_t;


#endif // SYSLOG_H_INCLUDED
