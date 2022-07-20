#ifndef SYSLOG_H_INCLUDED
#define SYSLOG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_ID.h"

#define USE_SYSLOG

#define SYSLOG_LOC_BIT_E 7
#define SYSLOG_LOC_BIT_W 6

#define SYSLOG_DATALOG_BIT_D 7

#define SYSLOG_LENGTH 30
#define DATALOG_LENGTH 10

#define DATALOG_PAYLOAD_LEN_INT 4

typedef struct __attribute__ ((__packed__)) _syslog_message_t {

    U32 timestamp;
    U8 location;
    Tuareg_ID src;
    U8 datalog;

} syslog_message_t;


typedef union __attribute__ ((__packed__)) _datalog_entry_t {

    F32 as_float[DATALOG_PAYLOAD_LEN_INT];
    U32 as_U32[DATALOG_PAYLOAD_LEN_INT];
    U16 as_U16[2* DATALOG_PAYLOAD_LEN_INT];
    U8  as_U8[4* DATALOG_PAYLOAD_LEN_INT];

} datalog_entry_t;



typedef struct _syslog_mgr_flags_t {

    U8 syslog_new_entry :1;
    U8 datalog_new_entry :1;

} syslog_mgr_flags_t;



typedef struct _syslog_mgr_t {

    U32 Msg_E_ptr;
    U32 Msg_ptr;

    U32 D_ptr;

    syslog_mgr_flags_t flags;

} syslog_mgr_t;


volatile syslog_mgr_flags_t * Syslog_init();

void Syslog_Error(Tuareg_ID Src, U8 Location);
void Syslog_Error_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload);

void Syslog_Warning(Tuareg_ID Src, U8 Location);
void Syslog_Warning_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload);

void Syslog_Info(Tuareg_ID Src, U8 Location);
void Syslog_Info_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload);

U32 append_datalog(volatile datalog_entry_t * pSource);

void show_syslog(USART_TypeDef * Port);
void show_datalog(USART_TypeDef * Port);

void send_syslog(USART_TypeDef * Port);

void clear_syslog();
void clear_datalog();

#endif // SYSLOG_H_INCLUDED
