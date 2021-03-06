#ifndef HSPDLOG_H_INCLUDED
#define HSPDLOG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_ID.h"


#define HIGSPEEDLOG_LENGTH 35

#define HIGSPEEDLOG_BYTE5_CIS_LOBE_BIT 4
#define HIGSPEEDLOG_BYTE5_PHASE_COMP_BIT 5
#define HIGSPEEDLOG_BYTE5_PHASE_VALID_BIT 6

#define HIGSPEEDLOG_BYTE6_COIL1_POWERED_BIT 0
#define HIGSPEEDLOG_BYTE6_COIL2_POWERED_BIT 1
#define HIGSPEEDLOG_BYTE6_INJECTOR1_POWERED_BIT 2
#define HIGSPEEDLOG_BYTE6_INJECTOR2_POWERED_BIT 3



typedef enum {

    HLOGA_CRKPOS_UPD,


    HLOGA_CAMLOBE_BEG,
    HLOGA_CAMLOBE_END,

    HLOGA_COIL1_POWER,
    HLOGA_COIL1_UNPOWER,

    HLOGA_COIL2_POWER,
    HLOGA_COIL2_UNPOWER,

    HLOGA_INJECTOR1_POWER,
    HLOGA_INJECTOR1_UNPOWER,

    HLOGA_INJECTOR2_POWER,
    HLOGA_INJECTOR2_UNPOWER,

    HLOGA_ALIAS,

    HLOGA_COUNT

} highspeedlog_event_t;


/**
    highspeedlogentry_flags_t
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 coil1 :1;
        U8 coil2 :1;
        U8 injector1 :1;
        U8 injector2 :1;

        U8 cam_lobe :1;

        U8 phase_valid :1;
        U8 phase_comp :1;
     };

} highspeedlogentry_flags_t;




typedef struct _highspeedlog_entry_t {

    timestamp_t system_ts;
    timestamp_t fraction_ts;

    highspeedlog_event_t event;

    crank_position_t crank_position;

    highspeedlogentry_flags_t flags;

} highspeedlog_entry_t;






typedef struct _highspeedlog_flags_t {

    U8 log_full :1;


} highspeedlog_flags_t;



typedef struct _highspeedlog_mgr_t {

    U32 entry_ptr;

    bool cam_lobe_begin_triggered;

    highspeedlog_flags_t flags;

} highspeedlog_mgr_t;





volatile highspeedlog_flags_t * highspeedlog_init();
void clear_highspeedlog();

void highspeedlog_write(highspeedlog_event_t Event);

void highspeedlog_register_error();

void highspeedlog_register_crankpos();

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


void send_highspeedlog(USART_TypeDef * Port);





#endif // HSPDLOG_H_INCLUDED
