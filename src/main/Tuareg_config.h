#ifndef TUAREG_CONFIG_H_INCLUDED
#define TUAREG_CONFIG_H_INCLUDED

#include "uart.h"


/**
Tuareg_Setup_flags_t
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 CrashSensor_trig_high :1;
        U8 SidestandSensor_trig_high :1;
        U8 RunSwitch_override :1;
        U8 Sidestand_override :1;
        U8 CrashSensor_override :1;
        U8 QuietDash :1;
        U8 SmoothTrans_ena :1;
     };

} Tuareg_Setup_flags_t;


/***************************************************************************************************************************************************
*   Tuareg main configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Tuareg_Setup_t {

    U8 Version;

    //advance angles corresponding to crank_position_t
    U16 trigger_advance_map[CRK_POSITION_COUNT];

    //dynamic delay introduced by VR interface schematics (between key passing sensor and decoder event generation)
    U16 decoder_delay_us;

    //rev limiter function
    U16 max_rpm;
    U16 limp_max_rpm;

    //overheat protection
    U16 overheat_thres_K;

    //standby timeout
    U8 standby_timeout_s;

    //control strategy parameters
    U16 spd_max_rpm;
    U16 smooth_transition_radius_rpm;

    //conversion factors for ground speed calculation
    F32 gear_ratio[GEAR_COUNT -1];

    //EMA filter factors
    F32 TPS_alpha;
    F32 MAP_alpha;

    //fuel pump priming duration
    U8 fuel_pump_priming_duration;

    //all boolean elements
    volatile Tuareg_Setup_flags_t flags;

} Tuareg_Setup_t;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Tuareg_Setup_t Tuareg_Setup;

exec_result_t load_Tuareg_Config();

exec_result_t store_Tuareg_Setup();
void show_Tuareg_Setup(USART_TypeDef * Port);
exec_result_t modify_Tuareg_Setup(U32 Offset, U32 Value);
void send_Tuareg_Setup(USART_TypeDef * Port);

exec_result_t load_TachTable();
exec_result_t store_TachTable();
void show_TachTable(USART_TypeDef * Port);
exec_result_t modify_TachTable(U32 Offset, U32 Value);
void send_TachTable(USART_TypeDef * Port);
U32 getValue_TachTable(U32 Rpm);



/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_TRIGGER_ANGLE(angle) if((angle) > 360) return EXEC_ERROR



#endif // TUAREG_CONFIG_H_INCLUDED
