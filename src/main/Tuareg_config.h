#ifndef TUAREG_CONFIG_H_INCLUDED
#define TUAREG_CONFIG_H_INCLUDED

#include "uart.h"


#define TUAREG_SETUP_SIZE 53


/**
Tuareg_Setup_flags_t
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 CrashSensor_trig_high :1;
        U8 RunSwitch_trig_high :1;
        U8 SidestandSensor_trig_high :1;
        U8 Halt_on_SidestandSensor :1;
     };

} Tuareg_Setup_flags_t;


/***************************************************************************************************************************************************
*   Tuareg main configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Tuareg_Setup_t {

    U8 Version;

    //advance angles corresponding to crank_position_t
    VU16 trigger_advance_map[CRK_POSITION_COUNT];

    //dynamic delay introduced by VR interface schematics (between key passing sensor and decoder event generation)
    U16 decoder_delay_us;

    //rev limiter function
    U16 max_rpm;
    U16 limp_max_rpm;

    //overheat protection
    U16 overheat_thres_K;

    //standby timeout
    U8 standby_timeout_s;

    //rpm until cranking features are activated
    U16 cranking_end_rpm;

    //conversion factors for ground speed calculation
    VF32 gear_ratio[GEAR_COUNT];


    //all boolean elements
    volatile Tuareg_Setup_flags_t flags;

} Tuareg_Setup_t;


/***************************************************************************************************************************************************
*   essential config section
***************************************************************************************************************************************************/



/**
Maximum RPM

higher crank speed will trigger the rev limiter

config item:
Decoder_Setup.max_rpm

default:
TUAREG_SETUP_DEFAULT_MAX_RPM
*/
#define TUAREG_SETUP_DEFAULT_MAX_RPM 4000


/**
trigger wheel geometry

contains the trigger wheel layout (angles corresp. to positions A1 .. D2)
defines the crank angle corresponding to the trigger wheel key position,
counting from the position closest to TDC against the normal rotation direction
(reflecting ignition advance)

config item:
Tuareg_Setup.trigger_position_map[POSITION_COUNT]

default:
TUAREG_SETUP_DEFAULT_POSITION_A1_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_A2_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_B1_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_B2_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_C1_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_C2_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_D1_ADVANCE
TUAREG_SETUP_DEFAULT_POSITION_D2_ADVANCE
*/
#define TUAREG_SETUP_DEFAULT_POSITION_A1_ADVANCE 98
#define TUAREG_SETUP_DEFAULT_POSITION_A2_ADVANCE 60
#define TUAREG_SETUP_DEFAULT_POSITION_B1_ADVANCE 10
#define TUAREG_SETUP_DEFAULT_POSITION_B2_ADVANCE 3
#define TUAREG_SETUP_DEFAULT_POSITION_C1_ADVANCE 180
#define TUAREG_SETUP_DEFAULT_POSITION_C2_ADVANCE 172
#define TUAREG_SETUP_DEFAULT_POSITION_D1_ADVANCE 190
#define TUAREG_SETUP_DEFAULT_POSITION_D2_ADVANCE 278


/**
VR delay

dynamic delay introduced by VR interface schematics (between key passing sensor and signal edge generation)
the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

config item:
Decoder_Setup.decoder_delay_us

default:
TUAREG_SETUP_DEFAULT_VR_DELAY
*/
#define TUAREG_SETUP_DEFAULT_DECODER_DELAY 40





/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Tuareg_Setup_t Tuareg_Setup;

exec_result_t load_Tuareg_Setup();
void load_essential_Tuareg_Setup();
exec_result_t store_Tuareg_Setup();

void show_Tuareg_Setup(USART_TypeDef * Port);

exec_result_t modify_Tuareg_Setup(U32 Offset, U32 Value);

void send_Tuareg_Setup(USART_TypeDef * Port);


/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_TRIGGER_ANGLE(angle) if((angle) > 360) return EXEC_ERROR



#endif // TUAREG_CONFIG_H_INCLUDED
