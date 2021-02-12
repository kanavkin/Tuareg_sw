#ifndef DECODER_CONFIG_H_INCLUDED
#define DECODER_CONFIG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "decoder_hw.h"
#include "decoder_logic.h"

#define DECODER_SETUP_SIZE 15

/***************************************************************************************************************************************************
*   decoder configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Decoder_Setup_t {

    /*
    version
    */
    U8 Version;

    /*
    general setup
    */

    //amount of seconds until a decoder timeout will be detected, when no trigger event has occurred
    U8 timeout_s;

    /*
    crank sensor setup
    */

    //amount of timer ticks until the crank sensor related external irq will be re enabled
    U8 crank_noise_filter;

    //sensing polarity to set up for crank sensor keys
    decoder_sensing_t key_begin_sensing;
    decoder_sensing_t key_end_sensing;

    // valid key/90° ratio interval for detecting positions key/gap
    U8 sync_ratio_min_pct;
    U8 sync_ratio_max_pct;

    /*
    cis setup
    */

    //amount of timer ticks until the crank sensor related external irq will be re enabled
    U8 cam_noise_filter;

    //sensing polarity to set up for cis cam lobe
    decoder_sensing_t lobe_begin_sensing;
    decoder_sensing_t lobe_end_sensing;

    //position to enable/disable the cis
    crank_position_t cis_enable_pos;
    crank_position_t cis_disable_pos;

    //resulting engine phase when cis has received a valid cam signal
    engine_phase_t cis_triggered_phase;

    //minimal crank angle the cam signal must have been present to the cis to detect cis_triggered_phase
    U8 cis_min_angle_deg;

    //minimal amount of correctly detected cam cycles to consider the phase information valid
    U8 cis_sync_thres;

} Decoder_Setup_t;


/***************************************************************************************************************************************************
*   essential config section
***************************************************************************************************************************************************/

/**
decoder software version

indicates the decoder software version for which these defaults apply

config items:
Decoder_Setup.Version

defaults:
DECODER_CONFIG_DEFAULT_VERSION
*/
#define DECODER_CONFIG_DEFAULT_VERSION 3


/**
decoder timeout detection

amount of seconds until a decoder timeout will be detected, when no trigger event has occurred

config items:
Decoder_Setup.timeout_s

defaults:
DECODER_CONFIG_DEFAULT_TIMEOUT
*/
#define DECODER_CONFIG_DEFAULT_TIMEOUT 3


/**
crank noise filter

the amount of timer  ticks until we re enable the crank pickup irq
adjusted to about 2° crank shaft at 9500 rpm
(smallest segment is about 5° in length)
(setting: ps 800 at 100 MHz)

config item:
Decoder_Setup.crank_noise_filter

default:
DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER
*/
#define DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER 4


/**
crank pickup polarity

polarity to set up for crank sensor key beginning / end

config item:
Decoder_Setup.key_begin_sensing
Decoder_Setup.key_end_sensing

default:
DECODER_CONFIG_DEFAULT_KEY_BEGIN_SENSING
DECODER_CONFIG_DEFAULT_KEY_END_SENSING
*/
#define DECODER_CONFIG_DEFAULT_KEY_BEGIN_SENSING SENSING_FALL
#define DECODER_CONFIG_DEFAULT_KEY_END_SENSING SENSING_RISE


/**
sync checker

segment 1 has a key to (key + gap) ratio of about 40 percent

The decoder sync check begins with storing the current captured interval as key interval (at key end). The next captured interval will be considered the gap interval.
The crank position after a successful sync check will be the position after sync_check_begin_pos

this config defines the interval, which measured sync ratios will be considered valid
a relaxed sync check will be applied, when less than sync_stability_thrs consecutive positions have been captured with sync

config items:
Decoder_Setup.sync_ratio_min_pct
Decoder_Setup.sync_ratio_max_pct

defaults:
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX
*/
#define DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN 20
#define DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX 60


/**
cam noise filter

the amount of timer  ticks until we re enable the cis irq


config item:
Decoder_Setup.cam_noise_filter

default:
DECODER_CONFIG_DEFAULT_CAM_NOISE_FILTER
*/
#define DECODER_CONFIG_DEFAULT_CAM_NOISE_FILTER 25


/**
cis signal polarity

polarity to set up for cis lobe beginning / end

config item:
Decoder_Setup.lobe_begin_sensing
Decoder_Setup.lobe_end_sensing

default:
DECODER_CONFIG_DEFAULT_LOBE_BEGIN_SENSING
DECODER_CONFIG_DEFAULT_LOBE_END_SENSING
*/
#define DECODER_CONFIG_DEFAULT_LOBE_BEGIN_SENSING SENSING_RISE
#define DECODER_CONFIG_DEFAULT_LOBE_END_SENSING SENSING_FALL


/**
cis activation

crank position to enable/disable the cis

config item:
Decoder_Setup.cis_enable_pos
Decoder_Setup.cis_disable_pos

default:
DECODER_CONFIG_DEFAULT_CIS_ENABLE_POS
DECODER_CONFIG_DEFAULT_CIS_DISABLE_POS
*/
#define DECODER_CONFIG_DEFAULT_CIS_ENABLE_POS CRK_POSITION_C1
#define DECODER_CONFIG_DEFAULT_CIS_DISABLE_POS CRK_POSITION_D2


/**
cis phase polarity

resulting engine phase when cis has been triggered by a valid cam signal

config item:
Decoder_Setup.cis_triggered_phase

default:
DECODER_CONFIG_DEFAULT_CIS_TRIGGERED_PHASE
*/
#define DECODER_CONFIG_DEFAULT_CIS_TRIGGERED_PHASE PHASE_CYL1_EX


/**
cis signal validation

minimal crank angle the cam signal must have been present to the cis to detect the triggered phase

config item:
Decoder_Setup.cis_min_angle_deg

default:
DECODER_CONFIG_DEFAULT_CIS_MIN_ANGLE_DEG
*/
#define DECODER_CONFIG_DEFAULT_CIS_MIN_ANGLE_DEG 40


/**
cis synchronization threshold

minimal amount of correctly detected cam cycles to consider the phase information valid

config item:
Decoder_Setup.cis_sync_thres

default:
DECODER_CONFIG_DEFAULT_CIS_SYNC_THRES
*/
#define DECODER_CONFIG_DEFAULT_CIS_SYNC_THRES 100



/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Decoder_Setup_t Decoder_Setup;

exec_result_t load_Decoder_Setup();
void load_essential_Decoder_Setup();
exec_result_t store_Decoder_Setup();

void show_Decoder_Setup(USART_TypeDef * Port);

exec_result_t modify_Decoder_Setup(U32 Offset, U32 Value);

void send_Decoder_Setup(USART_TypeDef * Port);


/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_TRIGGER_ANGLE(angle) if((angle) > 360) return EXEC_ERROR



#endif // DECODER_CONFIG_H_INCLUDED
