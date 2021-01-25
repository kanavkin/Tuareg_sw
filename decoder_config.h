#ifndef DECODER_CONFIG_H_INCLUDED
#define DECODER_CONFIG_H_INCLUDED

#define DECODER_SETUP_SIZE 6

/***************************************************************************************************************************************************
*   decoder configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Decoder_Setup_t {

    //minimal amount of timer ticks between two crankshaft sensor irq can be detected
    U8 crank_noise_filter;

    // sync check begins with key
    crank_position_t sync_check_begin_pos;

    // valid key/90° ratio interval for detecting positions key/gap
    U8 sync_ratio_min_pct;
    U8 sync_ratio_max_pct;

    //amount of seconds until a decoder timeout will be detected, when no trigger event has occurred
    U8 timeout_s;

    U8 Version;

} Decoder_Setup_t;


/***************************************************************************************************************************************************
*   essential config section
***************************************************************************************************************************************************/

/**
crank noise filter

the amount of timer  ticks until we re enable the crank pickup irq
adjusted to about 2° crank shaft at 9500 rpm
(smallest segment is about 5° in length)
(setting: ps 400 at 100 MHz)

config item:
Decoder_Setup.crank_noise_filter

default:
DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER
*/
#define DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER 8


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
Decoder_Setup.sync_check_begin_pos

defaults:
DECODER_CONFIG_DEFAULT_SYNC_CHECK_BEGIN_POS
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX
*/
#define DECODER_CONFIG_DEFAULT_SYNC_CHECK_BEGIN_POS 2
#define DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN 20
#define DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX 60


/**
decoder timeout detection

amount of seconds until a decoder timeout will be detected, when no trigger event has occurred

config items:
Decoder_Setup.timeout_s

defaults:
DECODER_CONFIG_DEFAULT_TIMEOUT
*/
#define DECODER_CONFIG_DEFAULT_TIMEOUT 3



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
