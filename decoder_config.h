#ifndef DECODER_CONFIG_H_INCLUDED
#define DECODER_CONFIG_H_INCLUDED

#define DECODER_CONFIG_SIZE 24

/***************************************************************************************************************************************************
*   decoder configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Decoder_Setup_t {

    //contains the trigger wheel layout (angles corresp. to crank_position_t)
    crank_position_table_t trigger_position_map;

    //static correction angle between the trigger wheel key (POSITION_xx_ANGLE) and crank angle
    S16 trigger_offset_deg;

    //dynamic delay introduced by VR interface schematics (between key passing sensor and signal edge generation)
    U16 vr_delay_us;

    //minimal amount of timer ticks between two crankshaft sensor irq can be detected
    U8 crank_noise_filter;

    // valid key/90° ratio interval for detecting positions A2/B1
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
trigger wheel geometry

contains the trigger wheel layout (angles corresp. to positions A1 .. D2)
defines the crank angle corresponding to the trigger wheel key position,
counting from the position closest to TDC against the normal rotation direction
(reflecting ignition advance)

config item:
Decoder_Setup.trigger_position_map[POSITION_COUNT]

default:
DECODER_CONFIG_DEFAULT_POSITION_A1_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_A2_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_B1_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_B2_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_C1_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_C2_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_D1_ANGLE
DECODER_CONFIG_DEFAULT_POSITION_D2_ANGLE
*/
#define DECODER_CONFIG_DEFAULT_POSITION_A1_ANGLE 0
#define DECODER_CONFIG_DEFAULT_POSITION_A2_ANGLE 40
#define DECODER_CONFIG_DEFAULT_POSITION_B1_ANGLE 90
#define DECODER_CONFIG_DEFAULT_POSITION_B2_ANGLE 98
#define DECODER_CONFIG_DEFAULT_POSITION_C1_ANGLE 180
#define DECODER_CONFIG_DEFAULT_POSITION_C2_ANGLE 188
#define DECODER_CONFIG_DEFAULT_POSITION_D1_ANGLE 270
#define DECODER_CONFIG_DEFAULT_POSITION_D2_ANGLE 278


/**
decoder offset

static correction angle between the trigger wheel key (POSITION_xx_ANGLE) and crank angle
the angle the crank shaft is actually at significantly differs from the trigger wheel key position angle

config item:
Decoder_Setup.trigger_offset_deg

default:
DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET
*/
#define DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET 260


/**
decoder delay

dynamic delay introduced by VR interface schematics (between key passing sensor and signal edge generation)
the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

config item:
Decoder_Setup.vr_delay_us

default:
DECODER_CONFIG_DEFAULT_VR_DELAY
*/
#define DECODER_CONFIG_DEFAULT_VR_DELAY 40


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

this config defines the interval, which measured sync ratios will be considered valid
a relaxed sync check will be applied, when less than sync_stability_thrs consecutive positions have been captured with sync

config items:
Decoder_Setup.sync_ratio_min_pct
Decoder_Setup.sync_ratio_max_pct

defaults:
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN
DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX
*/
#define DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN 30
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
