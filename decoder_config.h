#ifndef DECODER_CONFIG_H_INCLUDED
#define DECODER_CONFIG_H_INCLUDED

/***************************************************************************************************************************************************
*   decoder configuration page
***************************************************************************************************************************************************/
typedef struct _Decoder_Config_t {

    //contains the trigger wheel layout (angles corresp. to positions A1 .. D2)
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

} Decoder_Config_t;



/***************************************************************************************************************************************************
*   data offset constants for modification via CLI
***************************************************************************************************************************************************/
typedef enum {

    DECODER_CONFIG_CLI_TRIGGERMAP_A1,
    DECODER_CONFIG_CLI_TRIGGERMAP_A2,
    DECODER_CONFIG_CLI_TRIGGERMAP_B1,
    DECODER_CONFIG_CLI_TRIGGERMAP_B2,
    DECODER_CONFIG_CLI_TRIGGERMAP_C1,
    DECODER_CONFIG_CLI_TRIGGERMAP_C2,
    DECODER_CONFIG_CLI_TRIGGERMAP_D1,
    DECODER_CONFIG_CLI_TRIGGERMAP_D2,

    DECODER_CONFIG_CLI_TRIGGER_OFFSET,
    DECODER_CONFIG_CLI_VR_DELAY,
    DECODER_CONFIG_CLI_CRANK_NOISE_FILTER,
    DECODER_CONFIG_CLI_SYNC_RATIO_MIN,
    DECODER_CONFIG_CLI_SYNC_RATIO_MAX,
    DECODER_CONFIG_CLI_TIMEOUT

} Decoder_Config_CLI_offset;


/***************************************************************************************************************************************************
*   data offsets for for eeprom layout based on the stored data size
***************************************************************************************************************************************************/
typedef enum {

    DECODER_CONFIG_EE_VERSION =0,               // 1 byte

    DECODER_CONFIG_EE_TRIGGERMAP =1,            // 8*2 bytes 1..16

    DECODER_CONFIG_EE_TRIGGER_OFFSET =17,       // 2 bytes 17..18
    DECODER_CONFIG_EE_VR_DELAY =19,             // 2 bytes 19..20
    DECODER_CONFIG_EE_CRANK_NOISE_FILTER =20,   // 1 byte
    DECODER_CONFIG_EE_SYNC_RATIO_MIN =21,       // 1 byte
    DECODER_CONFIG_EE_SYNC_RATIO_MAX =22,       // 1 byte
    DECODER_CONFIG_EE_TIMEOUT =23,              // 1 byte

    DECODER_CONFIG_EE_NEXT_FREE_OFFSET =24      // next free offset after the page

} Decoder_Config_Eeprom_offset;



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
Decoder_Config.trigger_position_map[POSITION_COUNT]

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
Decoder_Config.trigger_offset_deg

default:
DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET
*/
#define DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET 260


/**
decoder delay

dynamic delay introduced by VR interface schematics (between key passing sensor and signal edge generation)
the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

config item:
Decoder_Config.vr_delay_us

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
Decoder_Config.crank_noise_filter

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
Decoder_Config.sync_ratio_min_pct
Decoder_Config.sync_ratio_max_pct

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
Decoder_Config.timeout_s

defaults:
DECODER_CONFIG_DEFAULT_TIMEOUT
*/
#define DECODER_CONFIG_DEFAULT_TIMEOUT 3



/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Decoder_Config_t Decoder_Config;

exec_result_t load_Decoder_Config();
void load_essential_Decoder_Config();
exec_result_t write_Decoder_Config();

void show_Decoder_Config(USART_TypeDef * Port);

exec_result_t modify_Decoder_Config(U32 Offset, U32 Value);


/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_TRIGGER_ANGLE(angle) if((angle) > 360) return EXEC_ERROR



#endif // DECODER_CONFIG_H_INCLUDED
