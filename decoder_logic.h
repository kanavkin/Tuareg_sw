#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "trigger_wheel_layout.h"

/**
essential config section
*/

/**
decoder offset

static correction angle between the trigger wheel key (POSITION_xx_ANGLE) and crank angle
the angle the crank shaft is actually at significantly differs from the trigger wheel key position angle

config item:
configPage12.decoder_offset_deg

default:
DEFAULT_CONFIG12_DECODER_OFFSET
*/
#define DEFAULT_CONFIG12_DECODER_OFFSET 260


/**
decoder delay

dynamic delay introduced by VR interface schematics (between key passing sensor and signal edge generation)
the VR interface hw introduces a delay of about 300 us from the key edge passing the sensor until the CRANK signal is triggered

config item:
configPage12.decoder_delay_us

default:
DEFAULT_CONFIG12_DECODER_DELAY
*/
#define DEFAULT_CONFIG12_DECODER_DELAY 320


/**
crank noise filter

the amount of timer  ticks until we re enable the crank pickup irq
adjusted to about 2° crank shaft at 9500 rpm
(smallest segment is about 5° in length)
(setting: ps 400 at 100 MHz)

config item:
configPage12.crank_noise_filter

default:
DEFAULT_CONFIG12_CRANK_NOISE_FILTER
*/
#define DEFAULT_CONFIG12_CRANK_NOISE_FILTER 8


/**
sync checker

segment 1 has a key to (key + gap) ratio of about 40 percent

this config defines the interval, which measured sync ratios will be considered valid
a relaxed sync check will be applied, when less than sync_stability_thrs consecutive positions have been captured with sync

config items:
configPage12.sync_ratio_min_pct
configPage12.sync_ratio_max_pct
configPage12.sync_stability_thrs

defaults:
DEFAULT_CONFIG12_SYNC_RATIO_MIN
DEFAULT_CONFIG12_SYNC_RATIO_MAX
*/
#define DEFAULT_CONFIG12_SYNC_RATIO_MIN 30
#define DEFAULT_CONFIG12_SYNC_RATIO_MAX 60
#define DEFAULT_CONFIG12_SYNC_STABILITY_THRS 80


/**
decoder timeout detection

amount of seconds until a decoder timeout will be detected, when no trigger event has occurred

config items:
configPage12.decoder_timeout_s

defaults:
DEFAULT_CONFIG12_DECODER_TIMEOUT_S
*/
#define DEFAULT_CONFIG12_DECODER_TIMEOUT_S 3


/**
when to enable cylinder identification sensor irq
*/
#define CYLINDER_SENSOR_ENA_POSITION CRK_POSITION_B2
#define CYLINDER_SENSOR_DISA_POSITION CRK_POSITION_C1


typedef enum {

    INIT,
    ASYNC_KEY,
    ASYNC_GAP,
    SYNC

} decoder_state_t;


typedef enum {

    DDIAG_ASYNC_SYNC_TR,
    DDIAG_SYNC_ASYNC_TR,

    DDIAG_CRANKHANDLER_CALLS,
    DDIAG_TRIGGER_IRQ_SYNC,
    DDIAG_TRIGGER_IRQ_DELAY,


    DDIAG_CRANKPOS_INIT,
    DDIAG_CRANKPOS_SYNC,
    DDIAG_CRANKPOS_ASYNC_KEY,
    DDIAG_CRANKPOS_ASYNC_GAP,

    DDIAG_TIMEOUT_EVENTS,
    DDIAG_TIMER_UPDATE_EVENTS,

    DDIAG_SYNCCHECK_CALLS,
    DDIAG_SYNCCHECK_RELAXED,

    DDIAG_CRANKTABLE_CALLS,
    DDIAG_ROTSPEED_CALLS,

    DDIAG_CISHANDLER_CALLS,
    DDIAG_CRANKPOS_CIS_PHASED,
    DDIAG_CRANKPOS_CIS_UNDEFINED,
    DDIAG_PHASED_UNDEFINED_TR,

    DDIAG_COUNT

} decoder_diag_t;


typedef struct {

    /*
    decoder syncronisation
    */
    volatile decoder_state_t sync_mode;

    VU32 sync_buffer_key;
    VU32 sync_buffer_gap;
    VU32 sync_stability;

    /*
    engine dynamics
    */
    VU32 cycle_timing_buffer;
    VU32 cycle_timing_counter;

    /*
    crank position
    */
    volatile crank_position_t crank_position;


    /*
    stalled engine detection
    - decoder_timeout_thrs reflects the configured threshold in timer update events when the timeout shall occur
    - timeout_count is the actually counts, how much consecutive timer update events have occurred
    */
    U32 decoder_timeout_thrs;
    VU32 timeout_count;

    /*
    camshaft
    */
    volatile engine_phase_t phase;

    //decoder statistics
    VU32 diag[DDIAG_COUNT];

    //segment duration statistics
    VU16 segment_duration_deg[CRK_POSITION_COUNT];
    VU16 segment_duration_base_rpm;

} decoder_internals_t;


typedef struct {

    //rotational period of crankshaft
    VU32 crank_T_us;
    VS32 crank_deltaT_us;

    /*
    crank position
    */
    volatile crank_position_t crank_position;

    /*
    camshaft
    */
    volatile engine_phase_t phase;

} decoder_interface_t;


volatile decoder_interface_t * init_decoder_logic();

extern void decoder_logic_crank_handler(VU32 timer_buffer);
extern void decoder_logic_cam_handler();
extern void decoder_logic_timer_compare_handler();
extern void decoder_logic_timer_update_handler();

extern void reset_position_data();
extern void reset_crank_timing_data();
extern void reset_sync_stability();

void update_crank_position_table(volatile crank_position_table_t * Table);
void update_engine_speed(VU32 Interval);

extern void reset_diag_data();
void decoder_export_diag(VU32 * pTarget);

void decoder_statistics_handler(VU32 Interval);
void reset_decoder_statistics();
void decoder_export_statistics(VU32 * pTarget);

extern void sync_lost_debug_handler();
extern void got_sync_debug_handler();
extern void decoder_timeout_debug_handler();

#endif // DECODERLOGIC_H_INCLUDED
