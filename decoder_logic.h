#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "trigger_wheel_layout.h"
#include "process_table.h"

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
#define DEFAULT_CONFIG12_DECODER_DELAY 40


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

    DSTATE_INIT,
    DSTATE_ASYNC,
    DSTATE_ASYNC_KEY,
    DSTATE_ASYNC_GAP,
    DSTATE_SYNC,
    DSTATE_COUNT

} decoder_state_t;


typedef struct {

    /*
    decoder synchronization
    */
    volatile decoder_state_t state;


    //sync timing
    VU32 sync_buffer_key;
    VU32 sync_buffer_gap;

    //cycle timing
    VU32 cycle_timing_buffer;
    VU32 cycle_timing_counter;

    //rotational period of crankshaft
    VU32 crank_period_us;


    //crank position
    volatile crank_position_t crank_position;

    volatile engine_phase_t phase;


    /*
    stalled engine detection
    - decoder_timeout_thrs reflects the configured threshold in timer update events when the timeout shall occur
    - timeout_count is the actually counts, how much consecutive timer update events have occurred
    */
    U32 decoder_timeout_thrs;
    VU32 timeout_count;


    //segment duration statistics
    /*
    VU16 segment_duration_deg[CRK_POSITION_COUNT];
    VU16 segment_duration_base_rpm;
    */


} decoder_internals_t;


typedef struct {

    //rotational period of crankshaft
    VU32 crank_period_us;

    volatile crank_position_t crank_position;
    volatile engine_phase_t phase;

} decoder_interface_t;


volatile decoder_interface_t * init_decoder_logic();

extern void decoder_logic_crank_handler(VU32 timer_buffer);
extern void decoder_logic_cam_handler();
extern void decoder_logic_timer_compare_handler();
extern void decoder_logic_timer_update_handler();

extern void reset_position_data();
extern void reset_crank_timing_data();

extern void decoder_set_state(decoder_state_t NewState);

void update_crank_position_table(volatile crank_position_table_t * pTable);

void update_engine_speed(VU32 Interval);

extern void reset_diag_data();
void decoder_export_diag(VU32 * pTarget);

void decoder_statistics_handler(VU32 Interval);
void reset_decoder_statistics();
void decoder_export_statistics(VU32 * pTarget);

extern void sync_lost_debug_handler();
extern void got_sync_debug_handler();
extern void decoder_timeout_debug_handler();

extern void decoder_update_interface();
extern void reset_timeout_counter();

extern VU32 check_sync_ratio();

#endif // DECODERLOGIC_H_INCLUDED
