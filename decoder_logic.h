#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "process_table.h"

#define DECODER_REQUIRED_CONFIG_VERSION 2


/**
when to enable cylinder identification sensor irq
*/
#define CYLINDER_SENSOR_ENA_POSITION CRK_POSITION_B2
#define CYLINDER_SENSOR_DISA_POSITION CRK_POSITION_C1


typedef enum {

    DSTATE_TIMEOUT,
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

    VU8 timeout :1;
    VU8 position_valid :1;
    VU8 phase_valid :1;
    VU8 period_valid :1;
    VU8 rpm_valid :1;

} decoder_opstate_t;



typedef struct {

    //state
    decoder_opstate_t state;

    //rotational period of crankshaft
    VU32 crank_period_us;
    VU32 crank_rpm;

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

extern bool check_sync_ratio();

#endif // DECODERLOGIC_H_INCLUDED
