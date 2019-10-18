#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"


/**
the amount of timer 9 ticks until we re enable the crank pickup irq
adjusted to about 2° crank shaft at 9500 rpm
(smallest segment is about 5° in length)
(setting: ps 400 at 100 MHz)
*/
#define CRANK_NOISE_FILTER 8


/**
segment 1 has a key/gap ratio of 0.8
*/
#define SYNC_RATIO_MIN 65
#define SYNC_RATIO_MAX 95

/**
the amount of timer 2 update events until we detect a stalled engine
adjusted to 5 s
(setting: ps 400 at 100 MHz)
*/
#define DECODER_TIMEOUT 19


/**
when to enable cylinder identification sensor irq
*/
#define CYLINDER_SENSOR_ENA_POSITION POSITION_B2
#define CYLINDER_SENSOR_DISA_POSITION POSITION_C1


typedef enum {

    CYL1_WORK,
    CYL2_WORK,
    PHASE_UNDEFINED

} engine_phase_t;


typedef enum {

    INIT= 0,
    ASYNC_KEY= 0xAA,
    ASYNC_GAP= 0xAB,
    SYNC= 0xB0

} decoder_state_t;


typedef enum {

    POSITION_A1= 0,
    POSITION_A2= 1,
    POSITION_B1= 2,
    POSITION_B2= 3,
    POSITION_C1= 4,
    POSITION_C2= 5,
    POSITION_D1= 6,
    POSITION_D2= 7,
    UNDEFINED_POSITION= 0xFF

} engine_position_t;


typedef struct {

    decoder_state_t sync_mode;

    U32 sync_buffer_key;
    U32 sync_buffer_gap;

    U32 cycle_timing_buffer;
    U32 cycle_timing_counter;

    U32 timeout_count;

    engine_position_t crank_position;

    U32 engine_rpm;
    U32 rpm_calc_constant;
    U32 crank_rotation_period_us;

    engine_phase_t phase;


    //decoder statistics
    U32 diag_positions_crank_synced;
    U32 diag_positions_crank_async;
    U32 diag_sync_lost_events;
    U32 diag_positions_cis_phased;
    U32 diag_positions_cis_undefined;
    U32 diag_phase_lost_events;

} decoder_logic_t;

volatile decoder_logic_t * init_decoder_logic();

extern void decoder_logic_crank_handler(VU32 Timestamp);
extern void decoder_logic_cam_handler();
extern void decoder_logic_timer_compare_handler();
extern void decoder_logic_timer_update_handler();


#endif // DECODERLOGIC_H_INCLUDED
