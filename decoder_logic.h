#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "trigger_wheel_layout.h"


/**
the amount of timer  ticks until we re enable the crank pickup irq
adjusted to about 2° crank shaft at 9500 rpm
(smallest segment is about 5° in length)
(setting: ps 400 at 100 MHz)
*/
#define CRANK_NOISE_FILTER 8


/**
segment 1 has a sync ratio of 0.4
*/
#define SYNC_RATIO_MIN 30
#define SYNC_RATIO_MAX 60

/**
the amount of seconds until we detect a stalled engine
*/
#define DECODER_TIMEOUT_MS 3000UL


#define DECODER_TIMEOUT (DECODER_TIMEOUT_MS / DECODER_TIMER_OVERFLOW_MS)


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

    DDIAG_CRANKHANDLER_CALLS,
    DDIAG_CISHANDLER_CALLS,

    DDIAG_SYNCCHECK_CALLS,
    DDIAG_CRANKTABLE_CALLS,
    DDIAG_ROTSPEED_CALLS,

    DDIAG_ASYNC_SYNC_TR,
    DDIAG_SYNC_ASYNC_TR,

    DDIAG_CRANKPOS_INIT,
    DDIAG_CRANKPOS_SYNC,
    DDIAG_CRANKPOS_ASYNC_KEY,
    DDIAG_CRANKPOS_ASYNC_GAP,

    DDIAG_TRIGGER_IRQ_SYNC,
    DDIAG_TIMEOUT_EVENTS,

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

    /*
    engine dynamics
    */
    VU32 cycle_timing_buffer;
    VU32 cycle_timing_counter;

    /*
    crank position
    */
    volatile engine_position_t crank_position;

    /*
    stalled engine detection
    */
    VU32 timeout_count;

    /*
    camshaft
    */
    volatile engine_phase_t phase;

    //decoder statistics
    VU32 diag[DDIAG_COUNT];

} decoder_internals_t;


typedef struct {


    //rotational period of crankshaft
    VU32 crank_T_us;
    VS32 crank_deltaT_us;

    VU32 engine_rpm;

    /*
    crank position
    */
    volatile engine_position_t crank_position;


    /*
    calculated crank angles for the next positions
    at the current engine speed
    */
    VU16 crank_position_table_deg[POSITION_COUNT];

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
extern void reset_statistics_data();

void calculate_crank_position_table(U32 Period, U16 * Table);
void update_engine_speed(VU32 Interval);

#endif // DECODERLOGIC_H_INCLUDED
