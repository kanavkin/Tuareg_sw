#ifndef DECODER_DEBUG_H_INCLUDED
#define DECODER_DEBUG_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_decoder.h"
#include "decoder_cis.h"
#include "decoder_logic.h"


/******************************************************************************************************************************
decoder debugging functions - compiler switches
******************************************************************************************************************************/

//enable event debugging messages?
//#define DECODER_EVENT_DEBUG

//enable cis debugging?
//#define DECODER_CIS_DEBUG

//decoder timing debugging?
//#define DECODER_TIMING_DEBUG

//decoder init debugging?
//#define DECODER_INIT_DEBUG




/*********************************************************************************************************************************
decoder init debugging
*********************************************************************************************************************************/

typedef enum {

    DECINITDBG_HW_INPUTS,
    DECINITDBG_HW_SENSING,
    DECINITDBG_HW_TIMER,
    DECINITDBG_HW_IRQ,
    DECINITDBG_LOGIC_INTERNALS,
    DECINITDBG_LOGIC_STATE_INIT,
    DECINITDBG_LOGIC_STANDSTILL,
    DECINITDBG_LOGIC_UNMASK_CRK,
    DECINITDBG_COUNT

} decoder_init_debug_t;


void init_decoder_debug(decoder_init_debug_t Action);


/*********************************************************************************************************************************
timing debugging
*********************************************************************************************************************************/


typedef struct {

    U32 hw_period_us;
    U32 hw_timer_val;

    decoder_output_t out;

} decoder_timing_debug_t;



void decoder_timing_debug_next_cycle();
void decoder_update_timing_debug();




/******************************************************************************************************************************
event debugging
******************************************************************************************************************************/

typedef union
{
     U8 all_flags;

     struct {

        VU8 got_sync :1;
        VU8 lost_sync :1;
        VU8 timer_overflow :1;
        VU8 standstill :1;
     };

} decoder_event_debug_flags_t;


extern decoder_event_debug_flags_t Decoder_Events;

void decoder_process_debug_events();
void register_sync_lost_debug_event();
void register_got_sync_debug_event();
void register_timer_overflow_debug_event();
void register_timeout_debug_event();






/******************************************************************************************************************************
cis debugging
******************************************************************************************************************************/


/**
decoder_cis_debug_t defines a transfer object for cis debugging
*/
typedef struct _decoder_cis_debug_t {

    decoder_cis_t cis;

    //interval calculation input
    U32 timer_period_us;
    U32 crank_period_us;

    //interval calculation output
    U32 lobe_interval_us;
    U32 lobe_angle_deg;

} decoder_cis_debug_t;


void decoder_cis_debug_next_cycle();
void decoder_update_cis_debug(U32 Lobe_interval_us, U32 Lobe_angle_deg);
void print_decoder_cis_debug_data(USART_TypeDef * Port);


/******************************************************************************************************************************
interface debugging
******************************************************************************************************************************/
void decoder_debug_show_internals(USART_TypeDef * Port);




/******************************************************************************************************************************
decoder event debugging
******************************************************************************************************************************/

void trigger_crk_irq_debug();

void trigger_cam_irq_debug();




#endif // DECODER_DEBUG_H_INCLUDED
