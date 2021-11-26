#ifndef DECODER_DEBUG_H_INCLUDED
#define DECODER_DEBUG_H_INCLUDED





typedef union
{
     U32 all_flags;

     struct
     {
        //preconditions
        U32 decoder_period_valid :1;
        U32 cis_failure :1;

        U32 lobe_begin_detected :1;
        U32 lobe_end_detected :1;

        U32 preconditions_ok :1;

        //output
        U32 cis_triggered :1;
        U32 phase_match :1;

        //interface
        U32 decoder_output_phase_valid :1;

     };

} decoder_cis_debug_flags_t;



/**
decoder_cis_debug_t defines a transfer object for cis debugging
*/
typedef struct _decoder_cis_debug_t {

    decoder_cis_debug_flags_t flags;

    //interval calculation input
    U32 lobe_begin_ts;
    U32 lobe_end_ts;
    U32 decoder_timer_period_us;
    U32 decoder_crank_period_us;

    //interval calculation output
    U32 lobe_interval_us;
    U32 lobe_angle_deg;

    //validation
    U32 decoder_cis_sync_counter;

} decoder_cis_debug_t;










#endif // DECODER_DEBUG_H_INCLUDED
