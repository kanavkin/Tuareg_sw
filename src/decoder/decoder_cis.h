#ifndef DECODERCIS_H_INCLUDED
#define DECODERCIS_H_INCLUDED



typedef union
{
     U32 all_flags;

     struct
     {
        //preconditions
        U32 period_valid :1;
        U32 failure :1;
        U32 preconditions_ok :1;

        U32 lobe_begin_detected :1;
        U32 lobe_end_detected :1;

        //output
        U32 triggered :1;
        U32 phase_match :1;
        U32 phase_valid :1;

     };

} decoder_cis_flags_t;



typedef struct {

    U32 lobe_begin_timestamp;
    U32 lobe_end_timestamp;
    U32 sync_counter;
    U32 detected_lobe_ends;

    decoder_cis_flags_t flags;

} decoder_cis_t;



void enable_cis();
void disable_cis();
void decoder_update_cis();
void decoder_cis_handler();
void decoder_cis_noisefilter_handler();


#endif // DECODERCIS_H_INCLUDED
