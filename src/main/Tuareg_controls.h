#ifndef TUAREG_CTRL_H_INCLUDED
#define TUAREG_CTRL_H_INCLUDED

#include <Tuareg_platform.h>
#include <ctrlset.h>



extern const F32 cDefault_AFR_target;
extern const F32 cMin_AFR_target;
extern const F32 cMax_AFR_target;
extern const F32 cMin_VE_val;
extern const F32 cMax_VE_val;
extern const F32 cMin_Adv_val;
extern const F32 cMax_Adv_val;




typedef union
{
     U16 all_flags;

     struct
     {
        //control strategy
        U16 SPD_ctrl :1;
        U16 smooth_transition :1;
        U16 AFR_fallback :1;

        //data health state
        U16 valid :1;

     };

} Tuareg_control_flags_t;



/**

*/
typedef struct _Tuareg_controls_t {

    //control set data
    F32 IgnAdv_deg;
    F32 VE_pct;
    F32 AFRtgt;

    //mapset in use
    ctrlset_designator_t Set;

    //ignition timing and alignment
    ignition_controls_t Ignition;

    //fueling parameters
    fueling_control_t Fueling;

    //control flags
    volatile Tuareg_control_flags_t Flags;


} Tuareg_controls_t;


void Tuareg_update_controls();



#endif // TUAREG_CTRL_H_INCLUDED
