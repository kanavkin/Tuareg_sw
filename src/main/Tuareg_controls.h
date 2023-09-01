#ifndef TUAREG_CTRL_H_INCLUDED
#define TUAREG_CTRL_H_INCLUDED

#include <Tuareg_platform.h>
#include <ctrlset.h>








typedef struct _Tuareg_control_flags_t {


    //control strategy
    U32 SPD_ctrl :1;
    U32 sequential_ctrl :1;

    //data health state
    U32 valid :1;


} Tuareg_control_flags_t;



/**

*/
typedef struct _Tuareg_controls_t {

    //control set data
    F32 IgnAdv;
    F32 VE;
    F32 AFRtgt;

    //mapset in use
    ctrlset_designator_t Set;


    /**
    state machine and health status
    */
    volatile Tuareg_control_flags_t Flags;


    /*
    system watchdogs
    VU32 decoder_watchdog;
    */


} Tuareg_controls_t;



exec_result_t Tuareg_update_controls();



#endif // TUAREG_CTRL_H_INCLUDED
