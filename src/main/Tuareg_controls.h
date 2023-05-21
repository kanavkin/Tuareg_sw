#ifndef TUAREG_CTRL_H_INCLUDED
#define TUAREG_CTRL_H_INCLUDED

#include <Tuareg_platform.h>



extern const crank_position_t cTuareg_controls_update_pos;




typedef struct _Tuareg_control_flags_t {

    /**
    is engine operation allowed?
    single source of truth for all vital preconditions:
    !fatal_error
    !rev_limiter
    !overheat
    !kill_switch, ...

    */
    U32 run_permission :1;

    //special operation conditions
    U32 service_mode :1;

    U32 limited_op :1;
    U32 rev_limiter :1;

    U32 standby :1;
    U32 cranking :1;
    U32 idle :1;



    //control strategy
    U32 MAP_nTPS_ctrl :1;
    U32 sequential_ctrl :1;


} Tuareg_control_flags_t;



/**

*/
typedef struct _Tuareg_controls_t {

    //control set data
    F32 IgnAdv;
    F32 VE;
    F32 AFRtgt;



    /**
    state machine and health status
    */
    volatile Tuareg_control_flags_t flags;


    /*
    system watchdogs
    VU32 decoder_watchdog;
    */


} Tuareg_controls_t;



exec_result_t Tuareg_update_controls();



#endif // TUAREG_CTRL_H_INCLUDED
