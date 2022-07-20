#ifndef TUAREG_DIAGNOSTICS_H_INCLUDED
#define TUAREG_DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


typedef enum {

    //EXTI2_IRQHandler()
    TDIAG_DECODER_IRQ,
    TDIAG_DECODER_UPDATE,
    TDIAG_DECODER_TIMEOUT,
    TDIAG_DECODER_PASSIVE,

    //EXTI3_IRQHandler()
    TDIAG_IGNITION_IRQ,

    //Tuareg_trigger_ignition()
    TDIAG_TRIG_IGN_CALLS,
    TDIAG_TRIG_COIL_DWELL1,
    TDIAG_TRIG_COIL_DWELL2,
    TDIAG_TRIG_COIL_IGN1,
    TDIAG_TRIG_COIL_IGN2,

    //Tuareg_update_process_data()
    TDIAG_PROCESSDATA_CALLS,

    //Tuareg_update_ignition_timing()
    TDIAG_IGNITIONCALC_CALLS,

    //Tuareg_update_Runmode()
    TDIAG_UPDATE_RUNMODE_CALLS,
    TDIAG_HALTSRC_PRESENT,
    TDIAG_HALTSRC_CLEAR,

    //Tuareg_set_Runmode()
    TDIAG_ENTER_INIT,
    TDIAG_ENTER_HALT,
    TDIAG_RUNNING_HALT_TR,
    TDIAG_STB_HALT_TR,
    TDIAG_INIT_HALT_TR,
    TDIAG_ENTER_RUNNING,
    TDIAG_CRANKING_RUNNING_TR,
    TDIAG_HALT_RUNNING_TR,
    TDIAG_ENTER_STB,
    TDIAG_RUNNING_STB_TR,
    TDIAG_CRANKING_STB_TR,
    TDIAG_HALT_STB_TR,
    TDIAG_ENTER_CRANKING,
    TDIAG_ENTER_MTEST,
    TDIAG_INVALID_RUNMODE,

    TDIAG_COUNT

} tuareg_diag_t;



void tuareg_diag_log_event(tuareg_diag_t event);
exec_result_t print_tuareg_diag(USART_TypeDef * Port);



#endif // TUAREG_DIAGNOSTICS_H_INCLUDED
