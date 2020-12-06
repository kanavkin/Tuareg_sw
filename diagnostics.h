#ifndef DIAGNOSTICS_H_INCLUDED
#define DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"

#define USE_DIAGNOSTICS

#define DIAG_PRINT_LEGEND_COLUMNS 5


typedef enum {

    SCHEDIAG_DELAY_CLIPPED,
    SCHEDIAG_DELAY_BYPASS,

    SCHEDIAG_ICH1_SET,
    SCHEDIAG_ICH1_CURRC_SET,
    SCHEDIAG_ICH1_NEXTC_PRELOAD_SET,
    SCHEDIAG_ICH1_NEXTC_UPDATE_SET,
    SCHEDIAG_ICH2_SET,
    SCHEDIAG_FCH1_SET,
    SCHEDIAG_FCH2_SET,

    SCHEDIAG_ICH1_TRIG,
    SCHEDIAG_ICH2_TRIG,
    SCHEDIAG_FCH1_TRIG,
    SCHEDIAG_FCH2_TRIG,

    SCHEDIAG_ICH1_RESET,
    SCHEDIAG_ICH2_RESET,
    SCHEDIAG_FCH1_RESET,
    SCHEDIAG_FCH2_RESET,

    SCHEDIAG_ICH1_RETRIGD,
    SCHEDIAG_ICH2_RETRIGD,
    SCHEDIAG_FCH1_RETRIGD,
    SCHEDIAG_FCH2_RETRIGD,

    SCHEDIAG_COUNT

} scheduler_diag_t;


typedef enum {


    IGNHWDIAG_SWIER3_TRIGGERED,

    IGNHWDIAG_COUNT

} ignhw_diag_t;

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


    //main()
    TDIAG_TSTUDIO_CALLS,

    //Tuareg_update_Runmode()
    TDIAG_MODECTRL,
    TDIAG_KILL_SIDESTAND,
    TDIAG_KILL_RUNSWITCH,

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



typedef enum {

    //decoder_logic_crank_handler()
    DDIAG_HW_EXTI0_CALLS,
    DDIAG_CRANKHANDLER_CALLS,
    DDIAG_CRANKPOS_INIT,
    DDIAG_CRANKPOS_SYNC,
    DDIAG_CRANKPOS_ASYNC,
    DDIAG_CRANKPOS_ASYNC_KEY,
    DDIAG_CRANKPOS_ASYNC_GAP,
    DDIAG_ASYNC_SYNC_TR,
    DDIAG_SYNC_ASYNC_TR,


    DDIAG_HW_SWIER2_CALLS,
    DDIAG_TRIGGER_IRQ_SYNC,

    DDIAG_HW_TIM9_CC1_CALLS,
    DDIAG_TIMER_COMPARE_EVENTS,



    DDIAG_HW_TIM9_UE_CALLS,
    DDIAG_TIMER_UPDATE_EVENTS,
    DDIAG_TIMEOUT_EVENTS,

    DDIAG_SYNCCHECK_CALLS,
    DDIAG_SYNCCHECK_SUCCESS,
    DDIAG_SYNCCHECK_FAILED,

    DDIAG_CRANKTABLE_CALLS,
    DDIAG_ROTSPEED_CALLS,

    DDIAG_HW_EXTI1_CALLS,

    DDIAG_COUNT

} decoder_diag_t;





void scheduler_diag_log_event(scheduler_diag_t event);
void print_scheduler_diag(USART_TypeDef * Port);
void print_scheduler_diag_legend(USART_TypeDef * Port);

void ignhw_diag_log_event(ignhw_diag_t event);
void print_ignhw_diag(USART_TypeDef * Port);
void print_ignhw_diag_legend(USART_TypeDef * Port);

void tuareg_diag_log_event(tuareg_diag_t event);
void tuareg_diag_log_parameter(tuareg_diag_t Parameter, U32 Value);
void print_tuareg_diag(USART_TypeDef * Port);
void print_tuareg_diag_legend(USART_TypeDef * Port);

void decoder_diag_log_event(decoder_diag_t Event);
void decoder_diag_log_parameter(decoder_diag_t Parameter, U32 Value);
void print_decoder_diag(USART_TypeDef * Port);
void print_decoder_diag_legend(USART_TypeDef * Port);
#endif // DIAGNOSTICS_H_INCLUDED
