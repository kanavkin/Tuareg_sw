#ifndef DIAGNOSTICS_H_INCLUDED
#define DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"

#define USE_DIAGNOSTICS

#define DIAG_PRINT_LEGEND_COLUMNS 5


typedef enum {

    SCHEDIAG_ICH1_SET,
    SCHEDIAG_ICH1_CURRC_SET,
    SCHEDIAG_ICH1_NEXTC_PRELOAD_SET,
    SCHEDIAG_ICH1_NEXTC_UPDATE_SET,
    SCHEDIAG_ICH1_RETRIGD,
    SCHEDIAG_ICH1_TRIG,
    SCHEDIAG_ICH1_RESET,

    SCHEDIAG_ICH2_SET,
    SCHEDIAG_ICH2_CURRC_SET,
    SCHEDIAG_ICH2_NEXTC_PRELOAD_SET,
    SCHEDIAG_ICH2_NEXTC_UPDATE_SET,
    SCHEDIAG_ICH2_RETRIGD,
    SCHEDIAG_ICH2_TRIG,
    SCHEDIAG_ICH2_RESET,

    SCHEDIAG_FCH1_SET,
    SCHEDIAG_FCH1_CURRC_SET,
    SCHEDIAG_FCH1_NEXTC_PRELOAD_SET,
    SCHEDIAG_FCH1_NEXTC_UPDATE_SET,
    SCHEDIAG_FCH1_RETRIGD,
    SCHEDIAG_FCH1_TRIG,
    SCHEDIAG_FCH1_RESET,

    SCHEDIAG_FCH2_SET,
    SCHEDIAG_FCH2_CURRC_SET,
    SCHEDIAG_FCH2_NEXTC_PRELOAD_SET,
    SCHEDIAG_FCH2_NEXTC_UPDATE_SET,
    SCHEDIAG_FCH2_RETRIGD,
    SCHEDIAG_FCH2_TRIG,
    SCHEDIAG_FCH2_RESET,

    SCHEDIAG_DELAY_MININT1,
    SCHEDIAG_DELAY_MININT2,
    SCHEDIAG_DELAY_BYPASS,

    SCHEDIAG_COUNT

} scheduler_diag_t;




typedef enum {

    IGNDIAG_CRKPOSH_CALLS,
    IGNDIAG_CRKPOSH_PRECOND_FAIL,
    IGNDIAG_CRKPOSH_IGNPOS,
    IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER,
    IGNDIAG_CRKPOSH_IGN2SCHED_UNPOWER,
    IGNDIAG_CRKPOSH_IGN1_UNPOWER,
    IGNDIAG_CRKPOSH_IGN2_UNPOWER,
    IGNDIAG_CRKPOSH_IGN1_POWER,
    IGNDIAG_CRKPOSH_IGN2_POWER,

    IGNDIAG_IRQ3H_CALLS,
    IGNDIAG_IRQ3H_PRECOND_FAIL,
    IGNDIAG_IRQ3H_IGN1SCHED_POWER,
    IGNDIAG_IRQ3H_IGN2SCHED_POWER,
    IGNDIAG_IRQ3H_IGN1_POWER,
    IGNDIAG_IRQ3H_IGN2_POWER,

    IGNDIAG_UPDIGNCTRL_CALLS,
    IGNDIAG_UPDIGNCTRL_REVLIM,
    IGNDIAG_UPDIGNCTRL_DYN,
    IGNDIAG_UPDIGNCTRL_DYN_FAIL,

    IGNDIAG_COUNT

} ignition_diag_t;




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



typedef enum {

    DDIAG_CRK_EXTI_EVENTS,

    DDIAG_CRKPOS_INIT,
    DDIAG_CRKPOS_ASYNC,
    DDIAG_CRKPOS_ASYNC_KEY,
    DDIAG_CRKPOS_ASYNC_GAP,
    DDIAG_CRKPOS_SYNC,

    DDIAG_SYNCHK_ASYN_FAIL,
    DDIAG_SYNCHK_ASYN_PASS,
    DDIAG_SYNCHK_SYN_FAIL,
    DDIAG_SYNCHK_SYN_PASS,

    DDIAG_UPDATE_IRQ_CALLS,



    DDIAG_CRK_NOISEF_EVENTS,



    DDIAG_CAM_NOISEF_EVENTS,


    DDIAG_TIM_UPDATE_EVENTS,
    DDIAG_TIMEOUT_EVENTS,


    DDIAG_CAM_EXTI_EVENTS,
    DDIAG_CISHDL_PRECOND_FAIL,

    DDIAG_CISUPD_PRECOND_FAIL,
    DDIAG_CISUPD_INTERVAL_FAIL,
    DDIAG_CISUPD_PHASE_FAIL,
    DDIAG_CISUPD_PHASE_PASS,



    DDIAG_COUNT

} decoder_diag_t;



typedef enum {

    SNDIAG_READ_DSENSORS_CALLS,
    SNDIAG_ADCIRQ_CALLS,
    SNDIAG_ADCIRQ_INJECTEDGR_CALLS,
    SNDIAG_DMAIRQ_CALLS,
    SNDIAG_DMAIRQ_CH1_CALLS,

    SNDIAG_COUNT

} sensors_diag_t;



void scheduler_diag_log_event(scheduler_diag_t event);
void print_scheduler_diag(USART_TypeDef * Port);
void print_scheduler_diag_legend(USART_TypeDef * Port);

void ignition_diag_log_event(ignition_diag_t event);
void print_ignition_diag(USART_TypeDef * Port);
void print_ignition_diag_legend(USART_TypeDef * Port);

void tuareg_diag_log_event(tuareg_diag_t event);
void print_tuareg_diag(USART_TypeDef * Port);
void print_tuareg_diag_legend(USART_TypeDef * Port);

void decoder_diag_log_event(decoder_diag_t Event);
void print_decoder_diag(USART_TypeDef * Port);
void print_decoder_diag_legend(USART_TypeDef * Port);

void sensors_diag_log_event(sensors_diag_t event);
void print_sensors_diag(USART_TypeDef * Port);
void print_sensors_diag_legend(USART_TypeDef * Port);

#endif // DIAGNOSTICS_H_INCLUDED
