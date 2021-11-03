#ifndef IGNITION_DIAGNOSTICS_H_INCLUDED
#define IGNITION_DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


typedef enum {

    IGNDIAG_CRKPOSH_CALLS,
    IGNDIAG_CRKPOSH_INHIBIT,
    IGNDIAG_CRKPOSH_CTRLS_INVALID,
    IGNDIAG_CRKPOSH_IGNPOS,
    IGNDIAG_CRKPOSH_IGN1SCHED,
    IGNDIAG_CRKPOSH_IGN2SCHED,
    IGNDIAG_CRKPOSH_IGN1_UNPOWER,
    IGNDIAG_CRKPOSH_IGN2_UNPOWER,
    IGNDIAG_CRKPOSH_IGN1_POWER,
    IGNDIAG_CRKPOSH_IGN2_POWER,

    IGNDIAG_UPDIGNCTRL_CALLS,
    IGNDIAG_UPDIGNCTRL_REVLIM,
    IGNDIAG_UPDIGNCTRL_DYN,
    IGNDIAG_UPDIGNCTRL_DYN_FAIL,

    IGNDIAG_COUNT

} ignition_diag_t;


void ignition_diag_log_event(ignition_diag_t event);
exec_result_t print_ignition_diag(USART_TypeDef * Port);


#endif // IGNITION_DIAGNOSTICS_H_INCLUDED
