#ifndef SENSORS_DIAGNOSTICS_H_INCLUDED
#define SENSORS_DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"



typedef enum {

    SNDIAG_UPDATE_DSENSORS_CALLS,
    SNDIAG_ADCIRQ_CALLS,
    SNDIAG_ADCIRQ_INJECTEDGR_CALLS,
    SNDIAG_DMAIRQ_CALLS,
    SNDIAG_DMAIRQ_CH1_CALLS,

    SNDIAG_COUNT

} sensors_diag_t;


void sensors_diag_log_event(sensors_diag_t event);
exec_result_t print_sensors_diag(USART_TypeDef * Port);

#endif // SENSORS_DIAGNOSTICS_H_INCLUDED
