#ifndef TUNERSTUDIO_SERVICE_H_INCLUDED
#define TUNERSTUDIO_SERVICE_H_INCLUDED

#include "Tuareg_process_data.h"
#include "decoder_logic.h"


typedef enum {

    SACT_NONE,

    SACT_FUEL_PUMP,
    SACT_COIL_1,
    SACT_COIL_2,
    SACT_INJECTOR_1,
    SACT_INJECTOR_2,

    SACT_COUNT



} service_actor_t;



void ts_debug_features(U32 FeatureID);
void ts_debug_info(U32 InfoID);

void cli_show_process_data(volatile process_data_t * pImage);
void cli_show_ignition_controls(volatile ignition_control_t * pTiming);
void cli_show_decoder_interface(volatile Tuareg_decoder_t * pInterface);
void cli_print_sensor_data(USART_TypeDef * Port);

void request_service_mode();
void request_service_activation(U32 Actor, U32 On, U32 Off, U32 End);
#endif // TUNERSTUDIO_SERVICE_H_INCLUDED
