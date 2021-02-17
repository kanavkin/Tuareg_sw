#ifndef TUNERSTUDIO_SERVICE_H_INCLUDED
#define TUNERSTUDIO_SERVICE_H_INCLUDED

#include "Tuareg_process_data.h"
#include "decoder_logic.h"

void ts_service_features(U32 FeatureID);
void ts_service_info(U32 InfoID);

void cli_show_process_data(volatile process_data_t * pImage);
void cli_show_ignition_controls(volatile ignition_control_t * pTiming);
void cli_show_decoder_interface(volatile Tuareg_decoder_t * pInterface);
void cli_print_sensor_data(USART_TypeDef * Port);

#endif // TUNERSTUDIO_SERVICE_H_INCLUDED
