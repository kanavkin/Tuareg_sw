#ifndef TUNERSTUDIO_SERVICE_H_INCLUDED
#define TUNERSTUDIO_SERVICE_H_INCLUDED

#include "Tuareg_process_data.h"

#include "Tuareg_decoder.h"



void ts_debug_features(U32 FeatureID);
void ts_debug_info(U32 InfoID, USART_TypeDef * Port);

void cli_show_process_data(volatile process_data_t * pImage);
void cli_show_ignition_controls(volatile ignition_controls_t * pTiming);
void cli_show_decoder_interface(volatile decoder_output_t * pInterface);
void cli_print_sensor_data(USART_TypeDef * Port);

#endif // TUNERSTUDIO_SERVICE_H_INCLUDED
