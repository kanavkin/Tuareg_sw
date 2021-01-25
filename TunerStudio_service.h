#ifndef TUNERSTUDIO_SERVICE_H_INCLUDED
#define TUNERSTUDIO_SERVICE_H_INCLUDED

#include "Tuareg_process_data.h"
#include "decoder_logic.h"

void ts_service_features(U32 FeatureID);

void cli_show_process_data(volatile process_data_t * pImage);
void cli_show_ignition_timing(volatile ignition_control_t * pTiming);
void cli_show_decoder_interface(volatile decoder_interface_t * pInterface);

#endif // TUNERSTUDIO_SERVICE_H_INCLUDED
