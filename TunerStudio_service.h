#ifndef TUNERSTUDIO_SERVICE_H_INCLUDED
#define TUNERSTUDIO_SERVICE_H_INCLUDED


void ts_service_features(U32 FeatureID);

void cli_show_process_data(volatile process_data_t * pImage);
void cli_show_ignition_timing(volatile ignition_control_t * pTiming);

#endif // TUNERSTUDIO_SERVICE_H_INCLUDED
