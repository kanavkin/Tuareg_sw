#ifndef TUAREG_SERVICE_FUNCTIONS_H_INCLUDED
#define TUAREG_SERVICE_FUNCTIONS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_ignition.h"


#define SERVICE_FUELPUMP_MAX_TIMEOUT_S 180

#define SERVICE_ACTOR_MAX_ON_MS 1000
#define SERVICE_ACTOR_MAX_OFF_MS 50000
#define SERVICE_ACTOR_MAX_CYCLES 5000


typedef struct _service_flags_t {

        U8 fuel_pump_control :1;
        U8 coil1_control :1;
        U8 coil2_control :1;
        U8 injector1_control :1;
        U8 injector2_control :1;

} service_flags_t;


typedef struct _service_mgr_t {

    timestamp_t fuel_pump_timeout;

    U32 injector1_on_ms;
    U32 injector1_off_ms;
    timestamp_t injector1_toggle;
    U32 injector1_cycle_count;

    U32 injector2_on_ms;
    U32 injector2_off_ms;
    timestamp_t injector2_toggle;
    U32 injector2_cycle_count;

    U32 coil1_on_ms;
    U32 coil1_off_ms;
    timestamp_t coil1_toggle;
    U32 coil1_cycle_count;

    U32 coil2_on_ms;
    U32 coil2_off_ms;
    timestamp_t coil2_toggle;
    U32 coil2_cycle_count;

    service_flags_t flags;

} service_mgr_t;


void init_service_functions();
void service_functions_periodic_update();

void activate_fuel_pump(U32 Timeout_s);
void fuel_pump_periodic_update(U32 now);

void activate_injector1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void injector1_periodic_update(U32 now);

void activate_injector2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void injector2_periodic_update(U32 now);

void activate_coil1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void coil1_periodic_update(U32 now);

void activate_coil2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void coil2_periodic_update(U32 now);




#endif // TUAREG_SERVICE_FUNCTIONS_H_INCLUDED
