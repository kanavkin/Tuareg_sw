#ifndef TUAREG_SERVICE_FUNCTIONS_H_INCLUDED
#define TUAREG_SERVICE_FUNCTIONS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_ignition.h"


#define SERVICE_FUELPUMP_MAX_TIMEOUT_S 180

#define SERVICE_ACTOR_MAX_ON_MS 1000
#define SERVICE_ACTOR_MAX_OFF_MS 1000

#define SERVICE_ACTOR_MAX_CYCLES 0

#define SERVICE_INJECTOR_MAX_ONTIME 180
#define SERVICE_COIL_MAX_ONTIME 10


typedef enum {

    SACT_NONE,

    SACT_FUEL_PUMP,
    SACT_INJECTOR_1,
    SACT_INJECTOR_2,

    SACT_COIL_1,
    SACT_COIL_2,

    SACT_COUNT

} service_actor_t;


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
    U32 injector1_on_remain_ms;

    U32 injector2_on_ms;
    U32 injector2_off_ms;
    timestamp_t injector2_toggle;
    U32 injector2_on_remain_ms;

    U32 coil1_on_ms;
    U32 coil1_off_ms;
    timestamp_t coil1_toggle;
    U32 coil1_on_remain_ms;

    U32 coil2_on_ms;
    U32 coil2_off_ms;
    timestamp_t coil2_toggle;
    U32 coil2_on_remain_ms;

    service_flags_t flags;

} service_mgr_t;


void request_service_mode();
void request_service_activation(VU32 Actor, VU32 On, VU32 Off, VU32 End);

void init_service_functions();
void service_functions_periodic_update();

void activate_fuel_pump(VU32 Timeout_s);
void deactivate_fuel_pump();
void fuel_pump_periodic_update(U32 now);

void activate_injector1(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s);
void deactivate_injector1();
void injector1_periodic_update(VU32 now);

void activate_injector2(VU32 On_time_ms, VU32 Off_time_ms, VU32 On_target_s);
void deactivate_injector2();
void injector2_periodic_update(VU32 now);

void activate_coil1(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void deactivate_coil1();
void coil1_periodic_update(VU32 now);

void activate_coil2(U32 On_time_ms, U32 Off_time_ms, U32 Cycles);
void deactivate_coil2();
void coil2_periodic_update(VU32 now);

void activate_fatal();
void activate_reset();



#endif // TUAREG_SERVICE_FUNCTIONS_H_INCLUDED
