#ifndef TUAREGTYPES_H_INCLUDED
#define TUAREGTYPES_H_INCLUDED

#include "stm32_libs/boctok_types.h"



#define RETURN_OK 0
#define RETURN_FAIL 0xFF



typedef enum {

    CYL1_WORK,
    CYL2_WORK,
    PHASE_UNDEFINED

} engine_phase_t;


typedef enum {

    ALPHA_N,
    SPEED_DENS

} ctrl_strategy_t;



#endif // TUAREGTYPES_H_INCLUDED
