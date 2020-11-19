#ifndef TUAREGTYPES_H_INCLUDED
#define TUAREGTYPES_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "trigger_wheel_layout.h"


/**
basic types to handle a return value safely
*/
typedef enum {

    EXEC_ERROR,
    EXEC_OK

} exec_result_t;


/**
basic types to handle an actors state
*/
typedef enum {

    ACTOR_UNPOWERED,
    ACTOR_POWERED

} actor_control_t;





#define RETURN_OK 0
#define RETURN_FAIL 0xFF



typedef struct {

    VU16 crank_angle_deg[CRK_POSITION_COUNT];

} crank_position_table_t;


typedef enum {

    PHASE_CYL1_COMP,
    PHASE_CYL1_EX,
    PHASE_UNDEFINED,
    PHASE_COUNT,

} engine_phase_t;


typedef enum {

    ALPHA_N,
    SPEED_DENS

} ctrl_strategy_t;


/**
basic type to handle an absolute angular difference in degree

variables of this type have to be designated with xxxx_PD
*/
typedef uint32_t angle_deg_t;




#endif // TUAREGTYPES_H_INCLUDED
