
#ifndef MAPSET_H
#define MAPSET_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

#include "map.h"



/**
A mapset provides a container to store ignition and fueling data together. They correspond to the
domain of its control set
*/
typedef struct __attribute__ ((__packed__)) _mapset_codomains_t {

    map_codomain_t IgnAdv;
    map_codomain_t VE;
    map_codomain_t AFRtgt;

} mapset_codomains_t;


/**
map set interpolation request transfer object
*/
typedef struct _mapset_req_t {

    //arguments
    F32 X;
    F32 Y;

    //results
    F32 IgnAdv;
    F32 VE;
    F32 AFRtgt;

} mapset_req_t;



exec_result_t mapset_get(volatile mapset_t * pSet, volatile mapset_req_t * pReq);



#endif // MAP_H
