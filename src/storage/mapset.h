
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
- internal interface -
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



/****************************************************************
user API

the user sees only the map object and performs the

basic operations
- map_get()
- map_load()
- map_store()
- map_modify()

to it
****************************************************************/

typedef struct _mapset_t {

    //first part - to be stored in eeprom
    volatile map_domain_t Dom;
    volatile mapset_codomains_t Cods;

    //second part - dynamic data
    volatile map_cache_t Cache;

} mapset_t;


//maps
exec_result_t map_get(volatile map_t *  pMap, F32 X, F32 Y, VF32 * pResult);

exec_result_t map_load(volatile map_t * pMap, U32 BaseAddress);
exec_result_t map_store(volatile map_t * pMap, U32 BaseAddress);

exec_result_t map_modify(volatile map_t * pMap, U32 Offset, U32 Value);

void send_map(USART_TypeDef * pPort, volatile map_t * pMap);
void show_map(USART_TypeDef * pPort, volatile map_t * pMap);


//map sets
exec_result_t mapset_get(volatile mapset_t * pSet, volatile mapset_req_t * pReq);



#endif // MAP_H
