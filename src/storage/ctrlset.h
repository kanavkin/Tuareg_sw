
#ifndef CTRLSET_H
#define CTRLSET_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

#include "map.h"

//map storage size (axis X, Y, and 3* axis Z)
#define CTRLSET_STORAGE_SIZE_B 2*MAP_DIM*MAP_DOM_CELL_SIZE_B + 3*MAP_DIM*MAP_DIM*MAP_CODOM_CELL_SIZE_B


/*****************************************************************************************************************
map set related data
*****************************************************************************************************************/

typedef enum {

    CTRLSET_MAP_STD,
    CTRLSET_TPS_STD,
    CTRLSET_TPS_LIMP,
    CTRLSET_COUNT

} ctrlset_designator_t;


/**
A mapset provides a container to store ignition and fueling data consistently together. They correspond to the
domain of its control set.
A control set can hold multiple map sets, e.g. a standard map, a fuel economy map, ...
*/
typedef struct __attribute__ ((__packed__)) _mapset_codomains_t {

    map_codomain_t IgnAdv;
    map_codomain_t VE;
    map_codomain_t AFRtgt;

} mapset_codomains_t;



/*****************************************************************************************************************
control set data
*****************************************************************************************************************/

/**
A control set holds all data for engine operation in a certain control regime e.g. MAP control or TPS control:
Ignition and fueling map data is stored efficiently for a common domain.
*/
typedef struct _ctrlset_t {

    //first part - to be stored in eeprom
    volatile map_domain_t Dom;
    volatile mapset_codomains_t Cods;

    //second part - dynamic data
    volatile map_cache_t Cache;

} ctrlset_t;



/**
control set interpolation request transfer object
*/
typedef struct _ctrlset_req_t {

    //arguments
    F32 X;
    F32 Y;

    //results
    F32 IgnAdv;
    F32 VE;
    F32 AFRtgt;

    bool valid;

} ctrlset_req_t;




exec_result_t ctrlset_load(volatile ctrlset_t * pCtrl, U32 BaseAddress);
exec_result_t ctrlset_store(volatile ctrlset_t * pCtrl, U32 BaseAddress);
exec_result_t ctrlset_modify(volatile ctrlset_t * pCtrl, U32 Offset, U32 Value);

exec_result_t ctrlset_get(volatile ctrlset_t * pSet, volatile ctrlset_req_t * pReq);

void ctrlset_show(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl);
void ctrlset_send(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl);

#endif // CTRLSET_H
