
#ifndef CTRLSET_H
#define CTRLSET_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

#include "map.h"


/**



*/
typedef struct _ctrlset_t {

    //first part - to be stored in eeprom
    volatile map_domain_t Dom;
    volatile mapset_t Set;

    //second part - dynamic data
    volatile map_cache_t Cache;

} ctrlset_t;




/**
control set look up / interpolation request transfer object
*/
typedef struct _ctrlset_req_t {

    //map data
    volatile map_domain_t * pDom;
    volatile map_codomain_t * pCod;
    volatile map_cache_t * pCache;

    //arguments
    F32 X;
    F32 Y;

    //result
    F32 Z;

} ctrlset_req_t;




exec_result_t map_get(volatile map_req_t *  pReq);

exec_result_t map_load(volatile map_t * pMap, U32 BaseAddress);

exec_result_t map_store(volatile map_t * pMap, U32 BaseAddress);

exec_result_t map_modify(volatile map_t * pMap, U32 Offset, U32 Value);

void send_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTable);


void show_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTableData);



#endif // CTRLSET_H
