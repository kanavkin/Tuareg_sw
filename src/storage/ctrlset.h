
#ifndef CTRLSET_H
#define CTRLSET_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

#include "map.h"


/**
A control set holds all data for engine operation in a certain control regime e.g. MAP control or TPS control:
Ignition and fueling map data is stored efficiently for a common domain.

In future versions a control set could hold more than one map set to allow for economy / full power setup switching
*/
typedef struct _ctrlset_t {

    //first part - to be stored in eeprom
    volatile map_domain_t Dom;
    volatile mapset_codomains_t Cods;

    //second part - dynamic data
    volatile map_cache_t Cache;

} ctrlset_t;



exec_result_t ctrlset_load(volatile ctrlset_t * pCtrl, U32 BaseAddress);
exec_result_t ctrlset_store(volatile ctrlset_t * pCtrl, U32 BaseAddress);
exec_result_t ctrlset_modify(volatile ctrlset_t * pCtrl, U32 Offset, U32 Value);

void ctrlset_show(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl);
void ctrlset_send(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl);

#endif // CTRLSET_H
