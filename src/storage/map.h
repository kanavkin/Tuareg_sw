
#ifndef MAP_H
#define MAP_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"


/**
common dimensions for all maps
(X, Y, Z axis)
*/
#define MAP_DIM 16


/**
This is the data container for the 3D map axes X and Y
*/
typedef struct __attribute__ ((__packed__)) _map_domain_t {

    U16 axisX[MAP_DIM];
    U16 axisY[MAP_DIM];

    U16 X_min_valid;
    U16 X_max_valid;

    U16 Y_min_valid;
    U16 Y_max_valid;

} map_domain_t;


/**
This is the data container for the 3D map axisZ
Z = f(X, Y)
*/
typedef struct __attribute__ ((__packed__)) _map_codomain_t {

    U8 axisZ [MAP_DIM] [MAP_DIM];

    U16 Z_min_valid;
    U16 Z_max_valid;

} map_codomain_t;


/**
map cache
Store the X and Y interval from the last request to make the next access faster
*/
typedef struct _map_cache_t {

    U32 last_Xmax_index;
    U32 last_Ymax_index;

} map_cache_t;


typedef struct _map_t {

    //first part - to be stored in eeprom
    volatile map_domain_t Dom;
    volatile map_codomain_t Cod;

    //second part - dynamic data
    volatile map_cache_t Cache;

} map_t;


/**
map domain look up transfer object
*/
typedef struct _map_domain_req_t {

    //arguments
    F32 X;
    F32 Y;

    //interval
    F32 xMin;
    F32 xMax;
    F32 yMin;
    F32 yMax;

    U32 xMin_index;
    U32 xMax_index;
    U32 yMin_index;
    U32 yMax_index;

} map_domain_req_t;



exec_result_t map_get(volatile map_t *  pMap, F32 X, F32 Y, VF32 * pResult);

exec_result_t map_load(volatile map_t * pMap, U32 BaseAddress);
exec_result_t map_store(volatile map_t * pMap, U32 BaseAddress);
exec_result_t map_modify(volatile map_t * pMap, U32 Offset, U32 Value);
void map_send(USART_TypeDef * pPort, volatile map_t * pMap);

void map_show_axes(USART_TypeDef * pPort, volatile map_domain_t * pDom, volatile map_codomain_t * pCod);
void map_show(USART_TypeDef * pPort, volatile map_t * pMap);



//helper functions
exec_result_t map_get_domain_x(volatile map_domain_t * pDom, volatile map_cache_t * pCache, volatile map_domain_req_t * pDomReq, F32 X);
exec_result_t map_get_domain_y(volatile map_domain_t * pDom, volatile map_cache_t * pCache, volatile map_domain_req_t * pDomReq, F32 Y);
exec_result_t map_interpolate(volatile map_domain_req_t * pDomReq, volatile map_codomain_t * pCod, VF32 * pResult);

#endif // MAP_H
