#ifndef CONVERSION_H_INCLUDED
#define CONVERSION_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


/*
this is a generic 32 bit wide input variable
*/
typedef union
{
    F32 in_F32;
    U32 in_U32;
    U16  in_U16[2];
    U8  in_U8[4];
    crank_position_t in_position;
    engine_phase_t in_phase;

} converter_32_t;


crank_position_t parse_position(U32 Input);

U32 compose_U32(U8 Msb, U8 Mid_h, U8 Mid_l, U8 Lsb);

float compose_float(U32 Buffer);
U32 serialize_float_U32(float Value);
void send_float(USART_TypeDef * Port, float Value);

void serialize_float_U8(float Value, U8 * pTarget);
void serialize_U16_U8(U16 Value, U8 * pTarget);
void serialize_U32_char(VU32 Value, U8 * pTarget);

#endif // CONVERSION_H_INCLUDED
