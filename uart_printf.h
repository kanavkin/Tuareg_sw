#ifndef UARTPRINTF_H_INCLUDED
#define UARTPRINTF_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "Tuareg_types.h"


typedef enum {

    NO_PAD =0,
    PAD_2 =(1 << 1),
    PAD_3 =(1 << 2),
    PAD_4 =(1 << 3),
    PAD_5 =(1 << 4),
    PAD_10 =(1 << 5),

    NO_TRAIL =(1 << 6),

    TO_HEX =(1 << 7),

} printf_format_t;


void printf_S(USART_TypeDef * Port, S32 Value, BF32 Format);
void printf_U(USART_TypeDef * Port, U32 Value, BF32 Format);
void printf_F32(USART_TypeDef * Port, F32 Value);
void printf_crkpos(USART_TypeDef * Port, crank_position_t Position);
void printf_phase(USART_TypeDef * Port, engine_phase_t Phase);

void print(USART_TypeDef * Port, char messg[] );

void Print_U8Hex(USART_TypeDef * Port, U8 value);
void printf_U8(USART_TypeDef * Port, U32 Value);

void printf_U32hex(USART_TypeDef * Port, U32 value);

#endif // UARTPRINTF_H_INCLUDED
