

#include "stm32_libs/boctok_types.h"
#include "conversion.h"
#include "uart.h"

/**
needed for UART_Print functions:
puts one converted decimal place to desired uart port,
returns the new lead_zero value
*/
U32 uart_push_decimal_place(USART_TypeDef * Port, U32 number, U32 lead_zero, U32 padding)
{
    if(number)
    {
        //decimal place to print
        UART_Tx(Port, number + 0x30);

        //no longer leading zeros
        return 0;
    }
    else
    {
        if(lead_zero)
        {
            if(padding)
            {
                UART_Tx(Port, ' ');
            }

        }
        else
        {
            UART_Tx(Port, '0');
        }

        //lead_zero unchanged
        return lead_zero;
    }
}



void UART_Print_S(USART_TypeDef * Port, S32 value, conversion_int_t inttype, U32 padding)
{
    switch(inttype)
    {
        case TYPE_S32:
            inttype= TYPE_U32;
            break;

        case TYPE_S16:
            inttype= TYPE_U16;
            break;

        case TYPE_S8:
            inttype= TYPE_U8;
            break;

        default:
            return;

    }

    if(value < 0)
    {
        UART_Tx(Port, '-');
        UART_Print_U(Port, -value, inttype, padding);
    }
    else
    {
        UART_Print_U(Port, value, inttype, padding);
    }
}


/**
0xFFFFFFFF = 4 294 967 295
*/
void UART_Print_U(USART_TypeDef * Port, U32 value, conversion_int_t inttype, U32 padding)
{
    U32 number;
    U32 lead_zero= 0xFFFFFFFF;

    switch(inttype)
    {
        case TYPE_U32:

                //milliard
                for(number=0; value > 999999999; number++)
                {
                    value -= 1000000000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //hundred millions
                for(number=0; value > 99999999; number++)
                {
                    value -= 100000000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //ten millions
                for(number=0; value > 9999999; number++)
                {
                    value -= 10000000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //millions
                for(number=0; value > 999999; number++)
                {
                    value -= 1000000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //hundred thousands
                for(number=0; value > 99999; number++)
                {
                    value -= 100000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                /**
                fall through
                */

        case TYPE_U16:

                //ten thousands
                for(number=0; value > 9999; number++)
                {
                    value -= 10000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //thousands
                for(number=0; value > 999; number++)
                {
                    value -= 1000;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                /**
                fall through
                */

        case TYPE_U8:

                //hundert
                for(number=0; value > 99; number++)
                {
                    value -= 100;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                //ten
                for(number=0; value > 9; number++)
                {
                    value -= 10;
                }

                lead_zero= uart_push_decimal_place(Port, number, lead_zero, padding);

                /**
                print remainder
                */
                UART_Tx(Port, value + 0x30);


                /**
                print trailing space
                */
                UART_Tx(Port, ' ');

                break;

        default:
            return;


    }

}


#define PRINTF32_BUFLEN 10

/**
prints a number with 2 digits following the decimal place
creates the string backwards, before printing it character-by-character from
the end to the start
*/
void UART_Print_F32(USART_TypeDef * Port, F32 Value)
{
    U8 result[PRINTF32_BUFLEN];
    S32 dVal, dec;
    U32 i;

    if(Value < 0.0)
    {
        //negative number
        Value -= 0.005;

        dVal = -Value;
    }
    else
    {
        //added for rounding
        Value += 0.005;

        dVal = Value;
    }


    dec = (S32)(Value * 100) % 100;

    result[0] = (dec % 10) + '0';
    result[1] = (dec / 10) + '0';
    result[2] = '.';

    //proceed to the integer part
    i = 3;

    if(dVal == 0)
    {
        //nothing to convert
        result[i] = '0';
    }
    else
    {
        //loop through the positions
        while ((dVal > 0) && (i < PRINTF32_BUFLEN))
        {
            result[i] = (dVal % 10) + '0';
            dVal /= 10;
            i++;
        }

         //adjust i to the first significant place
        i--;
    }



    if(Value < 0.0)
    {
        //negative number
        UART_Tx(Port, '-');
    }

    while(i > 0)
    {
        UART_Tx(Port, result[i]);
        i--;
    }

    //last fractional place
    UART_Tx(Port, result[0]);

    //trailing whitespace
    UART_Tx(Port, ' ');
}



void UART_Print_U8Hex(USART_TypeDef * Port, U8 value)
{
    const char digits[]= "0123456789ABCDEF";

    UART_Send(Port, "0x");

    //high nibble
    UART_Tx(Port, digits[(value >> 4) & 0x0f] );

    //low nibble
    UART_Tx(Port, digits[(value) & 0x0f] );

    //trailing space
    UART_Tx(Port, ' ');
}



void UART_Print_crank_position(USART_TypeDef * Port, crank_position_t Position)
{
    switch(Position)
    {
    case CRK_POSITION_A1:
        UART_Send(Port, "A1");
        break;
    case CRK_POSITION_A2:
        UART_Send(Port, "A2");
        break;
    case CRK_POSITION_B1:
        UART_Send(Port, "B1");
        break;
    case CRK_POSITION_B2:
        UART_Send(Port, "B2");
        break;
    case CRK_POSITION_C1:
        UART_Send(Port, "C1");
        break;
    case CRK_POSITION_C2:
        UART_Send(Port, "C2");
        break;
    case CRK_POSITION_D1:
        UART_Send(Port, "D1");
        break;
    case CRK_POSITION_D2:
        UART_Send(Port, "D2");
        break;
    default:
        UART_Send(Port, "XX");
        break;
    }
}



U32 serialize_float_U32(float Value)
{
    union {

        float in;
        U32  out;

    } u;

    u.in = Value;

    return u.out;
}


//writes 4 bytes to pTarget, LSB first
void serialize_float_U8(float Value, U8 * pTarget)
{
    U32 i;

    union {

        float in;
        U8  out[4];

    } u;

    u.in = Value;

    for(i=0; i<4; i++)
    {
        pTarget[i]= u.out[i];
    }
}

//writes 2 bytes to pTarget, LSB first
void serialize_U16_U8(U16 Value, U8 * pTarget)
{
      *pTarget= (U8)(Value & 0x00FF);
      *(pTarget +1)= (U8)(Value >> 8);
}



float compose_float(U32 Buffer)
{
    union {

        float float_out;
        U32  in;

    } u;

    u.in = Buffer;

    return u.float_out;
}




U32 compose_U32(U8 Msb, U8 Mid_h, U8 Mid_l, U8 Lsb)
/// TODO (oli#9#): broken, data order?
{
    union {

        U32 out;
        U8  in[4];

    } u;

    u.in[0] = Msb;
    u.in[1] = Mid_h;
    u.in[2] = Mid_l;
    u.in[3] = Lsb;

    return u.out;
}

