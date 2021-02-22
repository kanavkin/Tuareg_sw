

#include "stm32_libs/boctok_types.h"
#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"
#include "bitfields.h"

const char printf_hex_digits[]= "0123456789ABCDEF";


void printf_S(USART_TypeDef * Port, S32 Value, BF32 Format)
{
    if(Value < 0)
    {
        UART_Tx(Port, '-');
        Value= -Value;
    }

    printf_U(Port, Value, Format);
}


/**
0xFFFFFFFF = 4 294 967 295
*/
void printf_U(USART_TypeDef * Port, U32 Value, BF32 Format)
{
    U32 number, place;
    U32 min_place =9;
    U8 result[10], decimal;
    bool lead_zero= true;


    /*
    create decimal string
    */

    //milliard
    for(number=0; Value > 999999999; number++)
    {
        Value -= 1000000000;
    }

    result[0]= number;

    //hundred millions
    for(number=0; Value > 99999999; number++)
    {
        Value -= 100000000;
    }

    result[1]= number;

    //ten millions
    for(number=0; Value > 9999999; number++)
    {
        Value -= 10000000;
    }

    result[2]= number;

    //millions
    for(number=0; Value > 999999; number++)
    {
        Value -= 1000000;
    }

    result[3]= number;

    //hundred thousand
    for(number=0; Value > 99999; number++)
    {
        Value -= 100000;
    }

    result[4]= number;

    //ten thousand
    for(number=0; Value > 9999; number++)
    {
        Value -= 10000;
    }

    result[5]= number;

    //thousands
    for(number=0; Value > 999; number++)
    {
        Value -= 1000;
    }

    result[6]= number;

    //hundred
    for(number=0; Value > 99; number++)
    {
        Value -= 100;
    }

    result[7]= number;

    //tens
    for(number=0; Value > 9; number++)
    {
        Value -= 10;
    }

    result[8]= number;

    //one
    result[9]= Value;


    /*
    get the first decimal place that has to be printed due to padding
    (min_place := 10 - padding)
    */
    if( Format & PAD_10)
    {
        min_place =0;
    }
    else if (Format & PAD_5)
    {
        min_place =5;
    }
    else if (Format & PAD_4)
    {
        min_place =6;
    }
    else if (Format & PAD_3)
    {
        min_place =7;
    }
    else if (Format & PAD_2)
    {
        min_place =8;
    }



    //loop through decimal places
    for(place=0; place < 10; place++)
    {
        decimal= result[place];

        // print any number that is not zero and any zero that occurs in the middle of the string
        if((decimal) || (lead_zero == false) || (place == 9))
        {
            //print decimal place in ascii
            UART_Tx(Port, decimal + 0x30);

            //met an occupied decimal place or lead_zero was already false
            lead_zero= false;
        }
        else if((lead_zero) && (place >= min_place))
        {
            //print leading spaces for padding
            UART_Tx(Port, ' ');
        }

    }

    if( !(Format & NO_TRAIL))
    {
        //print trailing space
        UART_Tx(Port, ' ');
    }

}


#define PRINTF32_BUFLEN 10

/**
prints a number with 2 digits following the decimal place
creates the string backwards, before printing it character-by-character from
the end to the start
*/
void printf_F32(USART_TypeDef * Port, F32 Value)
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



inline void printf_nib(USART_TypeDef * Port, U32 Value)
{
    //print only low nibble
    UART_Tx(Port, printf_hex_digits[Value & 0x0f]);
}

void printf_U8hex_new(USART_TypeDef * Port, U8 value)
{
    print(Port, "0x");

    //high nibble
    printf_nib(Port, (value >> 4));

    //low nibble
    printf_nib(Port, value);

    //trailing space
    UART_Tx(Port, ' ');
}


void Print_U8Hex(USART_TypeDef * Port, U8 value)
{
    print(Port, "0x");

    //high nibble
    UART_Tx(Port, printf_hex_digits[(value >> 4) & 0x0f] );

    //low nibble
    UART_Tx(Port, printf_hex_digits[(value) & 0x0f] );

    //trailing space
    UART_Tx(Port, ' ');
}


void printf_U32hex(USART_TypeDef * Port, U32 value)
{
    U32 i;

    print(Port, "0x");

    for(i=0; i<8; i++)
    {
        printf_nib(Port, (value >> (28 - 4*i)) );
    }

    //trailing space
    UART_Tx(Port, ' ');
}


void printf_crkpos(USART_TypeDef * Port, crank_position_t Position)
{
    switch(Position)
    {
    case CRK_POSITION_A1:
        print(Port, "A1");
        break;
    case CRK_POSITION_A2:
        print(Port, "A2");
        break;
    case CRK_POSITION_B1:
        print(Port, "B1");
        break;
    case CRK_POSITION_B2:
        print(Port, "B2");
        break;
    case CRK_POSITION_C1:
        print(Port, "C1");
        break;
    case CRK_POSITION_C2:
        print(Port, "C2");
        break;
    case CRK_POSITION_D1:
        print(Port, "D1");
        break;
    case CRK_POSITION_D2:
        print(Port, "D2");
        break;
    default:
        print(Port, "XX");
        break;
    }

    //trailing space
    //UART_Tx(Port, ' ');
}

void printf_phase(USART_TypeDef * Port, engine_phase_t Phase)
{
    switch(Phase)
    {
    case PHASE_CYL1_COMP:
        print(Port, "COMP");
        break;

    case PHASE_CYL1_EX:
        print(Port, "EX");
        break;

    default:
        print(Port, "XX");
        break;
    }

    //trailing space
    UART_Tx(Port, ' ');
}


void printf_decoder_sensing(USART_TypeDef * Port, decoder_sensing_t Sensing)
{
    switch(Sensing)
    {
    case SENSING_DISABLED:
        print(Port, "DISABLED");
        break;

    case SENSING_RISE:
        print(Port, "RISE");
        break;

    case SENSING_FALL:
        print(Port, "FALL");
        break;

    case SENSING_EDGE:
        print(Port, "EDGE");
        break;

    case SENSING_INVERT:
        print(Port, "INVERT");
        break;

    default:
        print(Port, "XX");
        break;
    }

    //trailing space
    UART_Tx(Port, ' ');
}


/**
minimalistic U8 printout (padded)
*/
void printf_U8(USART_TypeDef * Port, U32 Value)
{
    U32 number;

    //hundred
    for(number=0; Value > 99; number++)
    {
        Value -= 100;
    }

    UART_Tx(Port, number + 0x30);

    //tens
    for(number=0; Value > 9; number++)
    {
        Value -= 10;
    }

    UART_Tx(Port, number + 0x30);

    //one
    UART_Tx(Port, Value + 0x30);
}





/***************************************************************************************************************************************
new standard uart printout frontend
***************************************************************************************************************************************/
/*
void printf(USART_TypeDef * Port, U32 Input, conversion_input_t Type, conversion_format_t Format)
{







}
*/


/***************************************************************************************************************************************
new standard uart printout helper functions
***************************************************************************************************************************************/
void print(USART_TypeDef * Port, char messg[] )
{
    for( ; *messg != 0 ; messg++)
    {
        UART_Tx(Port, *messg);
    }
}


