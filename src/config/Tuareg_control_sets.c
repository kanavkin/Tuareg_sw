#include "Tuareg.h"

volatile ctrlset_t Control_MAP;
volatile ctrlset_t Control_TPS;
volatile ctrlset_t Control_TPS_Limp;


/***************************************************************************************************************************************************
*   Control Set MAP
***************************************************************************************************************************************************/

exec_result_t load_Control_MAP()
{
    return ctrlset_load(&Control_MAP, EEPROM_CTRLSET_MAP_BASE);
}

exec_result_t store_Control_MAP()
{
    return ctrlset_store(&Control_MAP, EEPROM_CTRLSET_MAP_BASE);
}

exec_result_t modify_Control_MAP(U32 Offset, U32 Value)
{
    return ctrlset_modify(&Control_MAP, Offset, Value);
}

void show_Control_MAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nMAP based control set:");

    ctrlset_show(Port, &Control_MAP);
}

void send_Control_MAP(USART_TypeDef * Port)
{
    ctrlset_send(Port, &Control_MAP);
}


/***************************************************************************************************************************************************
*   Control Set TPS
***************************************************************************************************************************************************/

exec_result_t load_Control_TPS()
{
    return ctrlset_load(&Control_TPS, EEPROM_CTRLSET_TPS_BASE);
}

exec_result_t store_Control_TPS()
{
    return ctrlset_store(&Control_TPS, EEPROM_CTRLSET_TPS_BASE);
}

exec_result_t modify_Control_TPS(U32 Offset, U32 Value)
{
    return ctrlset_modify(&Control_TPS, Offset, Value);
}

void show_Control_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nTPS based control set:");

    ctrlset_show(Port, &Control_TPS);
}

void send_Control_TPS(USART_TypeDef * Port)
{
    ctrlset_send(Port, &Control_TPS);
}



/***************************************************************************************************************************************************
*   Control Set TPS in LIMP mode
***************************************************************************************************************************************************/

exec_result_t load_Control_TPS_Limp()
{
    return ctrlset_load(&Control_TPS_Limp, EEPROM_CTRLSET_TPS_LIMP_BASE);
}

exec_result_t store_Control_TPS_Limp()
{
    return ctrlset_store(&Control_TPS_Limp, EEPROM_CTRLSET_TPS_LIMP_BASE);
}

exec_result_t modify_Control_TPS_Limp(U32 Offset, U32 Value)
{
    return ctrlset_modify(&Control_TPS_Limp, Offset, Value);
}

void show_Control_TPS_Limp(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nTPS based control set for LIMP mode:");

    ctrlset_show(Port, &Control_TPS_Limp);
}

void send_Control_TPS_Limp(USART_TypeDef * Port)
{
    ctrlset_send(Port, &Control_TPS_Limp);
}


