#ifndef TUAREG_FUELING_CONTROLS_H_INCLUDED
#define TUAREG_FUELING_CONTROLS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"





/**
ignition_logic_state_t
*/
typedef union
{
     U16 all_flags;

     struct
     {
        U16 valid :1;


     };

} fueling_logic_flags_t;



/**
ignition_control_t defines a transfer object
*/
typedef struct _fueling_control_t {


    //status data
    fueling_logic_flags_t flags;

} fueling_control_t;




void Tuareg_update_fueling_controls();





#endif // TUAREG_FUELING_CONTROLS_H_INCLUDED
