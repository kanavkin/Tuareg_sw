/**
This module holds constants for the Tuareg application layer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"

const F32 cKelvin_offset= 273.15;

// gas constant 8.31446261815324 (kg * m²) / (s² * K * mol)
const F32 cR_gas= 8.31446;

//molar mass of air in mg per mol (default: 28970)
const F32 cM_air= 28970.0;
