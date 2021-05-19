#ifndef KNOCK_SENSOR_H_INCLUDED
#define KNOCK_SENSOR_H_INCLUDED


#include "stm32_libs/boctok_types.h"



#define PSC_SDO_CMD 0x40

#define CHANNEL_1_CMD 0xE0
#define CHANNEL_2_CMD 0xE1

#define BANDPASS_CMD 0x0

#define GAIN_CMD 0x80

#define INTEGR_CMD 0xC0

#define ADV_MODE_CMD 0x71


#define INTEGRATOR_PARAMETERS_SIZE 32
#define GAIN_PARAMETERS_SIZE 64
#define BAND_PARAMETERS_SIZE 64



/*
TPIC8101

pre scaler D[4:1]

0000 : 4 MHz
0001 : 5 MHz
0010 : 6 MHz
0011 ; 8 MHz
0100 ; 10 MHz
0101 ; 12 MHz
0110 : 16 MHz
0111 : 20 MHz
1000 : 24 MHz

SDO D[0]

0 : SDO active
1 : SDO high impedance

*/

/*
TPIC8101 integrator programming

v   -> Decimal Value (D4:D0)
Int -> Integrator Time Constant (μs)
BP  -> Band-Pass Frequency (kHz)

v  Int  BP      Gain        v   BP      Gain
0   40  1.22    2           32  4.95    0.421
1   45  1.26    1.882       33  5.12    0.4
2   50  1.31    1.778       34  5.29    0.381
3   55  1.35    1.684       35  5.48    0.364
4   60  1.4     1.6         36  5.68    0.348
5   65  1.45    1.523       37  5.9     0.333
6   70  1.51    1.455       38  6.12    0.32
7   75  1.57    1.391       39  6.37    0.308
8   80  1.63    1.333       40  6.64    0.296
9   90  1.71    1.28        41  6.94    0.286
10  100 1.78    1.231       42  7.27    0.276
11  110 1.87    1.185       43  7.63    0.267
12  120 1.96    1.143       44  8.02    0.258
13  130 2.07    1.063       45  8.46    0.25
14  140 2.18    1           46  8.95    0.236
15  150 2.31    0.944       47  9.5     0.222
16  160 2.46    0.895       48  10.12   0.211
17  180 2.54    0.85        49  10.46   0.2
18  200 2.62    0.81        50  10.83   0.19
19  220 2.71    0.773       51  11.22   0.182
20  240 2.81    0.739       52  11.65   0.174
21  260 2.92    0.708       53  12.1    0.167
22  280 3.03    0.68        54  12.6    0.16
23  300 3.15    0.654       55  13.14   0.154
24  320 3.28    0.63        56  13.72   0.148
25  360 3.43    0.607       57  14.36   0.143
26  400 3.59    0.586       58  15.07   0.138
27  440 3.76    0.567       59  15.84   0.133
28  480 3.95    0.548       60  16.71   0.129
29  520 4.16    0.5         61  17.67   0.125
30  560 4.39    0.471       62  18.76   0.118
31  600 4.66    0.444       63  19.98   0.111

*/


/*
TPIC8101 transfer function

V_out := V_in * A_prog * (8/pi) * (t/T_c) + 0.125

t -> knock window

*/



typedef struct _knock_sensor_controls_t {

    U32 integration_time_us;
    U32 integration_begin_timing_us;

    U32 tau_us;
    U32 tau_index;

    F32 gain;
    U32 gain_index;


} knock_sensor_controls_t ;






void SPI2_SendData(U32 Data);
U32 SPI2_ReadInputData();
void begin_Knock_integration();
void end_Knock_integration();
void deselect_Knock_CS();
void select_Knock_CS();



void update_knock_sensor_controls();
void set_integration_time_constant(U32 Index);
void update_integration_time_constant();

void set_gain_value(U32 Index);
void update_gain();


#endif // KNOCK_SENSOR_H_INCLUDED
