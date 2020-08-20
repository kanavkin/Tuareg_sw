#ifndef EEPROM_LAYOUT_H_INCLUDED
#define EEPROM_LAYOUT_H_INCLUDED

/**
Current layout of EEPROM data (Version 3) is as follows (All sizes are in bytes):
|---------------------------------------------------|
|Byte # |Size | Description                         |
|---------------------------------------------------|
| 0     |1    | Data structure version              |
| 1     |2    | X and Y sizes for VE table          |
| 3     |256  | VE Map (16x16)                      |
| 259   |16   | VE Table RPM bins                   |
| 275   |16   | VE Table MAP/TPS bins               |
| 291   |64   | Page 2 settings (Non-Map page)      |
| 355   |2    | X and Y sizes for Ign table         |
| 357   |256  | Ignition Map (16x16)                |
| 613   |16   | Ign Table RPM bins                  |
| 629   |16   | Ign Table MAP/TPS bins              |
| 645   |64   | Page 4 settings (Non-Map page)      |
| 709   |2    | X and Y sizes for AFR table         |
| 711   |256  | AFR Target Map (16x16)              |
| 967   |16   | AFR Table RPM bins                  |
| 983   |16   | AFR Table MAP/TPS bins              |
| 999   |64   | Remaining Page 3 settings           |
| 1063  |64   | Page 4 settings                     |
| 1127  |2    | X and Y sizes for boost table       |
| 1129  |64   | Boost Map (8x8)                     |
| 1193  |8    | Boost Table RPM bins                |
| 1201  |8    | Boost Table TPS bins                |
| 1209  |2    | X and Y sizes                       |
| 1211  |64   | PAGE 8 MAP2                         |
| 1275  |8    | Xbins Map2                          |
| 1283  |8    | Ybins Map2                          |
| 1291  |2    | X and Y sizes1                      |
| 1293``|36   | PAGE 9 MAP1                         |
| 1329  |12   | X and Y Bins1                       |
| 1341  |2    | X and Y size2                       |
| 1343  |36   | PAGE 9 MAP2                         |
| 1379  |6    | X and Y Bins2                       |
| 1391  |2    | X and Y sizes3                      |
| 1393  |36   | PAGE 9 MAP3                         |
| 1429  |6    | X and Y Bins3                       |
| 1441  |2    | X and Y size4                       |
| 1443  |36   | PAGE 9 MAP4                         |
| 1479  |6    | X and Y Bins4                       |
| 1500  |128  | CANBUS config and data (Table 10_)  |
| 1628  |192  | Table 11 - General settings         |
|                                                   |
| 2559  |512  | Calibration data (O2)               |
| 3071  |512  | Calibration data (IAT)              |
| 3583  |512  | Calibration data (CLT)              |
-----------------------------------------------------
*/

#define EEPROM_DATA_VERSION   0

#define EEPROM_CONFIG1_XSIZE  1
#define EEPROM_CONFIG1_YSIZE  2
#define EEPROM_CONFIG1_MAP    3
#define EEPROM_CONFIG1_XBINS  259
#define EEPROM_CONFIG1_YBINS  275
#define EEPROM_CONFIG2_START  291
#define EEPROM_CONFIG2_END    355 // +64   131
#define EEPROM_CONFIG3_XSIZE  355
#define EEPROM_CONFIG3_YSIZE  356
#define EEPROM_CONFIG3_MAP    357
#define EEPROM_CONFIG3_XBINS  613
#define EEPROM_CONFIG3_YBINS  629
#define EEPROM_CONFIG4_START  645
#define EEPROM_CONFIG4_END    709
#define EEPROM_CONFIG5_XSIZE  709
#define EEPROM_CONFIG5_YSIZE  710
#define EEPROM_CONFIG5_MAP    711
#define EEPROM_CONFIG5_XBINS  967
#define EEPROM_CONFIG5_YBINS  983
#define EEPROM_CONFIG6_START  999
#define EEPROM_CONFIG6_END    1063
#define EEPROM_CONFIG7_START  1063
#define EEPROM_CONFIG7_END    1127
#define EEPROM_CONFIG8_XSIZE1 1127
#define EEPROM_CONFIG8_YSIZE1 1128
#define EEPROM_CONFIG8_MAP1   1129
#define EEPROM_CONFIG8_XBINS1 1193
#define EEPROM_CONFIG8_YBINS1 1201
#define EEPROM_CONFIG8_XSIZE2 1209
#define EEPROM_CONFIG8_YSIZE2 1210
#define EEPROM_CONFIG8_MAP2   1211
#define EEPROM_CONFIG8_XBINS2 1275
#define EEPROM_CONFIG8_YBINS2 1283
#define EEPROM_CONFIG8_END    1291

#define EEPROM_CONFIG9_XSIZE1 1291
#define EEPROM_CONFIG9_YSIZE1 1292
#define EEPROM_CONFIG9_MAP1   1293
#define EEPROM_CONFIG9_XBINS1 1329
#define EEPROM_CONFIG9_YBINS1 1335
#define EEPROM_CONFIG9_XSIZE2 1341
#define EEPROM_CONFIG9_YSIZE2 1342
#define EEPROM_CONFIG9_MAP2   1343
#define EEPROM_CONFIG9_XBINS2 1379
#define EEPROM_CONFIG9_YBINS2 1385
#define EEPROM_CONFIG9_XSIZE3 1391
#define EEPROM_CONFIG9_YSIZE3 1392
#define EEPROM_CONFIG9_MAP3   1393
#define EEPROM_CONFIG9_XBINS3 1429
#define EEPROM_CONFIG9_YBINS3 1435
#define EEPROM_CONFIG9_XSIZE4 1441
#define EEPROM_CONFIG9_YSIZE4 1442
#define EEPROM_CONFIG9_MAP4   1443
#define EEPROM_CONFIG9_XBINS4 1479
#define EEPROM_CONFIG9_YBINS4 1485
#define EEPROM_CONFIG10_START 1500
#define EEPROM_CONFIG10_END   1628
#define EEPROM_CONFIG11_START 1628
#define EEPROM_CONFIG11_END   1820

/**
Calibration data is stored at the end of the EEPROM
(This is in case any further calibration tables are needed as they are large blocks)
most calibration data is 16 Bit long
*/
#define EEPROM_LAST_BARO      2558

#define EEPROM_CALIBRATION_START 2559
#define EEPROM_CALIB_DATA_WIDTH 2

#define EEPROM_CALIBRATION_IAT_X EEPROM_CALIBRATION_START
#define EEPROM_CALIBRATION_IAT_Y (EEPROM_CALIBRATION_IAT_X + EEPROM_CALIB_DATA_WIDTH * CALIBRATION_TABLE_DIMENSION)

#define EEPROM_CALIBRATION_CLT_X (EEPROM_CALIBRATION_IAT_Y + EEPROM_CALIB_DATA_WIDTH * CALIBRATION_TABLE_DIMENSION)
#define EEPROM_CALIBRATION_CLT_Y (EEPROM_CALIBRATION_CLT_X + EEPROM_CALIB_DATA_WIDTH * CALIBRATION_TABLE_DIMENSION)

#define EEPROM_CALIBRATION_TPS_M (EEPROM_CALIBRATION_CLT_Y + EEPROM_CALIB_DATA_WIDTH * CALIBRATION_TABLE_DIMENSION)
#define EEPROM_CALIBRATION_TPS_N (EEPROM_CALIBRATION_TPS_M + EEPROM_CALIB_DATA_WIDTH)

#warning TODO (oli#9#): remove eeprom filler bytes
// TPS calibration now by inverse calculation
// added 20 empty bytes to preserve layout by now
// remove !!!
// was 2*6*2 = 24 bytes -> got 2*2 = 4 bytes


#define EEPROM_CALIBRATION_MAP_M (EEPROM_CALIBRATION_TPS_N + EEPROM_CALIB_DATA_WIDTH + 20)
#define EEPROM_CALIBRATION_MAP_N (EEPROM_CALIBRATION_MAP_M + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_MAP_L (EEPROM_CALIBRATION_MAP_N + EEPROM_CALIB_DATA_WIDTH)

#define EEPROM_CALIBRATION_BARO_M (EEPROM_CALIBRATION_MAP_L + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_BARO_N (EEPROM_CALIBRATION_BARO_M + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_BARO_L (EEPROM_CALIBRATION_BARO_N + EEPROM_CALIB_DATA_WIDTH)

#define EEPROM_CALIBRATION_O2_M (EEPROM_CALIBRATION_BARO_L + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_O2_N (EEPROM_CALIBRATION_O2_M + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_O2_L (EEPROM_CALIBRATION_O2_N + EEPROM_CALIB_DATA_WIDTH)

#define EEPROM_CALIBRATION_VBAT_M (EEPROM_CALIBRATION_O2_L + EEPROM_CALIB_DATA_WIDTH)
#define EEPROM_CALIBRATION_VBAT_L (EEPROM_CALIBRATION_VBAT_M + EEPROM_CALIB_DATA_WIDTH)

/**
decoder config data for configpage 12
*/

#define EEPROM_CONFIG12_START (EEPROM_CALIBRATION_VBAT_L + EEPROM_CALIB_DATA_WIDTH)

//U16 trigger_position_map[POSITION_COUNT]
#define EEPROM_CONFIG12_TRIGGER_POSITION_MAP_START EEPROM_CONFIG12_START

//S16 decoder_offset_deg
#define EEPROM_CONFIG12_DECODER_OFFSET (EEPROM_CONFIG12_START + 2* CRK_POSITION_COUNT)

//U16 decoder_delay_us
#define EEPROM_CONFIG12_DECODER_DELAY (EEPROM_CONFIG12_START + 18)

//U8 crank_noise_filter
#define EEPROM_CONFIG12_CRANK_NOISE_FILTER (EEPROM_CONFIG12_START + 20)

//U8 sync_ratio_min_pct
#define EEPROM_CONFIG12_SYNC_RATIO_MIN (EEPROM_CONFIG12_START + 21)

//U8 sync_ratio_max_pct
#define EEPROM_CONFIG12_SYNC_RATIO_MAX (EEPROM_CONFIG12_START + 22)

//U8 sync_stability_thrs
#define EEPROM_CONFIG12_SYNC_STABILITY_THRS (EEPROM_CONFIG12_START + 23)

//U8 decoder_timeout_s
#define EEPROM_CONFIG12_DECODER_TIMEOUT (EEPROM_CONFIG12_START + 24)

/**
ignition config data for configpage 13
*/

#define EEPROM_CONFIG13_START (EEPROM_CONFIG12_DECODER_TIMEOUT + 1)

//U16 dynamic_min_rpm
#define EEPROM_CONFIG13_DYNAMIC_MIN EEPROM_CONFIG13_START

//U16 dynamic_dwell_us
#define EEPROM_CONFIG13_DYNAMIC_DWELL (EEPROM_CONFIG13_START + 2)

//U8 safety_margin_us
#define EEPROM_CONFIG13_SAFETY_MARGIN (EEPROM_CONFIG13_START + 4)

//crank_position_t idle_ignition_position;
#define EEPROM_CONFIG13_IDLE_IGN_POS (EEPROM_CONFIG13_START + 5)

//crank_position_t idle_dwell_position;
#define EEPROM_CONFIG13_IDLE_DWELL_POS (EEPROM_CONFIG13_START + 6)

//U8 idle_advance_deg;
#define EEPROM_CONFIG13_IDLE_ADVANCE_DEG (EEPROM_CONFIG13_START + 7)

//U8 idle_dwell_deg;
#define EEPROM_CONFIG13_IDLE_DWELL_DEG (EEPROM_CONFIG13_START + 8)

/**
Ignition map (MAP based)
*/
#define EEPROM_IGNITIONTABLE_MAP_Z (EEPROM_CONFIG13_START + 9)
#define EEPROM_IGNITIONTABLE_MAP_X (EEPROM_IGNITIONTABLE_MAP_Z + 256)
#define EEPROM_IGNITIONTABLE_MAP_Y (EEPROM_IGNITIONTABLE_MAP_X + 16)

/**
This is the last used eeprom address -> memory dump will be read until here
*/
#define EEPROM_STORAGE_END (EEPROM_CONFIG13_START + 9)


#endif // EEPROM_LAYOUT_H_INCLUDED
