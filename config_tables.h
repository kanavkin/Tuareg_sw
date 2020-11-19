#ifndef CONFIG_TABLES_H_INCLUDED
#define CONFIG_TABLES_H_INCLUDED

extern volatile table3D_t ignitionTable_TPS, ignitionTable_MAP, fuelTable, afrTable;
extern volatile table3D_t boostTable, vvtTable;
extern volatile table3D_t trim1Table, trim2Table, trim3Table, trim4Table;

extern volatile table2D taeTable;
extern volatile table2D WUETable;
extern volatile table2D crankingEnrichTable;
extern volatile table2D dwellVCorrectionTable;
extern volatile table2D injectorVCorrectionTable;
extern volatile table2D IATDensityCorrectionTable;
extern volatile table2D IATRetardTable;
extern volatile table2D rotarySplitTable;
extern volatile table2D IAT_calib_table;
extern volatile table2D CLT_calib_table;


#endif // CONFIG_TABLES_H_INCLUDED
