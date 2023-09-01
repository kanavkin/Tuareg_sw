#ifndef STORAGE_SYSLOG_LOCATIONS_H_INCLUDED
#define STORAGE_SYSLOG_LOCATIONS_H_INCLUDED


typedef enum {

    STORAGE_EEPROM_NOINIT,
    STORAGE_EEPROM_MIN_LENGTH,
    STORAGE_EEPROM_MAX_LENGTH,

    STORAGE_LOC_T2D_X_DATA_ERROR,
    STORAGE_LOC_T2D_X_CACHE_ERROR,
    STORAGE_LOC_T2D_LOGIC_ERROR,
    STORAGE_LOC_T2D_IPL_DIV_ERROR,

    STORAGE_LOC_T2D_INTERPOL_ERROR,

    STORAGE_LOC_MAP_INTERPOL_ARG_ERROR,
    STORAGE_LOC_MAP_INTERPOL_DIV_ERROR,

    STORAGE_LOC_MAP_X_DATA_ERROR,
    STORAGE_LOC_MAP_Y_DATA_ERROR,
    STORAGE_LOC_MAP_Z_DATA_ERROR,
    STORAGE_LOC_MAP_LOGIC_ERROR,

    STORAGE_LOC_CTRLSET_GET_DESIGNATOR_ERROR,

    STORAGE_LOC_COUNT

} Storage_sysloc_locations_t;




#endif // STORAGE_SYSLOG_LOCATIONS_H_INCLUDED
