#ifndef TUAREG_SYSLOC_LOCATIONS_H_INCLUDED
#define TUAREG_SYSLOC_LOCATIONS_H_INCLUDED

typedef enum {

    TUAREG_LOC_FATAL_ERROR,
    TUAREG_LOC_ENTER_LIMP_MODE,

    TUAREG_LOC_SET_RUNMODE_LIMP_EXIT,
    TUAREG_LOC_SET_RUNMODE_FATAL_EXIT,

    TUAREG_LOC_LOAD_CONFIG_FAIL,
    TUAREG_LOC_LOAD_CONFIG_SUCCESS,
    TUAREG_LOC_LOAD_CONFIG_VERSION_FAIL,
    TUAREG_LOC_ESSENTIALS_CONFIG_LOADED,

    TUAREG_LOC_SET_RUNMODE_DEFAULT_BRANCH,

    TUAREG_LOC_UPDATE_RUNMODE_HALTSRC_PRESENT,
    TUAREG_LOC_UPDATE_RUNMODE_LIMP_RUNSWITCH_RELEASED,
    TUAREG_LOC_UPDATE_RUNMODE_CRANKING_END,
    TUAREG_LOC_UPDATE_RUNMODE_HALTSRC_CLEARED,

    TUAREG_LOC_SET_RUNMODE_SERVICE,

    TUAREG_LOC_INJ_WATCHDOG,

    TUAREG_LOC_LIMP_TPSMAP_ERROR,

    TUAREG_LOC_COUNT



} Tuareg_sysloc_locations_t;




#endif // TUAREG_SYSLOC_LOCATIONS_H_INCLUDED
