#ifndef FUELING_SYSLOC_LOCATIONS_H_INCLUDED
#define FUELING_SYSLOC_LOCATIONS_H_INCLUDED

typedef enum {

    FUELING_LOC_CONFIG_ERROR,
    FUELING_LOC_CONFIGVERSION_ERROR,
    FUELING_LOC_READY,

    FUELING_LOC_UPDCTRL_VITAL_PRECOND,
    FUELING_LOC_UPDCTRL_LOAD_SENSORS,
    FUELING_LOC_UPDCTRL_SENSORS,
    FUELING_LOC_UPDCTRL_VE_INVALID,
    FUELING_LOC_UPDCTRL_AFR_INVALID,
    FUELING_LOC_UPDCTRL_CHTMP_INVALID,

    FUELING_LOC_SEQUENTIAL_CALC_FAIL,
    FUELING_LOC_INJ_DEADTIME_INVALID,

    FUELING_LOC_REL_COMP_CLIP,
    FUELING_LOC_ABS_COMP_CLIP,


    FUELING_LOC_ACCELCOMP_RPMSCALE,

    FUELING_LOC_UPDINJBEG_ENDTARGET_INVALID,

    FUELING_LOC_ACCELCOMP_CLIP_COLD_ACCEL_PCT,

    FUELING_LOC_BAROCORR_INVALID,

    FUELING_LOC_COUNT

} fueling_sysloc_locations_t;




#endif // FUELING_SYSLOC_LOCATIONS_H_INCLUDED
