#ifndef VITALSCHED_SYSLOG_H_INCLUDED
#define VITALSCHED_SYSLOG_H_INCLUDED


typedef enum {



    VITALSCHED_LOC_SETCH_INITCHECK,
    VITALSCHED_LOC_SETCH_PARMCHECK_CH,
    VITALSCHED_LOC_SETCH_PARMCHECK_INT1,
    VITALSCHED_LOC_SETCH_PARMCHECK_INT2,

    VITALSCHED_LOC_RESETCH_INITCHECK,
    VITALSCHED_LOC_RESETCH_PARMCHECK_CH,

    VITALSCHED_LOC_COUNT



} Vitalsched_syslog_locations_t;






#endif
