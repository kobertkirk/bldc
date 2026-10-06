/* Persistent configuration (stored in the last 2 KB flash page). */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define CONFIG_MAGIC   0x424C4443u   /* "BLDC" */
#define CONFIG_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;

    /* motor */
    float pole_pairs;          /* for rpm display only                       */
    float motor_r;             /* phase resistance, ohm (line-neutral)        */
    float motor_l;             /* phase inductance, H                         */
    float motor_flux;          /* flux linkage, V*s (rad)                     */
    float hall_angle[8];       /* electrical angle (rad) of each hall state   */
    uint32_t hall_valid;       /* 1 once 'detect' has succeeded               */
    uint32_t dir_invert;       /* swap what FWD means                         */

    /* limits */
    float i_phase_max;         /* A, peak phase current at full throttle      */
    float i_batt_max;          /* A, battery (DC) current limit, motoring     */
    float i_batt_regen;        /* A, battery current limit while regenerating */
    float i_brake;             /* A, phase current used for regen braking (0 = off) */
    float v_uv_start;          /* V, start of undervoltage fold-back          */
    float v_uv_cut;            /* V, zero torque below this                   */
    float v_ov;                /* V, over-voltage trip                        */
    float t_fet_start;         /* C, FET temperature fold-back start          */
    float t_fet_max;           /* C, zero torque at/above                     */
    float t_mot_start;         /* C, motor temperature fold-back start        */
    float t_mot_max;
    float mot_ntc_beta;        /* motor NTC beta (10k @25C assumed)            */
    uint32_t mot_ntc_en;       /* 0 = no motor sensor fitted                  */
    float erpm_max;            /* electrical rpm limit (0 = none)             */

    /* throttle */
    float thr_min_v;           /* V at throttle connector = 0 %               */
    float thr_max_v;           /* V at throttle connector = 100 %             */
    float thr_fault_lo;        /* V, below = wire broken                      */
    float thr_fault_hi;        /* V, above = short to 5 V                     */
    float ramp_up;             /* A/s torque ramp when increasing             */
    float ramp_down;           /* A/s torque ramp when decreasing             */

    /* control */
    float bw_hz;               /* current loop bandwidth                      */
    float detect_amps;         /* current used by detect                      */

    /* misc */
    uint32_t stream_ms;        /* telemetry period, 0 = off                   */
    uint32_t auto_off_min;     /* power off after N idle minutes, 0 = never   */

    uint32_t crc;
} config_t;

extern config_t cfg;

void config_defaults(config_t *c);
int config_load(void);           /* 1 = loaded from flash, 0 = defaults      */
int config_save(void);           /* 0 = ok; only call with PWM disabled       */

/* name/value table for the CLI */
typedef struct {
    const char *name;
    void *ptr;
    uint8_t is_float;
    const char *help;
} param_t;

extern const param_t params[];
extern const unsigned n_params;

#endif
