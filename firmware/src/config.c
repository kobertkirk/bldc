#include "config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

config_t cfg;

void config_defaults(config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->magic = CONFIG_MAGIC;
    c->version = CONFIG_VERSION;

    c->pole_pairs = 23.0f;        /* typical geared/direct-drive hub motor   */
    c->motor_r = 0.15f;           /* overwritten by 'detect'                 */
    c->motor_l = 0.0003f;
    c->motor_flux = 0.0f;         /* learned while riding                    */
    for (int i = 0; i < 8; i++)
        c->hall_angle[i] = NAN;
    c->hall_valid = 0;
    c->dir_invert = 0;

    c->i_phase_max = 60.0f;
    c->i_batt_max = 35.0f;
    c->i_batt_regen = 5.0f;
    c->i_brake = 0.0f;
    c->v_uv_start = 42.0f;        /* 13S: 3.23 V/cell                        */
    c->v_uv_cut = 39.0f;          /* 13S: 3.0 V/cell                         */
    c->v_ov = 60.0f;
    c->t_fet_start = 80.0f;
    c->t_fet_max = 100.0f;
    c->t_mot_start = 100.0f;
    c->t_mot_max = 130.0f;
    c->mot_ntc_beta = 3950.0f;
    c->mot_ntc_en = 0;
    c->erpm_max = 0.0f;

    c->thr_min_v = 1.0f;
    c->thr_max_v = 4.0f;
    c->thr_fault_lo = 0.4f;
    c->thr_fault_hi = 4.7f;
    c->ramp_up = 100.0f;
    c->ramp_down = 400.0f;

    c->bw_hz = 1000.0f;
    c->detect_amps = 15.0f;

    c->stream_ms = 100;
    c->auto_off_min = 0;
}

#define F(n, h) {#n, &cfg.n, 1, h}
#define U(n, h) {#n, &cfg.n, 0, h}
const param_t params[] = {
    F(pole_pairs, "motor pole pairs (rpm display)"),
    F(motor_r, "phase resistance [ohm] (detect)"),
    F(motor_l, "phase inductance [H] (detect)"),
    F(motor_flux, "flux linkage [Wb] (learned)"),
    U(hall_valid, "hall table valid (detect)"),
    U(dir_invert, "1 = swap forward/reverse"),
    F(i_phase_max, "max phase current [A]"),
    F(i_batt_max, "max battery current [A]"),
    F(i_batt_regen, "max regen battery current [A]"),
    F(i_brake, "regen brake phase current [A], 0=off"),
    F(v_uv_start, "undervoltage fold-back start [V]"),
    F(v_uv_cut, "undervoltage cut-off [V]"),
    F(v_ov, "overvoltage trip [V]"),
    F(t_fet_start, "FET temp fold-back start [C]"),
    F(t_fet_max, "FET temp cut-off [C]"),
    F(t_mot_start, "motor temp fold-back start [C]"),
    F(t_mot_max, "motor temp cut-off [C]"),
    F(mot_ntc_beta, "motor NTC beta"),
    U(mot_ntc_en, "1 = motor NTC fitted"),
    F(erpm_max, "electrical rpm limit, 0=none"),
    F(thr_min_v, "throttle volts at 0%"),
    F(thr_max_v, "throttle volts at 100%"),
    F(thr_fault_lo, "throttle fault below [V]"),
    F(thr_fault_hi, "throttle fault above [V]"),
    F(ramp_up, "torque ramp up [A/s]"),
    F(ramp_down, "torque ramp down [A/s]"),
    F(bw_hz, "current loop bandwidth [Hz]"),
    F(detect_amps, "detect current [A]"),
    U(stream_ms, "telemetry period [ms], 0=off"),
    U(auto_off_min, "auto power-off when idle [min], 0=never"),
};
const unsigned n_params = sizeof(params) / sizeof(params[0]);
