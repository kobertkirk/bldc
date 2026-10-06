/*
 * Field oriented control of a hall-sensored hub motor.
 *
 * Runs in the ADC1 injected-end-of-sequence interrupt at 20 kHz:
 *   currents -> Clarke/Park -> PI (d,q) with BEMF feed-forward and
 *   back-calculation anti-windup -> inverse Park -> min/max SVPWM.
 * Torque (iq) command = throttle * i_phase_max, ramped, then clamped by
 *   phase current, battery current (from P = 1.5*vq*iq), regen current,
 *   under-voltage, temperature, speed and stall fold-backs.
 */
#include "motor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "hall.h"
#include "hw.h"
#include "mathx.h"

motor_t m;
float wh_used, ah_used, wh_regen;

#define DT            CTRL_DT
#define HZ            ((float)PWM_HZ)
#define VMAX_FRAC     0.92f       /* keep low-side on-time for bootstrap refresh */
#define BLOCKING      (FLT_OVERCURRENT | FLT_OVERVOLTAGE | FLT_HALL | FLT_THROTTLE | \
                       FLT_FET_TEMP | FLT_MOTOR_TEMP | FLT_CSA_OFFSET)
#define LATCHED       (FLT_OVERCURRENT | FLT_HALL | FLT_DETECT)

static float off_a = 2048, off_b = 2048, off_c = 2048;
static float acc_a, acc_b, acc_c;
static uint32_t calib_n;

static float kp, ki;                 /* run gains */
static float kp_det, ki_det;         /* conservative gains for detection */
static float int_d, int_q, vd_out, vq_out;
static float iq_cmd;
static float thr_f;
static uint32_t armed_ticks, thr_bad_ticks, ov_ticks, hall_bad_ticks, idle_ticks;
static uint32_t stall_ticks, fault_ticks;
static int armed;
static int8_t active_dir = 1;
static volatile uint32_t flt_latched, flt_isr, flt_main;
static volatile float derate_temp = 1.0f;
static volatile uint32_t lim_main;
static float ibus_f, power_f;

/* detection */
static int det_what, det_phase;
static uint32_t det_t;
static float det_theta, det_acc_v, det_acc_i, det_v1, det_i1, det_vinj, det_prev_id, det_acc_di;
static uint32_t det_n;
static int8_t det_sign;
static uint32_t det_edges0;
static float det_r, det_l;
static volatile int det_pending;      /* 1 = ISR finished a step, main must post-process */
static char det_msg[128];
static volatile int det_msg_ready;

static void enter(motor_state_t s)
{
    if (s != ST_RUN && s < ST_DET_R)
        pwm_disable();
    m.state = s;
}

void motor_update_gains(void)
{
    float wc = TWO_PI * cfg.bw_hz;
    kp = cfg.motor_l * wc;
    ki = cfg.motor_r * wc;
}

void motor_init(void)
{
    float s, c;
    sincos_f(0.0f, &s, &c);           /* build LUT outside the ISR */
    motor_update_gains();
    kp_det = 0.0003f * TWO_PI * 200.0f;
    ki_det = 0.15f * TWO_PI * 200.0f;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    m.dir = 1;
    m.t_fet = m.t_mot = 25.0f;
    calib_n = 0;
    acc_a = acc_b = acc_c = 0.0f;
    flt_latched = 0;
    m.state = ST_CALIB;
#ifndef SIM
    uint32_t t0 = millis();
    while (m.state == ST_CALIB && millis() - t0 < 500)
        ;
#endif
}

/* ------------------------------------------------------------------------ */
static void apply_voltage(float vd, float vq, float theta, float vbus)
{
    float s, c;
    sincos_f(theta, &s, &c);
    float va = vd * c - vq * s;              /* alpha */
    float vb = vd * s + vq * c;              /* beta  */
    float pa = va;
    float pb = -0.5f * va + 0.5f * SQRT3 * vb;
    float pc = -0.5f * va - 0.5f * SQRT3 * vb;
    float mx = fmaxf(pa, fmaxf(pb, pc));
    float mn = fminf(pa, fminf(pb, pc));
    float off = 0.5f * (mx + mn);
    float inv = 1.0f / vbus;
    pwm_set(0.5f + (pa - off) * inv, 0.5f + (pb - off) * inv, 0.5f + (pc - off) * inv);
}

/* PI current loop.  Returns 1 if the voltage vector was limited. */
static int current_loop(float idr, float iqr, float id, float iq, float theta_out,
                        float vbus, float p, float i, float ffd, float ffq, float vinj_d)
{
    float vmax = vbus * VMAX_FRAC * INV_SQRT3;
    float ed = idr - id, eq = iqr - iq;
    float nd = int_d + i * DT * ed, nq = int_q + i * DT * eq;
    float vd = p * ed + nd + ffd;
    float vq = p * eq + nq + ffq;
    int lim = 0;
    float mag2 = vd * vd + vq * vq;
    if (mag2 > vmax * vmax) {
        /* scale the vector (keeps its direction), and only let the integrators
           move in the direction that reduces saturation (no dump on transients) */
        float k = vmax / sqrtf(mag2);
        vd *= k;
        vq *= k;
        lim = 1;
        if (ed * vd < 0.0f)
            int_d = nd;
        if (eq * vq < 0.0f)
            int_q = nq;
    } else {
        int_d = nd;
        int_q = nq;
    }
    int_d = clampf(int_d, -vmax, vmax);
    int_q = clampf(int_q, -vmax, vmax);
    vd_out = vd;
    vq_out = vq;
    float vdi = clampf(vd + vinj_d, -vmax, vmax);
    apply_voltage(vdi, vq, theta_out, vbus);
    m.vd = vd;
    m.vq = vq;
    m.mod = sqrtf(vd * vd + vq * vq) / (vbus * INV_SQRT3);
    return lim;
}

static void start_run(float omega)
{
    int_d = 0.0f;
    int_q = 0.0f;
    iq_cmd = 0.0f;
    vd_out = 0.0f;
    vq_out = omega * cfg.motor_flux;
    pwm_set(0.5f, 0.5f, 0.5f);
    pwm_enable();
    idle_ticks = 0;
    stall_ticks = 0;
    m.state = ST_RUN;
}

static void run_foc(float id, float iq, float theta, float omega, float vbus, float pct)
{
    uint32_t lim = lim_main;

    /* --- direction ------------------------------------------------------- */
    int8_t dir_req = dir_reverse() ? -1 : 1;
    if (cfg.dir_invert)
        dir_req = (int8_t)-dir_req;
    int stopped = fabsf(omega) < TWO_PI * 2.0f;       /* < 2 electrical rev/s */
    if (dir_req != active_dir && stopped && fabsf(iq_cmd) < 1.0f)
        active_dir = dir_req;
    float target = (dir_req == active_dir) ? pct * cfg.i_phase_max * active_dir : 0.0f;

    /* --- brake ----------------------------------------------------------- */
    m.brake = (uint8_t)brake_on();
    if (m.brake)
        target = (cfg.i_brake > 0.0f && !stopped) ? (omega > 0 ? -cfg.i_brake : cfg.i_brake) : 0.0f;

    /* --- fold-backs (motoring only) -------------------------------------- */
    float scale = derate_temp;
    if (vbus < cfg.v_uv_start) {
        float s = (vbus - cfg.v_uv_cut) / (cfg.v_uv_start - cfg.v_uv_cut);
        scale = fminf(scale, clampf(s, 0.0f, 1.0f));
        lim |= LIM_UV;
    }
    if (cfg.erpm_max > 0.0f) {
        float erpm = fabsf(omega) * (60.0f / TWO_PI);
        float s = 1.0f - (erpm - cfg.erpm_max) / (0.1f * cfg.erpm_max);
        if (s < 1.0f) {
            scale = fminf(scale, clampf(s, 0.0f, 1.0f));
            lim |= LIM_SPEED;
        }
    }
    if (stopped && fabsf(iq) > 0.5f * cfg.i_phase_max) {
        if (stall_ticks < 10u * PWM_HZ)
            stall_ticks++;
    } else if (stall_ticks) {
        stall_ticks--;
    }
    if (stall_ticks > 3u * PWM_HZ) {
        scale = fminf(scale, 0.5f);
        lim |= LIM_STALL;
    }
    if (target * omega >= 0.0f)                       /* motoring */
        target *= scale;

    /* --- ramp ------------------------------------------------------------ */
    float rate = (fabsf(target) > fabsf(iq_cmd) && target * iq_cmd >= 0.0f) ? cfg.ramp_up
                                                                             : cfg.ramp_down;
    float step = rate * DT;
    iq_cmd += clampf(target - iq_cmd, -step, step);

    /* --- battery / phase current limits ---------------------------------- */
    float vqa = fmaxf(fabsf(vq_out), 1.0f);
    float lim_mot = cfg.i_batt_max * vbus / (1.5f * vqa);
    float lim_reg = cfg.i_batt_regen * vbus / (1.5f * vqa);
    float hi = vq_out >= 0.0f ? lim_mot : lim_reg;
    float lo = vq_out >= 0.0f ? -lim_reg : -lim_mot;
    hi = fminf(hi, cfg.i_phase_max);
    lo = fmaxf(lo, -cfg.i_phase_max);
    float iq_ref = clampf(iq_cmd, lo, hi);
    if (iq_ref != iq_cmd) {
        lim |= (iq_ref == cfg.i_phase_max || iq_ref == -cfg.i_phase_max) ? LIM_PHASE : LIM_BATT;
        iq_cmd = iq_ref;          /* don't wind up the ramp beyond the limit */
    }
    m.iq_ref = iq_ref;

    /* --- current loop ---------------------------------------------------- */
    float ffd = -omega * cfg.motor_l * iq;
    float ffq = omega * (cfg.motor_l * id + cfg.motor_flux);
    float th_out = theta + omega * 1.5f * DT;         /* compensate PWM delay */
    if (current_loop(0.0f, iq_ref, id, iq, th_out, vbus, kp, ki, ffd, ffq, 0.0f))
        lim |= LIM_VOLTAGE;

    /* --- learn flux linkage from steady running -------------------------- */
    if (fabsf(omega) > 150.0f && !(lim & LIM_VOLTAGE)) {
        float fl = (vq_out - cfg.motor_r * iq - omega * cfg.motor_l * id) / omega;
        if (fl > 0.0f && fl < 0.5f)
            cfg.motor_flux += 0.0002f * (fl - cfg.motor_flux);
    }
    m.limits = lim;

    /* --- back to idle when stopped with no demand ------------------------ */
    if (pct == 0.0f && !(m.brake && cfg.i_brake > 0.0f) && stopped && fabsf(iq) < 2.0f) {
        if (++idle_ticks > PWM_HZ / 3u)
            enter(ST_IDLE);
    } else {
        idle_ticks = 0;
    }
}

/* ------------------------------------------------------------------------ */
static void det_start_phase(int ph)
{
    det_phase = ph;
    det_t = 0;
    det_n = 0;
    det_acc_v = det_acc_i = det_acc_di = 0.0f;
}

static void det_step(float id, float iq, float vbus)
{
    float I = cfg.detect_amps;
    det_t++;
    switch (m.state) {
    case ST_DET_R:
        /* two current levels; R from the slope cancels dead-time error */
        {
            float lvl = det_phase == 0 ? 0.33f * I : I;
            current_loop(lvl, 0.0f, id, iq, 0.0f, vbus, kp_det, ki_det, 0, 0, 0);
            if (det_t > PWM_HZ * 3u / 10u) {
                det_acc_v += vd_out;
                det_acc_i += id;
                det_n++;
            }
            if (det_t >= PWM_HZ * 6u / 10u) {
                float v = det_acc_v / det_n, i = det_acc_i / det_n;
                if (det_phase == 0) {
                    det_v1 = v;
                    det_i1 = i;
                    det_start_phase(1);
                } else {
                    det_r = (v - det_v1) / (i - det_i1);
                    det_pending = 1;
                    pwm_set(0.5f, 0.5f, 0.5f);
                    m.state = ST_IDLE;        /* main continues the sequence */
                    pwm_disable();
                }
            }
        }
        break;

    case ST_DET_L:
        /* d-axis bias current + alternating +-vinj each period: |di| = vinj*Ts/L */
        {
            det_sign = (int8_t)-det_sign;
            current_loop(0.5f * I, 0.0f, id, iq, 0.0f, vbus, kp_det, ki_det, 0, 0,
                         det_sign * det_vinj);
            if (det_t > 400) {
                det_acc_di += fabsf(id - det_prev_id);
                det_n++;
            }
            det_prev_id = id;
            if (det_n >= 4000) {
                float di = det_acc_di / det_n;
                if (di < 0.4f && det_vinj < 0.15f * vbus) {
                    det_vinj *= 2.0f;
                    det_start_phase(0);
                } else {
                    det_l = det_vinj * DT / di;
                    det_pending = 2;
                    m.state = ST_IDLE;
                    pwm_disable();
                }
            }
        }
        break;

    case ST_DET_HALL:
        {
            const uint32_t align = PWM_HZ;              /* 1 s   */
            const float f_e = 1.0f;                     /* electrical Hz */
            const uint32_t sweep = 5u * PWM_HZ;         /* 5 revs each way */
            float amp = I;
            if (det_phase == 0) {
                amp = I * fminf(1.0f, (float)det_t / (0.3f * PWM_HZ));
                if (det_t >= align) {
                    det_start_phase(1);
                    det_edges0 = hall_edge_count();
                }
            } else if (det_phase == 1 || det_phase == 2) {
                float dirn = det_phase == 1 ? 1.0f : -1.0f;
                det_theta = wrap_2pi(det_theta + dirn * TWO_PI * f_e * DT);
                if (det_t > PWM_HZ)                       /* skip first rev */
                    hall_learn_sample(det_theta, hall_read());
                if (det_phase == 1 && det_t == 2u * PWM_HZ &&
                    hall_edge_count() == det_edges0) {
                    det_pending = 13;                     /* rotor not moving */
                    m.state = ST_IDLE;
                    pwm_disable();
                    break;
                }
                if (det_t >= sweep) {
                    if (det_phase == 1)
                        det_start_phase(2);
                    else
                        det_start_phase(3);
                }
            } else {
                amp = I * fmaxf(0.0f, 1.0f - (float)det_t / (0.3f * PWM_HZ));
                if (det_t >= PWM_HZ * 3u / 10u) {
                    det_pending = 3;
                    m.state = ST_IDLE;
                    pwm_disable();
                    break;
                }
            }
            /* current vector along the d-axis of the forced frame at det_theta */
            current_loop(amp, 0.0f, id, iq, det_theta, vbus, kp_det, ki_det, 0, 0, 0);
        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */
void motor_isr(void)
{
    uint32_t c0 = DWT->CYCCNT;
    adc_raw_t r;
    adc_read(&r);
    m.isr_count++;

    float vb = r.vbus * VOLTS_PER_COUNT;
    m.vbus += 0.05f * (vb - m.vbus);
    float vbus = fmaxf(m.vbus, 5.0f);
    thr_f += 0.01f * (r.thr * (VREF / ADC_FS) * THR_DIV - thr_f);
    m.thr_v = thr_f;
    m.raw_tfet += 0.002f * ((float)r.tfet - m.raw_tfet);
    m.raw_tmot += 0.002f * ((float)r.tmot - m.raw_tmot);

    if (m.state == ST_CALIB) {
        if (calib_n >= 200) {                 /* let filters settle first */
            acc_a += r.ia;
            acc_b += r.ib;
            acc_c += r.ic;
        }
        if (++calib_n >= 200 + 2048) {
            off_a = acc_a / 2048.0f;
            off_b = acc_b / 2048.0f;
            off_c = acc_c / 2048.0f;
            int bad = fabsf(off_a - 2048.0f) > 200.0f || fabsf(off_b - 2048.0f) > 200.0f ||
                      fabsf(off_c - 2048.0f) > 200.0f;
            flt_isr = bad ? FLT_CSA_OFFSET : 0;
            m.raw_tfet = r.tfet;
            m.raw_tmot = r.tmot;
            m.state = bad ? ST_FAULT : ST_IDLE;
        }
        return;
    }

    float ia = ((float)r.ia - off_a) * AMPS_PER_COUNT;
    float ib = ((float)r.ib - off_b) * AMPS_PER_COUNT;
    float ic = ((float)r.ic - off_c) * AMPS_PER_COUNT;
    m.ia = ia;
    m.ib = ib;
    m.ic = ic;

    /* ---- protections that must act within one PWM period ---------------- */
    if (fabsf(ia) > HARD_TRIP_AMPS || fabsf(ib) > HARD_TRIP_AMPS || fabsf(ic) > HARD_TRIP_AMPS) {
        pwm_disable();
        flt_latched |= FLT_OVERCURRENT;
    }
    uint32_t fi = flt_isr & FLT_CSA_OFFSET;
    if (vb > cfg.v_ov) {
        if (++ov_ticks > 20)
            fi |= FLT_OVERVOLTAGE;
    } else if (vb > cfg.v_ov - 1.0f && (flt_isr & FLT_OVERVOLTAGE)) {
        fi |= FLT_OVERVOLTAGE;               /* 1 V hysteresis */
    } else {
        ov_ticks = 0;
    }
    if (m.vbus < cfg.v_uv_cut)
        fi |= FLT_UNDERVOLTAGE;

    /* ---- throttle --------------------------------------------------------- */
    float pct = 0.0f;
    if (thr_f < cfg.thr_fault_lo || thr_f > cfg.thr_fault_hi) {
        if (++thr_bad_ticks > PWM_HZ / 20u)
            fi |= FLT_THROTTLE;
    } else {
        thr_bad_ticks = 0;
        pct = clampf((thr_f - cfg.thr_min_v) / (cfg.thr_max_v - cfg.thr_min_v), 0.0f, 1.0f);
        if (pct < 0.02f)
            pct = 0.0f;
    }
    if (fi & FLT_THROTTLE)
        pct = 0.0f;
    m.thr_pct = pct;
    if (pct == 0.0f) {
        if (++armed_ticks > PWM_HZ / 5u)
            armed = 1;
    } else {
        armed_ticks = 0;
    }
    if (!armed && pct > 0.0f)
        fi |= FLT_THR_NOT_ZERO;
    if (!cfg.hall_valid)
        fi |= FLT_NOT_DETECTED;

    /* ---- position --------------------------------------------------------- */
    uint32_t hs = hall_read();
    m.hall = hs;
    if (hs == 0 || hs == 7) {
        if (++hall_bad_ticks > PWM_HZ / 100u && m.state == ST_RUN)
            flt_latched |= FLT_HALL;
    } else {
        hall_bad_ticks = 0;
    }
    float theta, omega;
    if (m.state >= ST_DET_R && m.state <= ST_DET_HALL) {
        theta = det_theta;
        omega = 0.0f;
    } else {
        hall_get(micros(), &theta, &omega);
    }
    m.omega_e = omega;
    m.theta = theta;

    float s, c;
    sincos_f(theta, &s, &c);
    float ial = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
    float ibe = (ib - ic) * INV_SQRT3;
    float id = ial * c + ibe * s;
    float iq = -ial * s + ibe * c;
    m.id = id;
    m.iq = iq;

    flt_isr = fi;
    uint32_t faults = flt_latched | fi | flt_main;
    m.faults = faults;

    /* ---- state machine ---------------------------------------------------- */
    switch (m.state) {
    case ST_IDLE:
        m.limits = lim_main;
        m.iq_ref = 0.0f;
        if (faults & BLOCKING) {
            enter(ST_FAULT);
            break;
        }
        if (armed && cfg.hall_valid && !(faults & FLT_UNDERVOLTAGE) &&
            (pct > 0.0f || (brake_on() && cfg.i_brake > 0.0f && fabsf(omega) > TWO_PI * 2.0f)))
            start_run(omega);
        break;

    case ST_RUN:
        if (faults & BLOCKING) {
            enter(ST_FAULT);
            break;
        }
        run_foc(id, iq, theta, omega, vbus, pct);
        break;

    case ST_DET_R:
    case ST_DET_L:
    case ST_DET_HALL:
        if (faults & (FLT_OVERCURRENT | FLT_OVERVOLTAGE | FLT_FET_TEMP | FLT_CSA_OFFSET)) {
            det_pending = 99;
            enter(ST_FAULT);
            break;
        }
        det_step(id, iq, vbus);
        break;

    case ST_FAULT:
        pwm_disable();
        m.iq_ref = 0.0f;
        if (pct == 0.0f)
            fault_ticks++;
        else
            fault_ticks = 0;
        /* self-clearing latched faults once the throttle is released */
        if (fault_ticks > PWM_HZ) {
            uint32_t clr = FLT_OVERCURRENT;
            if (hall_bad_ticks == 0)
                clr |= FLT_HALL;
            flt_latched &= ~clr;
        }
        if (!(faults & BLOCKING) && fault_ticks > PWM_HZ / 2u) {
            fault_ticks = 0;
            m.state = ST_IDLE;
        }
        break;

    default:
        pwm_disable();
        break;
    }

    /* ---- battery-side power estimate -------------------------------------- */
    if (pwm_is_enabled()) {
        float p = 1.5f * (vd_out * id + vq_out * iq);
        power_f += 0.01f * (p - power_f);
    } else {
        power_f *= 0.99f;
        m.vd = m.vq = m.mod = 0.0f;
    }
    ibus_f = power_f / vbus;
    m.power = power_f;
    m.ibus = ibus_f;
    m.dir = active_dir;
    m.isr_cycles = DWT->CYCCNT - c0;
}

/* ------------------------------------------------------------------------ */
static float ntc_temp(float raw, float beta)
{
    float v = raw * (VREF / ADC_FS);
    if (v < 0.02f || v > VREF - 0.02f)
        return NAN;                          /* open or shorted sensor */
    float r = NTC_PULLUP_OHM * v / (VREF - v);
    return 1.0f / (1.0f / 298.15f + logf(r / NTC_R25) / beta) - 273.15f;
}

static float fold(float t, float start, float max)
{
    if (t <= start)
        return 1.0f;
    return clampf(1.0f - (t - start) / (max - start), 0.0f, 1.0f);
}

static void detect_finish(void)
{
    det_msg_ready = 1;
}

void motor_slow(uint32_t now_ms)
{
    (void)now_ms;
    /* temperatures and fold-back */
    uint32_t fm = 0, lim = 0;
    float tf = ntc_temp(m.raw_tfet, NTC_FET_BETA);
    float sc = 1.0f;
    if (isnan(tf)) {
        fm |= FLT_FET_TEMP;
        m.t_fet = -99.0f;
    } else {
        m.t_fet = tf;
        float f = fold(tf, cfg.t_fet_start, cfg.t_fet_max);
        if (f < 1.0f)
            lim |= LIM_FET_TEMP;
        sc = fminf(sc, f);
        if (tf > cfg.t_fet_max + 10.0f)
            fm |= FLT_FET_TEMP;
    }
    if (cfg.mot_ntc_en) {
        float tm = ntc_temp(m.raw_tmot, cfg.mot_ntc_beta);
        if (isnan(tm)) {
            fm |= FLT_MOTOR_TEMP;
            m.t_mot = -99.0f;
        } else {
            m.t_mot = tm;
            float f = fold(tm, cfg.t_mot_start, cfg.t_mot_max);
            if (f < 1.0f)
                lim |= LIM_MOT_TEMP;
            sc = fminf(sc, f);
            if (tm > cfg.t_mot_max + 10.0f)
                fm |= FLT_MOTOR_TEMP;
        }
    } else {
        m.t_mot = NAN;
    }
    derate_temp = sc;
    lim_main = lim;
    flt_main = fm;

    /* energy counters */
    float p = m.power;
    float e = p * (0.001f / 3600.0f);
    if (e > 0)
        wh_used += e;
    else
        wh_regen -= e;
    ah_used += m.ibus * (0.001f / 3600.0f);

    /* detection sequencing (heavy maths / text kept out of the ISR) */
    int pend = det_pending;
    if (!pend)
        return;
    det_pending = 0;
    char buf[96];
    switch (pend) {
    case 1:
        if (!(det_r > 0.003f && det_r < 3.0f)) {
            snprintf(det_msg, sizeof(det_msg), "detect FAILED: R=%d mOhm out of range (motor connected?)",
                     (int)(det_r * 1000.0f));
            flt_latched |= FLT_DETECT;
            detect_finish();
            return;
        }
        cfg.motor_r = det_r;
        if (det_what == 1) {
            motor_update_gains();
            snprintf(det_msg, sizeof(det_msg), "R = %d.%03d ohm", (int)det_r,
                     (int)(det_r * 1000.0f) % 1000);
            detect_finish();
            return;
        }
        det_vinj = 1.0f;
        det_sign = 1;
        det_prev_id = 0.0f;
        int_d = int_q = 0.0f;
        det_theta = 0.0f;
        det_start_phase(0);
        pwm_set(0.5f, 0.5f, 0.5f);
        pwm_enable();
        m.state = ST_DET_L;
        break;
    case 2:
        if (!(det_l > 5e-6f && det_l < 0.02f)) {
            snprintf(det_msg, sizeof(det_msg), "detect FAILED: L=%d uH out of range",
                     (int)(det_l * 1e6f));
            flt_latched |= FLT_DETECT;
            detect_finish();
            return;
        }
        cfg.motor_l = det_l;
        motor_update_gains();
        if (det_what == 2 || det_what == 1) {
            snprintf(det_msg, sizeof(det_msg), "R = %d mOhm, L = %d uH", (int)(cfg.motor_r * 1000.0f),
                     (int)(det_l * 1e6f));
            detect_finish();
            return;
        }
        hall_learn_reset();
        int_d = int_q = 0.0f;
        det_theta = 0.0f;
        det_start_phase(0);
        pwm_set(0.5f, 0.5f, 0.5f);
        pwm_enable();
        m.state = ST_DET_HALL;
        break;
    case 3:
        if (hall_learn_finish(buf, sizeof(buf)) == 0) {
            snprintf(det_msg, sizeof(det_msg),
                     "detect OK: R = %d mOhm, L = %d uH, halls learned. Type 'save' to store.",
                     (int)(cfg.motor_r * 1000.0f), (int)(cfg.motor_l * 1e6f));
        } else {
            snprintf(det_msg, sizeof(det_msg), "detect FAILED: %s", buf);
            flt_latched |= FLT_DETECT;
        }
        detect_finish();
        break;
    case 13:
        snprintf(det_msg, sizeof(det_msg),
                 "detect FAILED: no hall edges - lift the wheel so it spins freely, check hall wiring");
        flt_latched |= FLT_DETECT;
        detect_finish();
        break;
    default:
        snprintf(det_msg, sizeof(det_msg), "detect ABORTED by fault");
        flt_latched |= FLT_DETECT;
        detect_finish();
        break;
    }
}

int motor_detect(int what)
{
    if (m.state != ST_IDLE && m.state != ST_FAULT)
        return -1;
    if (m.faults & (FLT_OVERCURRENT | FLT_OVERVOLTAGE | FLT_CSA_OFFSET | FLT_UNDERVOLTAGE |
                    FLT_FET_TEMP))
        return -2;
    if (m.thr_pct > 0.0f)
        return -3;
    __disable_irq();
    flt_latched &= ~(FLT_DETECT | FLT_HALL);
    det_msg_ready = 0;
    det_msg[0] = 0;
    det_what = what;
    det_theta = 0.0f;
    int_d = int_q = 0.0f;
    det_start_phase(0);
    pwm_set(0.5f, 0.5f, 0.5f);
    if (what == 3) {
        hall_learn_reset();
        m.state = ST_DET_HALL;
    } else if (what == 2) {
        det_vinj = 1.0f;
        det_sign = 1;
        m.state = ST_DET_L;
    } else {
        m.state = ST_DET_R;
    }
    pwm_enable();
    __enable_irq();
    return 0;
}

const char *motor_detect_msg(void)
{
    return det_msg_ready ? det_msg : NULL;
}

void motor_clear_faults(void)
{
    __disable_irq();
    flt_latched = 0;
    __enable_irq();
}

void motor_shutdown(void)
{
    __disable_irq();
    pwm_disable();
    m.state = ST_OFF;
    __enable_irq();
}

const char *motor_state_name(motor_state_t s)
{
    static const char *n[] = {"CALIB", "IDLE", "RUN", "FAULT", "DET_R", "DET_L", "DET_HALL", "OFF"};
    return (unsigned)s < sizeof(n) / sizeof(n[0]) ? n[s] : "?";
}

void motor_fault_names(uint32_t f, char *buf, unsigned len)
{
    static const char *n[] = {"OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "HALL", "THROTTLE",
                              "FET_TEMP", "MOTOR_TEMP", "NOT_DETECTED", "DETECT", "CSA_OFFSET",
                              "THROTTLE_AT_POWERUP"};
    buf[0] = 0;
    unsigned pos = 0;
    for (unsigned i = 0; i < sizeof(n) / sizeof(n[0]); i++) {
        if (f & (1u << i)) {
            int k = snprintf(buf + pos, len - pos, "%s%s", pos ? "|" : "", n[i]);
            if (k < 0 || (unsigned)k >= len - pos)
                break;
            pos += (unsigned)k;
        }
    }
    if (!pos)
        snprintf(buf, len, "NONE");
}
