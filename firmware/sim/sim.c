/*
 * Closed-loop host simulation of the controller firmware.
 *
 * The real motor.c / hall.c / mathx.c / config.c are compiled for the PC and
 * driven by a model of the hardware:
 *   - averaged 3-phase inverter with 400 ns dead-time error and the one-period
 *     PWM preload delay of TIM1
 *   - surface-PM hub motor in the dq frame (R, L, flux, pole pairs)
 *   - 120-degree hall sensors with an arbitrary mounting offset
 *   - wheel + rider inertia, rolling + aero load
 *   - battery with internal resistance
 *   - INA240 / ADC quantisation, offsets and noise
 * Each scenario checks the behaviour a rider would care about and the test
 * exits non-zero if anything is out of bounds.
 *
 * build & run:  make sim   (from firmware/)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli.h"
#include "config.h"
#include "hall.h"
#include "hw.h"
#include "mathx.h"
#include "motor.h"

GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
TIM_TypeDef sim_tim1, sim_tim2;
IWDG_TypeDef sim_iwdg;
DWT_Type sim_dwt;
CoreDebug_Type sim_coredebug;

/* ---------------------------------------------------------------- hw stubs */
static int pwm_on;
static double duty_pend[3] = {0.5, 0.5, 0.5}, duty_act[3] = {0.5, 0.5, 0.5};
static adc_raw_t adc_now;
static uint32_t sim_ms;

void pwm_enable(void) { pwm_on = 1; }
void pwm_disable(void) { pwm_on = 0; }
int pwm_is_enabled(void) { return pwm_on; }
void pwm_set(float a, float b, float c)
{
    duty_pend[0] = clampf(a, 0, 1);
    duty_pend[1] = clampf(b, 0, 1);
    duty_pend[2] = clampf(c, 0, 1);
}
void adc_read(adc_raw_t *r) { *r = adc_now; }
uint32_t millis(void) { return sim_ms; }
void delay_ms(uint32_t ms) { (void)ms; }
void power_hold(int on) { (void)on; }
void led_ok(int on) { (void)on; }
void led_fault(int on) { (void)on; }

/* UART: input from a test string, output to stdout */
static const char *uart_in = "";
void uart_write_n(const char *str, unsigned n) { fwrite(str, 1, n, stdout); }
void uart_write(const char *str) { fputs(str, stdout); }
int uart_getc(void) { return *uart_in ? (unsigned char)*uart_in++ : -1; }
unsigned uart_tx_free(void) { return 4096; }
int config_save(void) { return 0; }

/* ---------------------------------------------------------------- plant */
typedef struct {
    double R, L, flux, pp;
    double J, t_static, k_aero;
    double vbat, rint;
    double hall_off;           /* rad electrical */
    double theta, wm;          /* rotor electrical angle, mech speed rad/s */
    double id, iq;
    double ibus, vbus;
    double thr_v;
    int dir_rev, brake, hall_force;   /* hall_force: -1 normal else forced code */
    double t;                  /* seconds */
    /* stats */
    double max_iph, max_ibus_avg, ibus_acc;
    int ibus_n;
    double tdead;
} plant_t;

static plant_t P;

static uint32_t hall_code(double th)
{
    double h = th + P.hall_off;
    uint32_t a = sin(h) > 0, b = sin(h - 2 * M_PI / 3) > 0, c = sin(h - 4 * M_PI / 3) > 0;
    return a | (b << 1) | (c << 2);
}

static void set_inputs(void)
{
    uint32_t code = P.hall_force >= 0 ? (uint32_t)P.hall_force : hall_code(P.theta);
    uint32_t idr = (1u << 3);                       /* button not pressed */
    if (!P.dir_rev)
        idr |= 1u << 4;
    if (!P.brake)
        idr |= 1u << 5;
    idr |= ((code & 1) << 10) | (((code >> 1) & 1) << 11) | (((code >> 2) & 1) << 12);
    sim_gpiob.IDR = idr;
}

static double gauss(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static void phase_currents(double *ia, double *ib, double *ic)
{
    double c = cos(P.theta), s = sin(P.theta);
    double al = P.id * c - P.iq * s, be = P.id * s + P.iq * c;
    *ia = al;
    *ib = -0.5 * al + sqrt(3) / 2 * be;
    *ic = -0.5 * al - sqrt(3) / 2 * be;
}

/* integrate the plant for one PWM period in 1 us steps */
static void plant_period(void)
{
    const int N = 50;
    const double dt = 1.0 / PWM_HZ / N;
    double ib_sum = 0;
    for (int k = 0; k < N; k++) {
        double ia, ib, ic;
        phase_currents(&ia, &ib, &ic);
        double we = P.wm * P.pp;
        double vd, vq;
        if (pwm_on) {
            double i3[3] = {ia, ib, ic}, v[3];
            for (int j = 0; j < 3; j++) {
                double dte = P.tdead * PWM_HZ;        /* dead-time duty error */
                double d = duty_act[j] - (i3[j] > 0 ? dte : (i3[j] < 0 ? -dte : 0));
                v[j] = clampf((float)d, 0, 1) * P.vbus;
            }
            double al = (2 * v[0] - v[1] - v[2]) / 3, be = (v[1] - v[2]) / sqrt(3);
            double c = cos(P.theta), s = sin(P.theta);
            vd = al * c + be * s;
            vq = -al * s + be * c;
            ib_sum += duty_act[0] * ia + duty_act[1] * ib + duty_act[2] * ic;
        } else {
            /* all FETs off: diodes only conduct if BEMF exceeds the bus */
            double bemf = fabs(we * P.flux) * sqrt(3);
            if (bemf < P.vbus) {
                P.id = P.iq = 0;
                vd = vq = 0;
            } else {
                vd = -P.R * P.id;
                vq = (we > 0 ? 1 : -1) * P.vbus / sqrt(3);
            }
        }
        if (pwm_on || P.id != 0 || P.iq != 0) {
            double did = (vd - P.R * P.id + we * P.L * P.iq) / P.L;
            double diq = (vq - P.R * P.iq - we * P.L * P.id - we * P.flux) / P.L;
            P.id += did * dt;
            P.iq += diq * dt;
        }
        double te = 1.5 * P.pp * P.flux * P.iq;
        double tl = 0;
        if (fabs(P.wm) > 1e-3)
            tl = (P.wm > 0 ? 1 : -1) * (P.t_static + P.k_aero * P.wm * P.wm);
        else if (fabs(te) < P.t_static)
            tl = te;                                   /* stiction */
        P.wm += (te - tl) / P.J * dt;
        double prev_code = hall_code(P.theta);
        P.theta += P.wm * P.pp * dt;
        if (P.theta > 2 * M_PI)
            P.theta -= 2 * M_PI;
        if (P.theta < 0)
            P.theta += 2 * M_PI;
        P.t += dt;
        sim_tim2.CNT = (uint32_t)(P.t * 1e6);
        if (P.hall_force < 0 && hall_code(P.theta) != prev_code) {
            set_inputs();
            hall_edge_isr(sim_tim2.CNT);
        }
        double a2, b2, c2;
        phase_currents(&a2, &b2, &c2);
        double mx = fmax(fabs(a2), fmax(fabs(b2), fabs(c2)));
        if (mx > P.max_iph)
            P.max_iph = mx;
    }
    P.ibus = ib_sum / N;
    P.vbus = P.vbat - P.rint * P.ibus;
    P.ibus_acc += P.ibus;
    if (++P.ibus_n == PWM_HZ / 10) {                 /* 100 ms average */
        double avg = P.ibus_acc / P.ibus_n;
        if (avg > P.max_ibus_avg)
            P.max_ibus_avg = avg;
        P.ibus_acc = 0;
        P.ibus_n = 0;
    }
}

static uint16_t adc12(double v)
{
    double c = v / VREF * ADC_FS + gauss() * 1.5;
    return (uint16_t)clampf((float)c, 0, 4095);
}

static void sample_and_isr(void)
{
    double ia, ib, ic;
    phase_currents(&ia, &ib, &ic);
    const double vmid = VREF / 2;
    const double g = CSA_GAIN * SHUNT_OHM;
    adc_now.ia = adc12(vmid + 0.004 + ia * g);       /* small offset errors */
    adc_now.ib = adc12(vmid - 0.003 + ib * g);
    adc_now.ic = adc12(vmid + 0.002 + ic * g);
    adc_now.vbus = adc12(P.vbus / VBUS_DIV);
    adc_now.thr = adc12(P.thr_v / THR_DIV);
    adc_now.tfet = adc12(VREF / 2);                  /* 25 C */
    adc_now.tmot = adc12(VREF / 2);
    set_inputs();
    motor_isr();
}

static void run(double seconds, void (*each_ms)(void))
{
    long periods = (long)(seconds * PWM_HZ);
    for (long i = 0; i < periods; i++) {
        memcpy(duty_act, duty_pend, sizeof(duty_act));
        sample_and_isr();
        if (getenv("SIMTRACE") && P.t > atof(getenv("SIMTRACE")) && P.t < atof(getenv("SIMTRACE")) + 0.006)
            printf("  tr t=%.5f iq_ref=%.1f iq=%.1f id=%.1f ctl_iq=%.1f ctl_id=%.1f err=%.1f vd=%.2f vq=%.2f\n", P.t,
                   m.iq_ref, P.iq, P.id, m.iq, m.id, remainder(m.theta - P.theta, 2 * M_PI) * 57.3, m.vd, m.vq);
        plant_period();
        if ((i % (PWM_HZ / 1000)) == 0) {
            sim_ms++;
            motor_slow(sim_ms);
            if (each_ms)
                each_ms();
        }
    }
}

/* ---------------------------------------------------------------- checks */
static int failures;
#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (cond) {                                         \
            printf("  PASS  ");                             \
        } else {                                            \
            printf("  FAIL  ");                             \
            failures++;                                     \
        }                                                   \
        printf(__VA_ARGS__);                                \
        printf("\n");                                       \
    } while (0)

static double rpm(void) { return P.wm * 60 / (2 * M_PI); }
static double kmh(void) { return P.wm * 0.33 * 3.6; }     /* 26" wheel */

static void plant_defaults(void)
{
    memset(&P, 0, sizeof(P));
    P.R = 0.12;
    P.L = 250e-6;
    P.flux = 0.030;
    P.pp = 23;
    P.J = 0.08;                 /* lifted wheel */
    P.t_static = 0.3;
    P.k_aero = 0.0;
    P.vbat = 52.0;
    P.rint = 0.12;
    P.vbus = P.vbat;
    P.hall_off = 37.0 * M_PI / 180;
    P.thr_v = 0.85;             /* hall throttle at rest */
    P.hall_force = -1;
    P.tdead = 400e-9;
}

static void ride_mode(void)
{
    P.J = 10.0;                 /* 100 kg bike + rider on a 0.33 m wheel */
    P.t_static = 3.0;           /* rolling resistance */
    P.k_aero = 0.016;           /* ~ 0.5*rho*CdA*r^3 */
}

/* stats collected while riding */
static double iq_err_acc, iq_err_n, min_te_ratio = 10, worst_ang, worst_ang_t, worst_ang_w;
static double win[5], min_win_ratio = 10;
static int win_n;
static void ride_stats(void)
{
    double ang = fabs(remainder(m.theta - P.theta, 2 * M_PI)) * 180 / M_PI;
    if (m.state == ST_RUN && ang > worst_ang) {
        worst_ang = ang;
        worst_ang_t = P.t;
        worst_ang_w = P.wm * P.pp;
    }
    if (m.state == ST_RUN && fabs(m.iq_ref) > 5 && !(m.limits & LIM_VOLTAGE)) {
        double e = m.iq_ref - P.iq;
        iq_err_acc += e * e;
        iq_err_n++;
        double r = P.iq / m.iq_ref;
        win[win_n++ % 5] = r;
        if (win_n >= 5) {
            double a = (win[0] + win[1] + win[2] + win[3] + win[4]) / 5;
            if (a < min_win_ratio)
                min_win_ratio = a;
        }
        if (r < min_te_ratio) {
            min_te_ratio = r;
            if (getenv("SIMDBG"))
                printf("  dbg t=%.4f ratio=%.2f iq_ref=%.1f iq=%.1f id=%.1f ang_err=%.1f we=%.1f lim=%lx hall=%lu\n",
                       P.t, r, m.iq_ref, P.iq, P.id, remainder(m.theta - P.theta, 2 * M_PI) * 57.3,
                       P.wm * P.pp, (unsigned long)m.limits, (unsigned long)m.hall);
        }
    }
}

static void suite(void);

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    srand(1);

    printf("######## motor 1: direct-drive hub, 23 pole pairs, R=120 mOhm, L=250 uH ########\n");
    plant_defaults();
    suite();

    printf("\n######## motor 2: 20 pole pairs, R=300 mOhm, L=600 uH, flux 0.045, hall offset -100 deg ########\n");
    plant_defaults();
    P.R = 0.30;
    P.L = 600e-6;
    P.flux = 0.045;
    P.pp = 20;
    P.hall_off = -100.0 * M_PI / 180;
    suite();

    printf("\n######## UART command line ########\n");
    cfg.stream_ms = 0;
    uart_in = "help\nhdr\ntlm\nstatus\nget i_batt_max\nset i_batt_max 30.5\nset bogus 1\nset i_batt_max x\n"
              "stream 250\nstream off\nget hall_angle\nclear\nfoo\n";
    cli_poll(sim_ms);
    CHECK(fabsf(cfg.i_batt_max - 30.5f) < 1e-4f, "'set i_batt_max 30.5' applied (%.2f)", cfg.i_batt_max);
    CHECK(cfg.stream_ms == 0, "'stream off' applied");

    printf("\n%s (%d failure%s)\n", failures ? "SIMULATION FAILED" : "ALL SIMULATION CHECKS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

static void suite(void)
{
    iq_err_acc = iq_err_n = 0;
    min_te_ratio = min_win_ratio = 10;
    worst_ang = 0;
    win_n = 0;
    pwm_on = 0;
    config_defaults(&cfg);
    hall_init();
    set_inputs();

    printf("== boot ==\n");
    m.state = ST_CALIB;
    motor_init();                       /* returns immediately in sim (millis frozen) */
    run(0.5, NULL);
    CHECK(m.state == ST_IDLE, "state after calibration = %s", motor_state_name(m.state));
    CHECK((m.faults & ~FLT_NOT_DETECTED) == 0 && (m.faults & FLT_NOT_DETECTED),
          "only NOT_DETECTED flagged before detect (faults=0x%lx)", (unsigned long)m.faults);
    P.thr_v = 2.5;
    run(0.3, NULL);
    CHECK(m.state == ST_IDLE && fabs(P.wm) < 0.01, "refuses to run before halls are learned");
    P.thr_v = 0.85;
    run(0.3, NULL);

    printf("== detect (wheel lifted) ==\n");
    CHECK(motor_detect(0) == 0, "detect started");
    for (int i = 0; i < 400 && !motor_detect_msg(); i++)
        run(0.05, NULL);
    const char *msg = motor_detect_msg();
    printf("  firmware says: %s\n", msg ? msg : "(nothing)");
    CHECK(msg && strstr(msg, "OK"), "detect completes successfully");
    CHECK(fabs(cfg.motor_r - P.R) / P.R < 0.15, "R = %.1f mOhm (true %.1f)", cfg.motor_r * 1e3, P.R * 1e3);
    CHECK(fabs(cfg.motor_l - P.L) / P.L < 0.20, "L = %.0f uH (true %.0f)", cfg.motor_l * 1e6, P.L * 1e6);
    double worst = 0;
    for (int s = 1; s <= 6; s++) {
        /* true sector centre: average angle where hall_code == s */
        double cs = 0, sn = 0;
        for (int k = 0; k < 3600; k++) {
            double th = k * 2 * M_PI / 3600;
            if (hall_code(th) == (uint32_t)s) {
                cs += cos(th);
                sn += sin(th);
            }
        }
        double truth = atan2(sn, cs);
        double err = remainder(cfg.hall_angle[s] - truth, 2 * M_PI) * 180 / M_PI;
        if (fabs(err) > worst)
            worst = fabs(err);
    }
    CHECK(cfg.hall_valid && worst < 8.0, "hall table learned, worst sector error %.1f deg", worst);
    run(1.0, NULL);
    CHECK(m.state == ST_IDLE, "idle after detect");

    printf("== ride: full throttle from standstill (100 kg, 26\" wheel) ==\n");
    ride_mode();
    P.wm = 0;
    P.max_iph = P.max_ibus_avg = 0;
    P.thr_v = 4.1;
    run(0.5, ride_stats);
    double v05 = kmh();
    CHECK(P.wm > 0.5, "moving forward after 0.5 s: %.1f km/h", v05);
    printf("  worst angle error %.1f deg at t=%.3f s, omega_e %.1f rad/s\n", worst_ang, worst_ang_t, worst_ang_w);
    CHECK(min_win_ratio > 0.80,
          "torque (5 ms avg) never below 80%% of command: min %.0f%% (instantaneous min %.0f%% at first hall edge)",
          min_win_ratio * 100, min_te_ratio * 100);
    run(14.5, ride_stats);
    double rms = sqrt(iq_err_acc / fmax(iq_err_n, 1));
    printf("  speed after 15 s: %.1f km/h (%.0f rpm), Vbus %.1f V, Ibus %.1f A, mod %.2f\n", kmh(),
           rpm(), P.vbus, P.ibus, m.mod);
    CHECK(m.state == ST_RUN && m.faults == 0, "still running, no faults (0x%lx)", (unsigned long)m.faults);
    CHECK(P.max_iph < cfg.i_phase_max * 1.10 + 2, "peak phase current %.1f A (limit %.0f A)", P.max_iph,
          cfg.i_phase_max);
    CHECK(P.max_ibus_avg < cfg.i_batt_max * 1.07, "peak battery current (100 ms avg) %.1f A (limit %.0f A)",
          P.max_ibus_avg, cfg.i_batt_max);
    CHECK(rms < 3.0, "torque current tracking error %.2f A rms (outside voltage limit)", rms);
    CHECK(fabs(m.ibus - P.ibus) < 2.5, "battery current estimate %.1f A vs true %.1f A", m.ibus, P.ibus);
    CHECK(cfg.motor_flux > 0.8 * P.flux && cfg.motor_flux < 1.2 * P.flux,
          "learned flux linkage %.4f Wb (true %.4f)", cfg.motor_flux, P.flux);

    printf("== release throttle: coast ==\n");
    P.thr_v = 0.85;
    double v0 = kmh();
    run(0.2, NULL);
    double iq_avg = 0;
    for (int i = 0; i < 20; i++) {
        run(0.05, NULL);
        iq_avg += fabs(P.iq) / 20;
    }
    CHECK(iq_avg < 1.5, "no drag torque while coasting (|iq| avg %.2f A)", iq_avg);
    CHECK(kmh() < v0 && kmh() > 0.8 * v0, "coasting %.1f -> %.1f km/h", v0, kmh());

    printf("== re-apply throttle at speed ==\n");
    P.max_iph = 0;
    P.thr_v = 2.5;
    run(1.0, NULL);
    CHECK(m.state == ST_RUN && P.iq > 5 && P.max_iph < 70, "smooth pick-up, iq %.1f A, peak %.1f A", P.iq,
          P.max_iph);

    printf("== throttle wire breaks while riding ==\n");
    P.thr_v = 0.0;
    run(0.3, NULL);
    CHECK((m.faults & FLT_THROTTLE) && fabs(P.iq) < 2, "throttle fault, torque removed (iq %.1f A)", P.iq);
    P.thr_v = 0.85;
    run(1.5, NULL);
    CHECK(!(m.faults & FLT_THROTTLE), "fault clears when throttle is back at rest");

    printf("== stop, then reverse ==\n");
    P.k_aero = 0.016;
    P.t_static = 300.0;         /* brake hard (mechanically) to stop quickly */
    run(3.0, NULL);
    P.t_static = 3.0;
    CHECK(fabs(P.wm) < 0.05 && m.state == ST_IDLE, "stopped and idle (%s)", motor_state_name(m.state));
    P.dir_rev = 1;
    P.thr_v = 2.0;
    run(3.0, NULL);
    CHECK(P.wm < -0.5, "reverse: %.1f km/h", kmh());
    P.dir_rev = 0;              /* flip back to FWD while still rolling backwards */
    run(0.5, NULL);
    CHECK(fabs(P.iq) < 3 || P.wm > -0.3, "no forward torque until the wheel has stopped (iq %.1f A)", P.iq);
    P.thr_v = 0.85;
    P.t_static = 300.0;
    run(2.0, NULL);
    P.t_static = 3.0;

    printf("== brake lever cuts power ==\n");
    P.thr_v = 4.0;
    run(2.0, NULL);
    P.brake = 1;
    run(0.2, NULL);
    CHECK(fabs(P.iq) < 2, "brake: iq %.1f A with full throttle held", P.iq);
    P.brake = 0;
    P.thr_v = 0.85;
    run(0.5, NULL);

    printf("== hall connector unplugged while riding ==\n");
    P.thr_v = 3.0;
    run(1.0, NULL);
    P.hall_force = 7;
    run(0.05, NULL);
    CHECK((m.faults & FLT_HALL) && !pwm_on, "hall fault trips, PWM off within 50 ms");
    P.hall_force = -1;
    P.thr_v = 0.85;
    run(2.0, NULL);
    CHECK(!(m.faults & FLT_HALL), "hall fault clears after reconnect + throttle release");

    printf("== low battery fold-back ==\n");
    P.vbat = 40.5;
    P.thr_v = 4.1;
    run(2.0, NULL);
    CHECK((m.limits & LIM_UV) && P.ibus < 0.6 * cfg.i_batt_max, "under-voltage limits power (Ibus %.1f A at %.1f V)",
          P.ibus, P.vbus);
    P.vbat = 52;
    P.thr_v = 0.85;
    run(1.0, NULL);

    printf("== phase short: 120 A current spike while riding ==\n");
    P.thr_v = 3.0;
    run(1.0, NULL);
    P.iq = 120.0;                       /* e.g. motor phase shorted to the frame */
    run(0.0002, NULL);                  /* 4 PWM periods */
    CHECK((m.faults & FLT_OVERCURRENT) && !pwm_on,
          "hard overcurrent trip within 200 us, PWM off (faults 0x%lx)", (unsigned long)m.faults);
    P.thr_v = 0.85;
    run(2.0, NULL);
    CHECK(!(m.faults & FLT_OVERCURRENT) && m.state == ST_IDLE,
          "overcurrent clears after throttle release (%s)", motor_state_name(m.state));

    printf("== over-voltage ==\n");
    P.vbat = 62;
    run(0.1, NULL);
    CHECK((m.faults & FLT_OVERVOLTAGE) && !pwm_on, "over-voltage trips");
    P.vbat = 52;
    run(1.0, NULL);
    CHECK(!(m.faults & FLT_OVERVOLTAGE), "over-voltage clears");
}
