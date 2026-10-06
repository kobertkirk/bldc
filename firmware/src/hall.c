/*
 * Hall sensor decoding with angle interpolation.
 *
 * Each hall state s has a learned electrical "center" angle.  A transition
 * s1 -> s2 happens at the midpoint of the two centers; between edges the
 * angle is extrapolated with the measured speed (averaged over a full
 * electrical revolution when available, which cancels sensor placement
 * error).  At very low speed the sector center is used directly, which still
 * gives >= 87 % torque, so starting from standstill is smooth.
 */
#include "hall.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hw.h"
#include "mathx.h"

#define LOW_SPEED_US   100000u     /* no edge for 100 ms -> treat as stopped */
#define HIST           6u

static float center[8];
static int8_t next_fwd[8], next_rev[8];
static float boundary[8][8];
static uint8_t table_ok;

static volatile uint32_t cur_state;
static volatile uint32_t edge_t, edge_count;
static volatile float edge_theta;
static volatile int8_t edge_dir;
static volatile uint32_t hist_t[HIST];
static volatile uint8_t hist_n;        /* consecutive same-direction edges */
static volatile uint8_t hist_i;
static volatile float omega_meas;

void hall_init(void)
{
    table_ok = 0;
    memset(next_fwd, -1, sizeof(next_fwd));
    memset(next_rev, -1, sizeof(next_rev));
    cur_state = hall_read();
    hist_n = 0;
    omega_meas = 0.0f;
    if (!cfg.hall_valid)
        return;
    for (int s = 0; s < 8; s++)
        center[s] = cfg.hall_angle[s];
    /* order the six states by angle */
    for (int s = 1; s <= 6; s++) {
        float best_f = 10.0f, best_r = 10.0f;
        for (int t = 1; t <= 6; t++) {
            if (t == s)
                continue;
            float d = wrap_2pi(center[t] - center[s]);      /* 0..2pi ahead */
            if (d < best_f) {
                best_f = d;
                next_fwd[s] = (int8_t)t;
            }
            float r = wrap_2pi(center[s] - center[t]);
            if (r < best_r) {
                best_r = r;
                next_rev[s] = (int8_t)t;
            }
        }
    }
    for (int s = 1; s <= 6; s++) {
        int f = next_fwd[s];
        boundary[s][f] = wrap_2pi(center[s] + 0.5f * wrap_2pi(center[f] - center[s]));
        boundary[f][s] = boundary[s][f];
    }
    table_ok = 1;
}

void hall_edge_isr(uint32_t t)
{
    uint32_t s = hall_read();
    uint32_t prev = cur_state;
    if (s == prev)
        return;                        /* glitch / bounce */
    cur_state = s;
    edge_count++;                      /* raw edge count (also used by detect) */
    if (!table_ok || s == 0 || s == 7 || prev == 0 || prev == 7) {
        hist_n = 0;
        return;
    }
    int8_t dir;
    if (next_fwd[prev] == (int8_t)s)
        dir = 1;
    else if (next_rev[prev] == (int8_t)s)
        dir = -1;
    else {                             /* skipped a state: restart estimate */
        hist_n = 0;
        edge_dir = 0;
        return;
    }
    uint32_t dt = t - edge_t;
    if (dir != edge_dir || dt > LOW_SPEED_US)
        hist_n = 0;
    if (hist_n > 0) {
        float w;
        if (hist_n >= HIST) {
            uint32_t span = t - hist_t[hist_i];          /* oldest entry */
            w = TWO_PI * 1e6f / (float)span;
        } else {
            w = (PI_F / 3.0f) * 1e6f / (float)dt;
        }
        omega_meas = dir > 0 ? w : -w;
    } else {
        omega_meas = 0.0f;
    }
    hist_t[hist_i] = t;
    hist_i = (uint8_t)((hist_i + 1) % HIST);
    if (hist_n < 255)
        hist_n++;
    edge_t = t;
    edge_dir = dir;
    edge_theta = boundary[prev][s];
}

void hall_get(uint32_t now, float *theta, float *omega)
{
    __disable_irq();
    uint32_t s = cur_state, et = edge_t;
    float eth = edge_theta, w = omega_meas;
    uint8_t n = hist_n;
    __enable_irq();

    if (!table_ok || s == 0 || s == 7) {
        *theta = 0.0f;
        *omega = 0.0f;
        return;
    }
    uint32_t el = now - et;
    if (n < 2 || el > LOW_SPEED_US || w == 0.0f) {
        *theta = center[s];
        *omega = 0.0f;
        return;
    }
    /* the motor cannot be faster than one sector per elapsed time */
    float wmax = (PI_F / 3.0f) * 1e6f / (float)el;
    if (fabsf(w) > wmax)
        w = w > 0 ? wmax : -wmax;
    float adv = w * (float)el * 1e-6f;
    const float lim = PI_F / 3.0f;
    if (adv > lim)
        adv = lim;
    if (adv < -lim)
        adv = -lim;
    *theta = wrap_2pi(eth + adv);
    *omega = w;
}

uint32_t hall_state(void) { return cur_state; }
uint32_t hall_edge_count(void) { return edge_count; }
uint32_t hall_ms_since_edge(uint32_t now) { return (now - edge_t) / 1000u; }

/* ------------------------------------------------------------------------ */
static float acc_c[8], acc_s[8];
static uint32_t acc_n[8];

void hall_learn_reset(void)
{
    memset(acc_c, 0, sizeof(acc_c));
    memset(acc_s, 0, sizeof(acc_s));
    memset(acc_n, 0, sizeof(acc_n));
}

void hall_learn_sample(float theta, uint32_t state)
{
    float s, c;
    sincos_f(theta, &s, &c);
    acc_c[state & 7] += c;
    acc_s[state & 7] += s;
    acc_n[state & 7]++;
}

int hall_learn_finish(char *msg, unsigned len)
{
    if (acc_n[0] || acc_n[7]) {
        snprintf(msg, len, "hall state 0/7 seen (%lu/%lu samples): check wiring/5V",
                 (unsigned long)acc_n[0], (unsigned long)acc_n[7]);
        return -1;
    }
    float ang[8];
    for (int s = 1; s <= 6; s++) {
        if (acc_n[s] < 50) {
            snprintf(msg, len, "hall state %d never seen: motor blocked or sensor dead", s);
            return -1;
        }
        ang[s] = wrap_2pi(atan2f(acc_s[s], acc_c[s]));
    }
    /* sanity: neighbours should be 60 deg (+-25) apart */
    for (int s = 1; s <= 6; s++) {
        float best = 10.0f;
        for (int t = 1; t <= 6; t++)
            if (t != s) {
                float d = wrap_2pi(ang[t] - ang[s]);
                if (d < best)
                    best = d;
            }
        if (best < 35.0f * DEG || best > 85.0f * DEG) {
            snprintf(msg, len, "hall spacing %d deg at state %d: not 120-deg halls?",
                     (int)(best / DEG), s);
            return -1;
        }
    }
    cfg.hall_angle[0] = cfg.hall_angle[7] = NAN;
    for (int s = 1; s <= 6; s++)
        cfg.hall_angle[s] = ang[s];
    cfg.hall_valid = 1;
    hall_init();
    snprintf(msg, len, "hall ok");
    return 0;
}
