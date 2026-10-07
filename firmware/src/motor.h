#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    ST_CALIB = 0,     /* measuring current sensor offsets            */
    ST_IDLE,          /* PWM off, waiting for throttle                */
    ST_RUN,           /* FOC running                                  */
    ST_FAULT,         /* PWM off until fault clears + throttle at 0   */
    ST_DET_R,         /* detect: resistance                           */
    ST_DET_L,         /* detect: inductance                           */
    ST_DET_HALL,      /* detect: hall table (wheel must spin freely!) */
    ST_OFF,           /* shutting down                                */
} motor_state_t;

/* fault bits */
#define FLT_OVERCURRENT   (1u << 0)
#define FLT_OVERVOLTAGE   (1u << 1)
#define FLT_UNDERVOLTAGE  (1u << 2)   /* below v_uv_cut (info, torque = 0)  */
#define FLT_HALL          (1u << 3)
#define FLT_THROTTLE      (1u << 4)
#define FLT_FET_TEMP      (1u << 5)
#define FLT_MOTOR_TEMP    (1u << 6)
#define FLT_NOT_DETECTED  (1u << 7)   /* hall table not learned yet         */
#define FLT_DETECT        (1u << 8)
#define FLT_CSA_OFFSET    (1u << 9)
#define FLT_THR_NOT_ZERO  (1u << 10)  /* throttle held at power-up          */
#define FLT_DRIVER        (1u << 11)  /* DRV8353 nFAULT (VDS OCP, gate, UVLO, OTSD) */

/* limiter flags (why torque is reduced) */
#define LIM_BATT          (1u << 0)
#define LIM_PHASE         (1u << 1)
#define LIM_UV            (1u << 2)
#define LIM_FET_TEMP      (1u << 3)
#define LIM_MOT_TEMP      (1u << 4)
#define LIM_SPEED         (1u << 5)
#define LIM_VOLTAGE       (1u << 6)   /* out of voltage headroom (top speed) */
#define LIM_STALL         (1u << 7)

typedef struct {
    volatile float vbus, ibus, ia, ib, ic, id, iq, iq_ref, vd, vq;
    volatile float omega_e;           /* rad/s electrical */
    volatile float theta;             /* rad electrical (estimated rotor angle) */
    volatile float mod;               /* modulation index 0..1 */
    volatile float thr_v, thr_pct;    /* throttle at connector, 0..1 */
    volatile float t_fet, t_mot;      /* deg C (computed in main loop) */
    volatile float power;             /* W (battery side) */
    volatile uint32_t hall;
    volatile int8_t dir;              /* +1 fwd, -1 rev (active) */
    volatile uint8_t brake;
    volatile uint32_t faults, limits;
    volatile motor_state_t state;
    volatile uint32_t isr_count;
    volatile uint32_t isr_cycles;     /* CPU cycles used by last ISR */
    /* raw slow channels for the main loop */
    volatile float raw_tfet, raw_tmot;
} motor_t;

extern motor_t m;
extern float wh_used, ah_used, wh_regen;
extern uint16_t drv_status1, drv_status2;    /* DRV8353 fault registers at the last trip */

void motor_init(void);               /* blocks ~100 ms for offset calibration */
void motor_isr(void);
void motor_slow(uint32_t now_ms);    /* 1 kHz from main loop */

int motor_detect(int what);          /* 0=all, 1=R, 2=L, 3=hall; -1 if busy */
const char *motor_detect_msg(void);  /* result text, NULL while running */
void motor_clear_faults(void);
void motor_driver_failed(void);      /* gate driver not configured: refuse to run */
void motor_shutdown(void);
void motor_update_gains(void);
const char *motor_state_name(motor_state_t s);
void motor_fault_names(uint32_t f, char *buf, unsigned len);

#endif
