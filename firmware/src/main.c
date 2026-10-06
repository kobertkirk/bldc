/*
 * 48 V / 35 A hall-sensored hub motor controller - main loop.
 *
 * Power button behaviour (momentary switch to GND):
 *   press          -> board powers up, firmware latches PWR_HOLD
 *   hold 2 s       -> motor stops, releases PWR_HOLD when the button is let go
 * Boot shortcut: hold BRAKE + full throttle for 3 s after power-up with the
 * wheel lifted to run auto-detect without a UART cable (saves automatically).
 */
#include <math.h>

#include "cli.h"
#include "config.h"
#include "hall.h"
#include "hw.h"
#include "motor.h"

#define LONG_PRESS_MS 2000u

static void leds(uint32_t now)
{
    /* green: slow blink idle, solid running, fast blink detecting
       red  : fault code = number of blinks (lowest set fault bit + 1) */
    int on;
    switch (m.state) {
    case ST_RUN:
        on = 1;
        break;
    case ST_DET_R:
    case ST_DET_L:
    case ST_DET_HALL:
        on = (now / 100) & 1;
        break;
    default:
        on = (now % 1000) < 100;
        break;
    }
    led_ok(on);

    uint32_t f = m.faults & ~FLT_UNDERVOLTAGE;
    if (!f) {
        led_fault(m.faults & FLT_UNDERVOLTAGE ? (now % 2000) < 100 : 0);
        return;
    }
    int code = 0;
    while (!(f & 1u)) {
        f >>= 1;
        code++;
    }
    code++;
    uint32_t t = now % (uint32_t)(code * 400 + 1500);
    led_fault(t < (uint32_t)code * 400 && (t % 400) < 200);
}

static void shutdown(void)
{
    motor_shutdown();
    cli_printf(">shutting down\r\n");
    while (btn_pressed()) {           /* releasing the hold while pressed does nothing */
        iwdg_kick();
        led_ok(0);
        led_fault((millis() / 50) & 1);
    }
    delay_ms(50);
    power_hold(0);
    /* if still powered (e.g. via SWD) wait for another press and restart */
    uint32_t t0 = millis();
    for (;;) {
        iwdg_kick();
        led_fault(0);
        if (millis() - t0 > 500 && btn_pressed())
            NVIC_SystemReset();
    }
}

int main(void)
{
    hw_early_hold();
    hw_init();
    int loaded = config_load();
    hall_init();
    motor_init();
    cli_init();
    if (!loaded)
        cli_printf(">no saved config, using defaults\r\n");
    iwdg_start();

    uint32_t last_ms = millis(), btn_t0 = 0, last_isr = 0, idle_since = millis();
    uint32_t combo_t0 = 0;
    int btn_armed = 0, combo_used = 0, combo_save = 0;

    for (;;) {
        uint32_t now = millis();

        /* watchdog only if the control interrupt is alive */
        if (m.isr_count != last_isr) {
            last_isr = m.isr_count;
            iwdg_kick();
        }

        if (now != last_ms) {
            last_ms = now;
            motor_slow(now);
            leds(now);

            /* power button: ignore the press that switched us on */
            if (!btn_pressed()) {
                btn_armed = 1;
                btn_t0 = 0;
            } else if (btn_armed) {
                if (!btn_t0)
                    btn_t0 = now;
                else if (now - btn_t0 > LONG_PRESS_MS)
                    shutdown();
            }
            if (cli_shutdown_req)
                shutdown();

            /* auto power-off */
            if (m.state != ST_IDLE || m.omega_e != 0.0f)
                idle_since = now;
            if (cfg.auto_off_min && now - idle_since > cfg.auto_off_min * 60000u)
                shutdown();

            /* brake + full throttle at power-up -> detect */
            if (!combo_used && now < 10000) {
                if (brake_on() && m.thr_v > cfg.thr_max_v - 0.3f && m.thr_v < cfg.thr_fault_hi) {
                    if (!combo_t0)
                        combo_t0 = now;
                    if (now - combo_t0 > 3000) {
                        combo_used = 1;
                        cli_printf(">release throttle to start detect\r\n");
                    }
                } else {
                    combo_t0 = 0;
                }
            }
            if (combo_used == 1 && m.thr_pct == 0.0f) {
                motor_clear_faults();
                if (motor_detect(0) == 0) {
                    combo_used = 2;
                    combo_save = 1;
                }
            }
            if (combo_save && m.state != ST_DET_R && m.state != ST_DET_L &&
                m.state != ST_DET_HALL && motor_detect_msg()) {
                combo_save = 0;
                cli_printf(">%s\r\n", motor_detect_msg());
                if (cfg.hall_valid && !(m.faults & FLT_DETECT))
                    cli_printf(config_save() ? ">ERR save failed\r\n" : ">OK saved\r\n");
            }
        }
        cli_poll(now);
    }
}
