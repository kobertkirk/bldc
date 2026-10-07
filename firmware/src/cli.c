/*
 * UART command line + telemetry (115200 8N1, lines end with CR or LF).
 *
 * Telemetry line (default every 100 ms, see 'stream'):
 *   $TLM,ms,state,faults,limits,vbus,ibus,power,ia,ib,ic,id,iq,iq_ref,erpm,rpm,
 *        mod,thr_v,thr_pct,dir,brake,hall,t_fet,t_mot,wh,ah,wh_regen
 * Replies to commands start with '>' so a host can separate them from telemetry.
 */
#include "cli.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "drv8353.h"
#include "hall.h"
#include "hw.h"
#include "mathx.h"
#include "motor.h"

#define FW_VERSION "bldc48 fw 1.0"

volatile int cli_shutdown_req;
static char line[96];
static unsigned line_len;
static uint32_t last_tlm;
static int det_waiting;

void cli_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
        uart_write_n(buf, (unsigned)n < sizeof(buf) ? (unsigned)n : sizeof(buf) - 1);
}

/* float -> text without pulling in printf float support */
static char *ftoa(char *p, float v, int dec)
{
    if (isnan(v)) {
        memcpy(p, "nan", 4);
        return p + 3;
    }
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    static const float pw[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    if (dec > 6)
        dec = 6;
    float scaled = v * pw[dec] + 0.5f;
    if (scaled > 4.0e9f)
        scaled = 4.0e9f;
    uint32_t iv = (uint32_t)scaled;
    uint32_t ip = iv / (uint32_t)pw[dec], fp = iv % (uint32_t)pw[dec];
    p += sprintf(p, "%lu", (unsigned long)ip);
    if (dec) {
        *p++ = '.';
        for (int d = dec - 1; d >= 0; d--) {
            p[d] = (char)('0' + fp % 10);
            fp /= 10;
        }
        p += dec;
    }
    *p = 0;
    return p;
}

static int parse_float(const char *s, float *out)
{
    float sign = 1, v = 0, scale = 0;
    int digits = 0, exp = 0, esign = 1;
    if (*s == '-' || *s == '+')
        sign = (*s++ == '-') ? -1 : 1;
    for (; *s; s++) {
        if (*s >= '0' && *s <= '9') {
            v = v * 10 + (float)(*s - '0');
            if (scale)
                scale *= 10;
            digits++;
        } else if (*s == '.' && !scale) {
            scale = 1;
        } else if (*s == 'e' || *s == 'E') {
            s++;
            if (*s == '-' || *s == '+')
                esign = (*s++ == '-') ? -1 : 1;
            while (*s >= '0' && *s <= '9')
                exp = exp * 10 + (*s++ - '0');
            break;
        } else {
            return -1;
        }
    }
    if (*s || !digits)
        return -1;
    if (scale)
        v /= scale;
    while (exp--)
        v = esign > 0 ? v * 10 : v / 10;
    *out = sign * v;
    return 0;
}

static void print_param(const param_t *p)
{
    char b[24];
    if (p->is_float)
        ftoa(b, *(float *)p->ptr, 6);
    else
        sprintf(b, "%lu", (unsigned long)*(uint32_t *)p->ptr);
    cli_printf(">%s = %s\r\n", p->name, b);
}

static void telemetry(uint32_t now)
{
    char b[400], *p = b;
    float erpm = m.omega_e * (60.0f / TWO_PI);
    p += sprintf(p, "$TLM,%lu,%s,%04lX,%02lX,", (unsigned long)now, motor_state_name(m.state),
                 (unsigned long)m.faults, (unsigned long)m.limits);
    const float v[] = {m.vbus, m.ibus, m.power, m.ia, m.ib, m.ic, m.id, m.iq, m.iq_ref, erpm,
                       erpm / cfg.pole_pairs, m.mod, m.thr_v, m.thr_pct * 100.0f};
    const int d[] = {2, 2, 1, 2, 2, 2, 2, 2, 2, 0, 1, 3, 3, 1};
    for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
        p = ftoa(p, v[i], d[i]);
        *p++ = ',';
    }
    p += sprintf(p, "%s,%d,%lu,", m.dir > 0 ? "FWD" : "REV", m.brake, (unsigned long)m.hall);
    p = ftoa(p, m.t_fet, 1);
    *p++ = ',';
    p = ftoa(p, m.t_mot, 1);
    *p++ = ',';
    p = ftoa(p, wh_used, 3);
    *p++ = ',';
    p = ftoa(p, ah_used, 4);
    *p++ = ',';
    p = ftoa(p, wh_regen, 3);
    p += sprintf(p, "\r\n");
    if (uart_tx_free() > (unsigned)(p - b))
        uart_write_n(b, (unsigned)(p - b));
}

static void status(void)
{
    char f[160], a[16], b2[16], c[16], d[16], e[16];
    motor_fault_names(m.faults, f, sizeof(f));
    ftoa(a, m.vbus, 2);
    ftoa(b2, m.ibus, 2);
    ftoa(c, m.thr_v, 2);
    ftoa(d, m.t_fet, 1);
    ftoa(e, m.omega_e * (60.0f / TWO_PI) / cfg.pole_pairs, 1);
    cli_printf(">state %s  faults %s  limits 0x%02lX\r\n", motor_state_name(m.state), f,
               (unsigned long)m.limits);
    cli_printf(">vbus %s V  ibus %s A  rpm %s  throttle %s V  dir %s  brake %d  hall %lu  "
               "t_fet %s C\r\n",
               a, b2, e, c, m.dir > 0 ? "FWD" : "REV", m.brake, (unsigned long)m.hall, d);
    cli_printf(">isr %lu cycles of %lu, hall table %s\r\n", (unsigned long)m.isr_cycles,
               (unsigned long)(SYSCLK_HZ / PWM_HZ), cfg.hall_valid ? "valid" : "NOT LEARNED (run 'detect')");
}

static void help(void)
{
    uart_write(">" FW_VERSION "\r\n"
               ">  status              one-shot human readable status\r\n"
               ">  tlm                 one telemetry line\r\n"
               ">  hdr                 telemetry column names\r\n"
               ">  stream <ms>|off     periodic telemetry (default 100 ms)\r\n"
               ">  get [name]          show parameter(s)\r\n"
               ">  set <name> <value>  change parameter (RAM)\r\n"
               ">  save | defaults     write config to flash | restore defaults (RAM)\r\n"
               ">  detect [r|l|hall]   measure motor R, L and learn halls (WHEEL OFF GROUND)\r\n"
               ">  clear               clear latched faults\r\n"
               ">  drv                 gate driver (DRV8353) fault registers\r\n"
               ">  off | reboot        power off | restart MCU\r\n");
}

static void command(char *s)
{
    char *argv[4] = {0};
    int argc = 0;
    for (char *t = strtok(s, " \t"); t && argc < 4; t = strtok(NULL, " \t"))
        argv[argc++] = t;
    if (!argc)
        return;
    const char *c = argv[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) {
        help();
    } else if (!strcmp(c, "status")) {
        status();
    } else if (!strcmp(c, "tlm")) {
        telemetry(millis());
    } else if (!strcmp(c, "hdr")) {
        cli_printf(">$TLM,ms,state,faults,limits,vbus,ibus,power,ia,ib,ic,id,iq,iq_ref,erpm,rpm,"
                   "mod,thr_v,thr_pct,dir,brake,hall,t_fet,t_mot,wh,ah,wh_regen\r\n");
    } else if (!strcmp(c, "stream")) {
        if (argc > 1 && strcmp(argv[1], "off")) {
            float v;
            if (parse_float(argv[1], &v) || v < 10 || v > 60000) {
                cli_printf(">ERR stream period 10..60000 ms\r\n");
                return;
            }
            cfg.stream_ms = (uint32_t)v;
        } else {
            cfg.stream_ms = 0;
        }
        cli_printf(">OK stream %lu\r\n", (unsigned long)cfg.stream_ms);
    } else if (!strcmp(c, "get")) {
        for (unsigned i = 0; i < n_params; i++)
            if (argc < 2 || !strcmp(argv[1], params[i].name))
                print_param(&params[i]);
        if (argc < 2 || !strcmp(argv[1], "hall_angle")) {
            for (int i = 1; i <= 6; i++) {
                char b[16];
                ftoa(b, cfg.hall_angle[i] / DEG, 1);
                cli_printf(">hall_angle[%d] = %s deg\r\n", i, b);
            }
        }
    } else if (!strcmp(c, "set")) {
        if (argc < 3) {
            cli_printf(">ERR usage: set <name> <value>\r\n");
            return;
        }
        float v;
        if (parse_float(argv[2], &v)) {
            cli_printf(">ERR bad number\r\n");
            return;
        }
        for (unsigned i = 0; i < n_params; i++) {
            if (!strcmp(argv[1], params[i].name)) {
                if (m.state == ST_RUN && strcmp(argv[1], "stream_ms")) {
                    cli_printf(">ERR stop the motor first\r\n");
                    return;
                }
                if (params[i].is_float)
                    *(float *)params[i].ptr = v;
                else
                    *(uint32_t *)params[i].ptr = (uint32_t)v;
                motor_update_gains();
                hall_init();
                print_param(&params[i]);
                return;
            }
        }
        cli_printf(">ERR unknown parameter (try 'get')\r\n");
    } else if (!strcmp(c, "save")) {
        if (m.state == ST_RUN || (m.state >= ST_DET_R && m.state <= ST_DET_HALL)) {
            cli_printf(">ERR stop the motor first\r\n");
            return;
        }
        cli_printf(config_save() ? ">ERR flash write failed\r\n" : ">OK saved\r\n");
    } else if (!strcmp(c, "defaults")) {
        if (m.state == ST_RUN) {
            cli_printf(">ERR stop the motor first\r\n");
            return;
        }
        config_defaults(&cfg);
        motor_update_gains();
        hall_init();
        cli_printf(">OK defaults loaded (not saved)\r\n");
    } else if (!strcmp(c, "detect")) {
        int what = 0;
        if (argc > 1)
            what = !strcmp(argv[1], "r") ? 1 : !strcmp(argv[1], "l") ? 2 : !strcmp(argv[1], "hall") ? 3 : -1;
        if (what < 0) {
            cli_printf(">ERR detect [r|l|hall]\r\n");
            return;
        }
        int rc = motor_detect(what);
        if (rc == -1)
            cli_printf(">ERR motor busy\r\n");
        else if (rc == -2)
            cli_printf(">ERR clear faults / check supply first\r\n");
        else if (rc == -3)
            cli_printf(">ERR release the throttle\r\n");
        else {
            cli_printf(">detect running - wheel must spin freely...\r\n");
            det_waiting = 1;
        }
    } else if (!strcmp(c, "clear")) {
        motor_clear_faults();
        cli_printf(">OK\r\n");
    } else if (!strcmp(c, "drv")) {
        /* DRV8353 fault words: live, and as captured at the last driver trip */
        cli_printf(">DRV8353 nFAULT=%s status1=0x%03X status2=0x%03X (last trip 0x%03X 0x%03X)\r\n",
                   drv_fault_active() ? "LOW" : "high", drv_read(DRV_REG_FAULT1),
                   drv_read(DRV_REG_FAULT2), drv_status1, drv_status2);
    } else if (!strcmp(c, "off")) {
        cli_printf(">powering off\r\n");
        cli_shutdown_req = 1;
    } else if (!strcmp(c, "reboot")) {
        cli_printf(">rebooting\r\n");
        delay_ms(20);
        NVIC_SystemReset();
    } else {
        cli_printf(">ERR unknown command '%s' (help)\r\n", c);
    }
}

void cli_init(void)
{
    cli_printf("\r\n>" FW_VERSION " ready, 'help' for commands\r\n");
    if (!cfg.hall_valid)
        cli_printf(">WARNING: hall table not learned. Lift the wheel and run 'detect', then 'save'.\r\n");
}

void cli_poll(uint32_t now)
{
    int ch;
    while ((ch = uart_getc()) >= 0) {
        if (ch == '\r' || ch == '\n') {
            line[line_len] = 0;
            if (line_len)
                command(line);
            line_len = 0;
        } else if (ch == 8 || ch == 127) {
            if (line_len)
                line_len--;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = (char)ch;
        }
    }
    if (det_waiting) {
        const char *msg = motor_detect_msg();
        if (msg) {
            cli_printf(">%s\r\n", msg);
            det_waiting = 0;
        }
    }
    if (cfg.stream_ms && now - last_tlm >= cfg.stream_ms) {
        last_tlm = now;
        telemetry(now);
    }
}
