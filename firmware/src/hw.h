#ifndef HW_H
#define HW_H

#include <stdint.h>

#include "board.h"

void hw_early_hold(void);          /* latch power ASAP after reset          */
void hw_init(void);

/* PWM */
void pwm_enable(void);
void pwm_disable(void);
int pwm_is_enabled(void);
void pwm_set(float da, float db, float dc);   /* duties 0..1             */

/* raw ADC results, valid inside motor_isr() */
typedef struct {
    uint16_t ia, ic, vbus;         /* ADC1 injected */
    uint16_t ib, thr, tfet, tmot;  /* ADC2 injected */
} adc_raw_t;
void adc_read(adc_raw_t *r);

/* time */
uint32_t millis(void);
static inline uint32_t micros(void) { return TIM2->CNT; }   /* 1 MHz, 32 bit */
void delay_ms(uint32_t ms);

/* digital I/O */
static inline uint32_t hall_read(void)
{
    uint32_t idr = HALL_PORT->IDR;
    return ((idr >> HALL_A_PIN) & 1u) | (((idr >> HALL_B_PIN) & 1u) << 1) |
           (((idr >> HALL_C_PIN) & 1u) << 2);
}
static inline int btn_pressed(void) { return !(BTN_PORT->IDR & (1u << BTN_PIN)); }
static inline int dir_reverse(void) { return !(DIR_PORT->IDR & (1u << DIR_PIN)); }
static inline int brake_on(void) { return !(BRAKE_PORT->IDR & (1u << BRAKE_PIN)); }
void power_hold(int on);
void led_ok(int on);
void led_fault(int on);

/* UART (USART1, 115200 8N1, interrupt driven) */
void uart_write(const char *s);
void uart_write_n(const char *s, unsigned n);
int uart_getc(void);               /* -1 if none */
unsigned uart_tx_free(void);

/* watchdog */
void iwdg_start(void);
static inline void iwdg_kick(void) { IWDG->KR = 0xAAAAu; }

#endif
