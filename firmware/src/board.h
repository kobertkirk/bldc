/*
 * Board definition: pin map and analog scaling.  Must match
 * hardware/tools/circuit.py (the schematic is generated from that file).
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx.h"

#define SYSCLK_HZ        170000000u
#define PWM_HZ           20000u
#define PWM_ARR          (SYSCLK_HZ / (2u * PWM_HZ))     /* center aligned: 4250 */
#define CTRL_DT          (1.0f / PWM_HZ)
#define DEADTIME_TICKS   68u                             /* 68 * 5.88 ns = 400 ns */

/* ---- GPIO --------------------------------------------------------------- */
/* TIM1 PWM: PA8/PA9/PA10 = CH1..3 (AF6), PB13/PB14 = CH1N/CH2N (AF6), PB15 = CH3N (AF4)
 *           -> DRV8353RS INHx / INLx (6x PWM mode)                                    */
/* ADC1: PA0 IN1 = I_A, PA2 IN3 = I_C, PA3 IN4 = VBUS                                  */
/* ADC2: PA1 IN2 = I_B, PA4 IN17 = THROTTLE, PA5 IN13 = TEMP_FET, PA6 IN3 = TEMP_MOTOR */
/*       (I_A and I_B convert at the same time on the two ADCs)                        */
/* SPI1 (AF5): PB3 SCK, PB4 MISO (DRV SDO), PB5 MOSI (DRV SDI); PA15 nSCS (GPIO)       */
#define HALL_PORT        GPIOB
#define HALL_A_PIN       10u
#define HALL_B_PIN       11u
#define HALL_C_PIN       12u
/* USART1: PB6 TX, PB7 RX (AF7) */
#define BTN_PORT         GPIOA      /* PA11: power button sense, low = pressed */
#define BTN_PIN          11u
#define DIR_PORT         GPIOA      /* PA12: FWD/REV switch, low = reverse     */
#define DIR_PIN          12u
#define BRAKE_PORT       GPIOC      /* PC14: brake switch, low = braking       */
#define BRAKE_PIN        14u
#define DRV_CS_PORT      GPIOA      /* PA15: DRV8353 nSCS                      */
#define DRV_CS_PIN       15u
#define DRV_EN_PORT      GPIOB      /* PB0: DRV8353 ENABLE, high = awake       */
#define DRV_EN_PIN       0u
#define DRV_FLT_PORT     GPIOB      /* PB1: DRV8353 nFAULT, low = fault        */
#define DRV_FLT_PIN      1u
#define HOLD_PORT        GPIOB      /* PB9: power latch hold, high = stay on   */
#define HOLD_PIN         9u
#define LED_OK_PORT      GPIOC      /* PC13: green status LED                  */
#define LED_OK_PIN       13u
#define LED_FLT_PORT     GPIOB      /* PB2: red fault LED                      */
#define LED_FLT_PIN      2u

/* ---- Analog scaling ----------------------------------------------------- */
#define VREF             3.3f
#define ADC_FS           4096.0f
#define SHUNT_OHM        0.0005f                              /* low-side Kelvin shunts */
#define CSA_GAIN         20.0f                                /* DRV8353 CSA, set over SPI */
/* Low-side shunts only see a phase current while its low-side FET conducts,
 * i.e. around the PWM counter peak.  Above this duty the window is too short
 * (CSA settling + dead time), so that phase is rebuilt from the other two. */
#define DUTY_MEAS_MAX    0.90f
#define DUTY_MAX         0.97f      /* charge pump: 100 % allowed; keep a margin */
#define ADC_TRIG_LEAD    40u        /* TIM1 ticks before the peak (235 ns)       */
#define AMPS_PER_COUNT   (VREF / ADC_FS / (CSA_GAIN * SHUNT_OHM))   /* 0.0806 A */
#define VBUS_DIV         ((100.0f + 5.6f) / 5.6f)
#define VOLTS_PER_COUNT  (VREF / ADC_FS * VBUS_DIV)            /* 0.0152 V */
#define THR_DIV          ((12.0f + 22.0f) / 22.0f)
#define NTC_PULLUP_OHM   10000.0f
#define NTC_FET_BETA     3435.0f
#define NTC_R25          10000.0f

#define HARD_TRIP_AMPS   80.0f       /* instantaneous phase current trip */
#define ADC_FULLSCALE_A  (2048.0f * AMPS_PER_COUNT)         /* +/-165 A */

/* The hard trip must sit well inside what the current sense can report,
   otherwise a saturated ADC reading never reaches it. */
_Static_assert(HARD_TRIP_AMPS < 0.9f * (2048.0f * 3.3f / 4096.0f / (CSA_GAIN * SHUNT_OHM)),
               "overcurrent trip above current-sense full scale");

#endif
