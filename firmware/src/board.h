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
/* TIM1 PWM: PA8/PA9/PA10 = CH1..3 (AF6), PB13/PB14 = CH1N/CH2N (AF6), PB15 = CH3N (AF4) */
/* ADC1: PA0 IN1 = I_A, PA1 IN2 = I_B, PA2 IN3 = I_C, PA3 IN4 = VBUS            */
/* ADC2: PA4 IN17 = THROTTLE, PA5 IN13 = TEMP_FET, PA6 IN3 = TEMP_MOTOR          */
#define HALL_PORT        GPIOB
#define HALL_A_PIN       10u
#define HALL_B_PIN       11u
#define HALL_C_PIN       12u
/* USART1: PB6 TX, PB7 RX (AF7) */
#define BTN_PORT         GPIOB      /* PB3: power button sense, low = pressed  */
#define BTN_PIN          3u
#define DIR_PORT         GPIOB      /* PB4: FWD/REV switch, low = reverse      */
#define DIR_PIN          4u
#define BRAKE_PORT       GPIOB      /* PB5: brake switch, low = braking        */
#define BRAKE_PIN        5u
#define HOLD_PORT        GPIOB      /* PB9: power latch hold, high = stay on   */
#define HOLD_PIN         9u
#define LED_OK_PORT      GPIOC      /* PC13: green status LED                  */
#define LED_OK_PIN       13u
#define LED_FLT_PORT     GPIOB      /* PB2: red fault LED                      */
#define LED_FLT_PIN      2u

/* ---- Analog scaling ----------------------------------------------------- */
#define VREF             3.3f
#define ADC_FS           4096.0f
#define SHUNT_OHM        0.0005f
#define CSA_GAIN         50.0f                                /* INA240A2 */
#define AMPS_PER_COUNT   (VREF / ADC_FS / (CSA_GAIN * SHUNT_OHM))   /* 0.0322 A */
#define VBUS_DIV         ((100.0f + 5.6f) / 5.6f)
#define VOLTS_PER_COUNT  (VREF / ADC_FS * VBUS_DIV)            /* 0.0152 V */
#define THR_DIV          ((12.0f + 22.0f) / 22.0f)
#define NTC_PULLUP_OHM   10000.0f
#define NTC_FET_BETA     3435.0f
#define NTC_R25          10000.0f

#define HARD_TRIP_AMPS   80.0f       /* instantaneous phase current trip */
#define ADC_FULLSCALE_A  (2048.0f * AMPS_PER_COUNT)

#endif
