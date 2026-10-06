/* Host-simulation stand-in for the CMSIS device header: just the registers
 * that the control code (motor.c, hall.c, hw.h inlines) touches. */
#ifndef SIM_STM32G4XX_H
#define SIM_STM32G4XX_H
#include <stdint.h>

typedef struct { volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CNT, BDTR, CCR1, CCR2, CCR3; } TIM_TypeDef;
typedef struct { volatile uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;

extern GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
extern TIM_TypeDef sim_tim1, sim_tim2;
extern IWDG_TypeDef sim_iwdg;
extern DWT_Type sim_dwt;
extern CoreDebug_Type sim_coredebug;

#define GPIOA (&sim_gpioa)
#define GPIOB (&sim_gpiob)
#define GPIOC (&sim_gpioc)
#define TIM1 (&sim_tim1)
#define TIM2 (&sim_tim2)
#define IWDG (&sim_iwdg)
#define DWT (&sim_dwt)
#define CoreDebug (&sim_coredebug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1u

static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void NVIC_SystemReset(void) {}
#endif
