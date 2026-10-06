/*
 * STM32G431 peripheral setup (register level, no HAL).
 *
 * TIM1  : 20 kHz center-aligned complementary PWM, 400 ns dead time,
 *         TRGO on update (RCR=1 -> once per PWM period) triggers the ADCs.
 * ADC1  : injected I_A, I_B, I_C, VBUS  -> JEOS interrupt = control loop
 * ADC2  : injected THROTTLE, T_FET, T_MOTOR on the same trigger
 * TIM2  : free running 1 MHz timestamp for hall edges
 * USART1: 115200 8N1 telemetry / command port
 */
#include "hw.h"

#include "hall.h"
#include "motor.h"

static volatile uint32_t ms_ticks;

/* ------------------------------------------------------------------------ */
static void gpio_mode(GPIO_TypeDef *p, unsigned pin, unsigned mode, unsigned af)
{
    p->MODER = (p->MODER & ~(3u << (2 * pin))) | (mode << (2 * pin));
    if (mode == 2u) {
        volatile uint32_t *afr = &p->AFR[pin >> 3];
        unsigned sh = 4 * (pin & 7u);
        *afr = (*afr & ~(0xFu << sh)) | (af << sh);
        p->OSPEEDR |= 3u << (2 * pin);
    }
}

static void gpio_pull(GPIO_TypeDef *p, unsigned pin, unsigned pupd)
{
    p->PUPDR = (p->PUPDR & ~(3u << (2 * pin))) | (pupd << (2 * pin));
}

void power_hold(int on) { HOLD_PORT->BSRR = on ? (1u << HOLD_PIN) : (1u << (HOLD_PIN + 16)); }
void led_ok(int on) { LED_OK_PORT->BSRR = on ? (1u << LED_OK_PIN) : (1u << (LED_OK_PIN + 16)); }
void led_fault(int on) { LED_FLT_PORT->BSRR = on ? (1u << LED_FLT_PIN) : (1u << (LED_FLT_PIN + 16)); }

void hw_early_hold(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    power_hold(1);
    gpio_mode(HOLD_PORT, HOLD_PIN, 1u, 0);
}

/* ------------------------------------------------------------------------ */
static void clock_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR5 &= ~PWR_CR5_R1MODE;                       /* range 1 boost */
    FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS)
        ;
    /* HSI16 / 4 * 85 / 2 = 170 MHz */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | (3u << RCC_PLLCFGR_PLLM_Pos) |
                   (85u << RCC_PLLCFGR_PLLN_Pos) | (0u << RCC_PLLCFGR_PLLR_Pos) |
                   RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;
    /* step through AHB/2 when going above 80 MHz (RM0440 6.1.5) */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_SW)) | RCC_CFGR_HPRE_3 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
    for (volatile int i = 0; i < 200; i++)
        ;
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    SystemCoreClock = SYSCLK_HZ;
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    led_ok(0);
    led_fault(0);
    gpio_mode(LED_OK_PORT, LED_OK_PIN, 1u, 0);
    gpio_mode(LED_FLT_PORT, LED_FLT_PIN, 1u, 0);

    gpio_mode(BTN_PORT, BTN_PIN, 0u, 0);     /* external pull-ups on board */
    gpio_pull(BTN_PORT, BTN_PIN, 0u);
    gpio_mode(DIR_PORT, DIR_PIN, 0u, 0);
    gpio_pull(DIR_PORT, DIR_PIN, 1u);
    gpio_mode(BRAKE_PORT, BRAKE_PIN, 0u, 0);
    gpio_pull(BRAKE_PORT, BRAKE_PIN, 1u);

    gpio_mode(HALL_PORT, HALL_A_PIN, 0u, 0);
    gpio_mode(HALL_PORT, HALL_B_PIN, 0u, 0);
    gpio_mode(HALL_PORT, HALL_C_PIN, 0u, 0);

    /* analog inputs PA0..PA6 are analog after reset; set explicitly anyway */
    for (unsigned pin = 0; pin <= 6; pin++)
        gpio_mode(GPIOA, pin, 3u, 0);

    /* EXTI10..12 on port B, both edges */
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(SYSCFG_EXTICR3_EXTI10 | SYSCFG_EXTICR3_EXTI11)) |
                        SYSCFG_EXTICR3_EXTI10_PB | SYSCFG_EXTICR3_EXTI11_PB;
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~SYSCFG_EXTICR4_EXTI12) | SYSCFG_EXTICR4_EXTI12_PB;
    uint32_t mask = (1u << HALL_A_PIN) | (1u << HALL_B_PIN) | (1u << HALL_C_PIN);
    EXTI->RTSR1 |= mask;
    EXTI->FTSR1 |= mask;
    EXTI->PR1 = mask;
    EXTI->IMR1 |= mask;
}

static void tim2_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;
    TIM2->PSC = SYSCLK_HZ / 1000000u - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
}

static void tim1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;           /* center aligned 1 */
    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;
    TIM1->RCR = 1;                                      /* 1 update / period */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_ARR / 2;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;
    /* MOE=0 + OSSI=1 -> all six gates held low (idle level) */
    TIM1->BDTR = (DEADTIME_TICKS << TIM_BDTR_DTG_Pos) | TIM_BDTR_OSSR | TIM_BDTR_OSSI;
    TIM1->CR2 = 2u << TIM_CR2_MMS_Pos;                  /* TRGO = update */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;

    /* only now hand the pins to the timer */
    gpio_mode(GPIOA, 8, 2u, 6);
    gpio_mode(GPIOA, 9, 2u, 6);
    gpio_mode(GPIOA, 10, 2u, 6);
    gpio_mode(GPIOB, 13, 2u, 6);
    gpio_mode(GPIOB, 14, 2u, 6);
    gpio_mode(GPIOB, 15, 2u, 4);                        /* CH3N is AF4 on PB15 */
}

void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

int pwm_is_enabled(void)
{
    return (TIM1->BDTR & TIM_BDTR_MOE) != 0;
}

static inline uint32_t duty_to_ccr(float d)
{
    if (d < 0.0f)
        d = 0.0f;
    if (d > 1.0f)
        d = 1.0f;
    return (uint32_t)(d * (float)PWM_ARR + 0.5f);
}

void pwm_set(float da, float db, float dc)
{
    TIM1->CCR1 = duty_to_ccr(da);
    TIM1->CCR2 = duty_to_ccr(db);
    TIM1->CCR3 = duty_to_ccr(dc);
}

/* ------------------------------------------------------------------------ */
static void adc_enable(ADC_TypeDef *adc)
{
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    delay_ms(1);                                        /* tADCVREG_STUP 20 us */
    adc->CR &= ~ADC_CR_ADCALDIF;
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL)
        ;
    for (volatile int i = 0; i < 100; i++)
        ;
    adc->ISR = ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;
    while (!(adc->ISR & ADC_ISR_ADRDY))
        ;
}

static void adc_init(void)
{
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | RCC_CCIPR_ADC12SEL_1;   /* SYSCLK */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    (void)RCC->AHB2ENR;
    ADC12_COMMON->CCR = ADC_CCR_PRESC_1;                /* async /4 = 42.5 MHz */

    adc_enable(ADC1);
    adc_enable(ADC2);

    /* sample times: currents/vbus 12.5 cycles, slow channels 47.5 cycles */
    ADC1->SMPR1 = (2u << (3 * 1)) | (2u << (3 * 2)) | (2u << (3 * 3)) | (2u << (3 * 4));
    ADC2->SMPR1 = (4u << (3 * 3));
    ADC2->SMPR2 = (4u << (3 * (13 - 10))) | (4u << (3 * (17 - 10)));

    ADC1->CFGR |= ADC_CFGR_JQDIS;
    ADC2->CFGR |= ADC_CFGR_JQDIS;
    /* JEXTSEL = 0 (TIM1_TRGO), rising edge */
    ADC1->JSQR = (3u << ADC_JSQR_JL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos) |
                 (1u << ADC_JSQR_JSQ1_Pos) | (2u << ADC_JSQR_JSQ2_Pos) |
                 (3u << ADC_JSQR_JSQ3_Pos) | (4u << ADC_JSQR_JSQ4_Pos);
    ADC2->JSQR = (2u << ADC_JSQR_JL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos) |
                 (17u << ADC_JSQR_JSQ1_Pos) | (13u << ADC_JSQR_JSQ2_Pos) |
                 (3u << ADC_JSQR_JSQ3_Pos);

    ADC1->ISR = ADC_ISR_JEOS;
    ADC1->IER = ADC_IER_JEOSIE;
    ADC2->CR |= ADC_CR_JADSTART;
    ADC1->CR |= ADC_CR_JADSTART;
}

void adc_read(adc_raw_t *r)
{
    r->ia = (uint16_t)ADC1->JDR1;
    r->ib = (uint16_t)ADC1->JDR2;
    r->ic = (uint16_t)ADC1->JDR3;
    r->vbus = (uint16_t)ADC1->JDR4;
    r->thr = (uint16_t)ADC2->JDR1;
    r->tfet = (uint16_t)ADC2->JDR2;
    r->tmot = (uint16_t)ADC2->JDR3;
}

void ADC1_2_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_JEOS) {
        ADC1->ISR = ADC_ISR_JEOS;
        motor_isr();
    }
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t t = micros();
    uint32_t mask = (1u << HALL_A_PIN) | (1u << HALL_B_PIN) | (1u << HALL_C_PIN);
    if (EXTI->PR1 & mask) {
        EXTI->PR1 = mask;
        hall_edge_isr(t);
    }
}

/* ------------------------------------------------------------------------ */
#define TXBUF 2048u
#define RXBUF 256u
static char txb[TXBUF];
static volatile unsigned tx_head, tx_tail;
static char rxb[RXBUF];
static volatile unsigned rx_head, rx_tail;

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    gpio_mode(GPIOB, 6, 2u, 7);
    gpio_mode(GPIOB, 7, 2u, 7);
    gpio_pull(GPIOB, 7, 1u);
    USART1->BRR = (SYSCLK_HZ + 115200u / 2) / 115200u;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART1->CR1 |= USART_CR1_UE;
}

void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;
    if (isr & USART_ISR_ORE)
        USART1->ICR = USART_ICR_ORECF;
    if (isr & USART_ISR_RXNE_RXFNE) {
        char c = (char)USART1->RDR;
        unsigned nh = (rx_head + 1) % RXBUF;
        if (nh != rx_tail) {
            rxb[rx_head] = c;
            rx_head = nh;
        }
    }
    if ((USART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
        if (tx_tail != tx_head) {
            USART1->TDR = (uint8_t)txb[tx_tail];
            tx_tail = (tx_tail + 1) % TXBUF;
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        }
    }
}

unsigned uart_tx_free(void)
{
    return (tx_tail + TXBUF - tx_head - 1) % TXBUF;
}

void uart_write_n(const char *s, unsigned n)
{
    while (n--) {
        unsigned nh = (tx_head + 1) % TXBUF;
        if (nh == tx_tail)
            break;                       /* drop rather than block the main loop */
        txb[tx_head] = *s++;
        tx_head = nh;
    }
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}

void uart_write(const char *s)
{
    unsigned n = 0;
    while (s[n])
        n++;
    uart_write_n(s, n);
}

int uart_getc(void)
{
    if (rx_tail == rx_head)
        return -1;
    char c = rxb[rx_tail];
    rx_tail = (rx_tail + 1) % RXBUF;
    return (unsigned char)c;
}

/* ------------------------------------------------------------------------ */
void SysTick_Handler(void)
{
    ms_ticks++;
}

uint32_t millis(void)
{
    return ms_ticks;
}

void delay_ms(uint32_t ms)
{
    uint32_t t0 = ms_ticks;
    while ((uint32_t)(ms_ticks - t0) < ms)
        ;
}

void iwdg_start(void)
{
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 3u;                      /* LSI 32 kHz / 32 = 1 kHz */
    IWDG->RLR = 200u;                   /* 200 ms */
    while (IWDG->SR)
        ;
    IWDG->KR = 0xAAAAu;
}

void hw_init(void)
{
    clock_init();
    SysTick_Config(SYSCLK_HZ / 1000u);
    NVIC_SetPriorityGrouping(3u);           /* 4 bits preemption */
    NVIC_SetPriority(SysTick_IRQn, 3);
    gpio_init();
    tim2_init();
    uart_init();
    tim1_init();
    adc_init();

    NVIC_SetPriority(EXTI15_10_IRQn, 0);    /* hall timestamps: highest */
    NVIC_SetPriority(ADC1_2_IRQn, 1);       /* control loop             */
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}
