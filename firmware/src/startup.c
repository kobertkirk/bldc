/* Minimal Cortex-M4F startup for STM32G431 (vector table + reset handler). */
#include <stdint.h>

#include "stm32g4xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);
extern void __libc_init_array(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK;
void HardFault_Handler(void) WEAK;
void MemManage_Handler(void) WEAK;
void BusFault_Handler(void) WEAK;
void UsageFault_Handler(void) WEAK;
void SVC_Handler(void) WEAK;
void DebugMon_Handler(void) WEAK;
void PendSV_Handler(void) WEAK;
void SysTick_Handler(void) WEAK;
void ADC1_2_IRQHandler(void) WEAK;
void EXTI15_10_IRQHandler(void) WEAK;
void USART1_IRQHandler(void) WEAK;

/* 102 device interrupts on STM32G431; unused ones go to Default_Handler */
#define N_IRQ 102
typedef void (*vec_t)(void);

uint32_t SystemCoreClock = 16000000u;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used))
const vec_t vector_table[16 + N_IRQ] = {
    (vec_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    [16 ... 16 + N_IRQ - 1] = Default_Handler,
    [16 + ADC1_2_IRQn] = ADC1_2_IRQHandler,
    [16 + EXTI15_10_IRQn] = EXTI15_10_IRQHandler,
    [16 + USART1_IRQn] = USART1_IRQHandler,
};
#pragma GCC diagnostic pop

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;)
        *dst++ = 0;
    SCB->CPACR |= (0xFu << 20);           /* enable FPU (CP10/CP11) */
    __DSB();
    __ISB();
    SCB->VTOR = (uint32_t)vector_table;
    __libc_init_array();
    main();
    for (;;)
        ;
}

/* Any unexpected interrupt / fault: gates off, then let the watchdog reset us. */
void Default_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    for (;;)
        ;
}

/* newlib stubs (no OS, no file I/O) */
struct stat;
int _close(int f) { (void)f; return -1; }
int _fstat(int f, struct stat *st) { (void)f; (void)st; return -1; }
int _getpid(void) { return 1; }
int _isatty(int f) { (void)f; return 0; }
int _kill(int p, int s) { (void)p; (void)s; return -1; }
int _lseek(int f, int o, int w) { (void)f; (void)o; (void)w; return -1; }
int _read(int f, char *b, int n) { (void)f; (void)b; (void)n; return -1; }
int _write(int f, const char *b, int n) { (void)f; (void)b; return n; }
void _exit(int c) { (void)c; for (;;) ; }
void *_sbrk(int incr)
{
    extern char end;
    static char *brk = &end;
    char *prev = brk;
    uintptr_t limit = (uintptr_t)&_estack - 0x800u;   /* keep 2 KB for the stack */
    if ((uintptr_t)(brk + incr) > limit)
        return (void *)-1;
    brk += incr;
    return prev;
}
