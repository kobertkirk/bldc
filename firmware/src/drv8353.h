/*
 * TI DRV8353RS smart gate driver: SPI configuration and fault readout.
 *
 * SPI frames are 16 bit, mode 1 (CPOL 0, CPHA 1): bit 15 = 1 for a read,
 * bits 14..11 = register address, bits 10..0 = data.
 */
#ifndef DRV8353_H
#define DRV8353_H

#include <stdint.h>

#define DRV_REG_FAULT1   0x0u      /* fault status 1 (read only)             */
#define DRV_REG_FAULT2   0x1u      /* VGS / CSA status 2 (read only)         */
#define DRV_REG_CTRL     0x2u      /* driver control                         */
#define DRV_REG_GATE_HS  0x3u      /* high-side IDRIVE + register lock       */
#define DRV_REG_GATE_LS  0x4u      /* low-side IDRIVE, TDRIVE, CBC           */
#define DRV_REG_OCP      0x5u      /* VDS overcurrent, dead time             */
#define DRV_REG_CSA      0x6u      /* current-sense amplifiers               */

/* driver control: 6x PWM mode, report over-temperature warnings */
#define DRV_CTRL_VAL     ((1u << 7) /* OTW_REP */)
#define DRV_CTRL_CLR_FLT (1u << 0)
/* gate drive: 300 mA source / 600 mA sink on both sides (~100 ns edges on the
   IPT015N10N5 Miller plateau), TDRIVE 1000 ns, cycle-by-cycle OCP clear */
#define DRV_LOCK_UNLOCK  (3u << 8)
#define DRV_LOCK_LOCK    (6u << 8)
#define DRV_IDRIVE       ((4u << 4) | 4u)
#define DRV_GATE_LS_VAL  ((1u << 10) | (1u << 8) | DRV_IDRIVE)
/* VDS overcurrent: 0.3 V (~110 A at 125 C Rds(on), a short-circuit backstop
   behind the 80 A firmware trip), 4 us deglitch, latched; 100 ns dead time
   inside the driver on top of the MCU's 400 ns */
#define DRV_OCP_VAL      ((1u << 8) | (0u << 6) | (2u << 4) | 0x6u)
/* CSA: bidirectional around VREF/2, gain 20, sense SPx-SNx */
#define DRV_CSA_GAIN20   (2u << 6)
#define DRV_CSA_VAL      ((1u << 9) | DRV_CSA_GAIN20)

int drv_init(void);                 /* 0 = configured and read back OK */
uint16_t drv_read(unsigned addr);
void drv_write(unsigned addr, uint16_t data);
int drv_fault_active(void);         /* nFAULT pin low */
void drv_clear_faults(void);
void drv_sleep(void);

#endif
