/*
 * DRV8353RS over SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI, AF5; PA15 nSCS as GPIO).
 */
#include "drv8353.h"

#include "hw.h"

static void cs(int level)
{
    DRV_CS_PORT->BSRR = level ? (1u << DRV_CS_PIN) : (1u << (DRV_CS_PIN + 16));
}

static uint16_t xfer(uint16_t w)
{
    while (!(SPI1->SR & SPI_SR_TXE))
        ;
    *(volatile uint16_t *)&SPI1->DR = w;
    while (!(SPI1->SR & SPI_SR_RXNE))
        ;
    uint16_t r = *(volatile uint16_t *)&SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY)
        ;
    return r;
}

static uint16_t frame(uint16_t w)
{
    cs(0);
    for (volatile int i = 0; i < 20; i++)          /* t_SU_nSCS 50 ns */
        ;
    uint16_t r = xfer(w);
    cs(1);
    for (volatile int i = 0; i < 80; i++)          /* t_HI_nSCS 400 ns */
        ;
    return r;
}

uint16_t drv_read(unsigned addr)
{
    return frame((uint16_t)((1u << 15) | ((addr & 0xFu) << 11))) & 0x7FFu;
}

void drv_write(unsigned addr, uint16_t data)
{
    (void)frame((uint16_t)(((addr & 0xFu) << 11) | (data & 0x7FFu)));
}

int drv_fault_active(void)
{
    return !(DRV_FLT_PORT->IDR & (1u << DRV_FLT_PIN));
}

void drv_clear_faults(void)
{
    drv_write(DRV_REG_CTRL, DRV_CTRL_VAL | DRV_CTRL_CLR_FLT);
}

void drv_sleep(void)
{
    DRV_EN_PORT->BSRR = 1u << (DRV_EN_PIN + 16);
}

int drv_init(void)
{
    /* SPI1 on APB2 (170 MHz) / 128 = 1.33 MHz, master, mode 1, 16 bit */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;
    SPI1->CR1 = 0;
    SPI1->CR2 = (15u << SPI_CR2_DS_Pos);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (6u << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;

    DRV_EN_PORT->BSRR = 1u << DRV_EN_PIN;           /* wake: t_WAKE <= 1 ms */
    delay_ms(2);

    drv_write(DRV_REG_GATE_HS, DRV_LOCK_UNLOCK | DRV_IDRIVE);
    drv_write(DRV_REG_CTRL, DRV_CTRL_VAL | DRV_CTRL_CLR_FLT);
    drv_write(DRV_REG_GATE_LS, DRV_GATE_LS_VAL);
    drv_write(DRV_REG_OCP, DRV_OCP_VAL);
    drv_write(DRV_REG_CSA, DRV_CSA_VAL);

    int ok = (drv_read(DRV_REG_CTRL) & ~DRV_CTRL_CLR_FLT) == DRV_CTRL_VAL &&
             drv_read(DRV_REG_GATE_LS) == DRV_GATE_LS_VAL &&
             drv_read(DRV_REG_OCP) == DRV_OCP_VAL &&
             drv_read(DRV_REG_CSA) == DRV_CSA_VAL;
    drv_write(DRV_REG_GATE_HS, DRV_LOCK_LOCK | DRV_IDRIVE);    /* lock the settings */
    return ok ? 0 : -1;
}
