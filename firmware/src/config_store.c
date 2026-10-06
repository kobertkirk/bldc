/* Config persistence in the last flash page (2 KB, page 63). */
#include <stddef.h>
#include <string.h>

#include "config.h"
#include "stm32g4xx.h"

#define CFG_PAGE      63u
#define CFG_ADDR      (0x08000000u + CFG_PAGE * 2048u)

static uint32_t crc32(const void *data, unsigned len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

int config_load(void)
{
    const config_t *f = (const config_t *)CFG_ADDR;
    if (f->magic == CONFIG_MAGIC && f->version == CONFIG_VERSION &&
        f->crc == crc32(f, offsetof(config_t, crc))) {
        memcpy(&cfg, f, sizeof(cfg));
        return 1;
    }
    config_defaults(&cfg);
    return 0;
}

static int flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    uint32_t err = FLASH->SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                                FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR |
                                FLASH_SR_FASTERR | FLASH_SR_OPERR);
    FLASH->SR = err | FLASH_SR_EOP;
    return err ? -1 : 0;
}

int config_save(void)
{
    static uint64_t buf[(sizeof(config_t) + 7) / 8];
    cfg.magic = CONFIG_MAGIC;
    cfg.version = CONFIG_VERSION;
    cfg.crc = crc32(&cfg, offsetof(config_t, crc));
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &cfg, sizeof(cfg));

    int rc = 0;
    __disable_irq();
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = 0xFFFFFFFFu;                       /* clear stale flags */
    flash_wait();
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB) | FLASH_CR_PER | (CFG_PAGE << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    rc |= flash_wait();
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);

    FLASH->CR |= FLASH_CR_PG;
    volatile uint32_t *dst = (volatile uint32_t *)CFG_ADDR;
    for (unsigned i = 0; i < sizeof(buf) / 8 && !rc; i++) {
        const uint32_t *w = (const uint32_t *)&buf[i];
        dst[2 * i] = w[0];
        __ISB();
        dst[2 * i + 1] = w[1];
        rc |= flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
    /* flush caches so the CPU sees the new page */
    FLASH->ACR &= ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    FLASH->ACR |= FLASH_ACR_DCRST | FLASH_ACR_ICRST;
    FLASH->ACR &= ~(FLASH_ACR_DCRST | FLASH_ACR_ICRST);
    FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN;
    __enable_irq();

    if (!rc && memcmp((const void *)CFG_ADDR, &cfg, sizeof(cfg)) != 0)
        rc = -1;
    return rc;
}
