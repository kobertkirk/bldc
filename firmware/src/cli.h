#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void cli_init(void);
void cli_poll(uint32_t now_ms);            /* call from main loop */
void cli_printf(const char *fmt, ...);
extern volatile int cli_shutdown_req;

#endif
