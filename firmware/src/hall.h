#ifndef HALL_H
#define HALL_H

#include <stdint.h>

void hall_init(void);                 /* (re)build tables from cfg.hall_angle */
void hall_edge_isr(uint32_t t_us);    /* EXTI, highest priority               */

/* Rotor electrical angle [rad, 0..2pi) and speed [rad/s electrical]. */
void hall_get(uint32_t now_us, float *theta, float *omega);
uint32_t hall_state(void);
uint32_t hall_edge_count(void);       /* increments on every state change     */
uint32_t hall_ms_since_edge(uint32_t now_us);

/* learning (used by detect, motor must be under forced-angle control) */
void hall_learn_reset(void);
void hall_learn_sample(float theta, uint32_t state);
int hall_learn_finish(char *msg, unsigned msglen);   /* 0 = ok */

#endif
