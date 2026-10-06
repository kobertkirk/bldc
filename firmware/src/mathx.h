/* Small fast math helpers for the control loop. */
#ifndef MATHX_H
#define MATHX_H

#define PI_F        3.14159265f
#define TWO_PI      6.28318531f
#define DEG         (PI_F / 180.0f)
#define SQRT3       1.73205081f
#define INV_SQRT3   0.57735027f

static inline float wrap_2pi(float a)
{
    while (a >= TWO_PI)
        a -= TWO_PI;
    while (a < 0.0f)
        a += TWO_PI;
    return a;
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void sincos_f(float a, float *s, float *c);   /* any angle, ~1e-5 error */

#endif
