#include "mathx.h"

#include <math.h>

#define LUT_BITS 9
#define LUT_N    (1 << LUT_BITS)

static float lut[LUT_N + 1];
static int lut_ready;

static void lut_init(void)
{
    for (int i = 0; i <= LUT_N; i++)
        lut[i] = sinf(TWO_PI * (float)i / LUT_N);
    lut_ready = 1;
}

static inline float lut_sin(float x)            /* x in turns, 0..1 */
{
    float f = x * LUT_N;
    int i = (int)f;
    float fr = f - (float)i;
    i &= LUT_N - 1;
    return lut[i] + (lut[i + 1] - lut[i]) * fr;
}

void sincos_f(float a, float *s, float *c)
{
    if (!lut_ready)
        lut_init();
    float t = a * (1.0f / TWO_PI);
    t -= floorf(t);
    *s = lut_sin(t);
    float tc = t + 0.25f;
    if (tc >= 1.0f)
        tc -= 1.0f;
    *c = lut_sin(tc);
}
