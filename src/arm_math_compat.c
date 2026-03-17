#include "arm_math.h"

#include <math.h>

void arm_cmplx_mag_f32(const float32_t *pSrc, float32_t *pDst, uint32_t numSamples)
{
    for (uint32_t i = 0; i < numSamples; ++i) {
        const float32_t re = pSrc[i * 2U];
        const float32_t im = pSrc[i * 2U + 1U];
        pDst[i] = sqrtf((re * re) + (im * im));
    }
}

arm_status arm_atan2_f32(float32_t y, float32_t x, float32_t *result)
{
    if (result == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    *result = atan2f(y, x);
    return ARM_MATH_SUCCESS;
}
