#ifndef PC_ARM_MATH_INCLUDE
#define PC_ARM_MATH_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef float float32_t;

typedef enum {
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1
} arm_status;

void arm_cmplx_mag_f32(const float32_t *pSrc, float32_t *pDst, uint32_t numSamples);
arm_status arm_atan2_f32(float32_t y, float32_t x, float32_t *result);

#ifdef __cplusplus
}
#endif

#endif
