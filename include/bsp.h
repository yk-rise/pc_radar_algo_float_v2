#ifndef PC_BSP_INCLUDE
#define PC_BSP_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "arm_math.h"

#ifndef __WEAK
#if defined(_MSC_VER)
#define __WEAK
#else
#define __WEAK __attribute__((weak))
#endif
#endif

static inline uint32_t bsp_ticks(void)
{
    return (uint32_t)clock();
}

static inline uint32_t bsp_time_cast(uint32_t te, uint32_t ts)
{
    if (te < ts) {
        return 0;
    }
    return (uint32_t)(((double)(te - ts) * 1000.0) / (double)CLOCKS_PER_SEC);
}

static inline void pr_info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
}

#ifdef __cplusplus
}
#endif

#endif
