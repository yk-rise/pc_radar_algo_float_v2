#ifndef RADAR_STRUCT_INCLUDE
#define RADAR_STRUCT_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "bsp.h"
#include "radar_algo_config.h"
#include "cfg_radar_profile.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)
/* 测角模块输出的结构体: 只包含最终物理量 */
typedef struct {
    float range;                    /* 距离(m) */
    float angle;                    /* 速度(m/s) */
    float velocity;                 /* 角度(°) */
    float snr;                      /* 信噪比(dB) */
    float powerdb;                  /* 幅度功率(dB) */
    float amplitude;                /* 幅度值 */
} TargetPointType;

/* 定义CFAR参数结构体 */
typedef struct {
    int   range_dim;                /* 距离维大小 */
    int   velocity_dim;             /* 速度维大小 */
    int   guard_cell[2];            /* 速度维单侧保护单元数, 距离维单侧保护单元数 */
    int   reference_cell[2];        /* 速度维单侧参考单元数，距离维单侧参考单元数 */
    float threshold_factors;        /* 线性检测门限参数(倍数) */
} CfarConfigType;

/* 定义检测结果结构体 */
typedef struct {
    int   range_index;              /* 距离维粗糙索引(0-based) */
    int   velocity_index;           /* 速度维粗糙索引(0-based) */
    float snr;                      /* 信噪比(dB) */
    float noise;                    /* 噪声估计值 */
    float amplitude;                /* 检测点的幅度 */
    float range_fine;               /* 距离维精细值(差值之后的结果) */
    float velocity_fine;            /* 速度维精细值(差值之后的结果) */
} DetectResultType;

#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)
/* 定义输出结果的定点结构体（全整数）*/
typedef struct {
    int16_t range;                    /* 距离(m)      (Q8格式: 物理值 = range / 256.0) */
    int16_t angle;                    /* 角度(°)      (Q8格式: 物理值 = angle / 256.0) */
    int16_t velocity;                 /* 速度(m/s)    (Q8格式: 物理值 = velocity / 256.0) */
    int16_t snr;                      /* 信噪比(dB)   (Q8格式: 直接承接 CFAR 结果) */
    int16_t powerdb;                  /* 幅度功率(dB) (Q8格式: 物理值 = powerdb / 256.0) */
    uint32_t amplitude;               /* 幅度绝对值    (直接承接 CFAR 结果) (必须改为 uint32_t，防止 Q8 溢出) */
} TargetPointType;

/* 定义CFAR参数结构体 */
typedef struct {
    int16_t range_dim;                /* 距离维大小 */
    int16_t velocity_dim;             /* 速度维大小 */
    int16_t guard_cell[2];            /* 速度维单侧保护单元数, 距离维单侧保护单元数 */
    int16_t reference_cell[2];        /* 速度维单侧参考单元数，距离维单侧参考单元数 */
    uint16_t threshold_factors;       /* 线性检测门限参数(使用 Q4 定点数，如 10.0f -> 160) */
} CfarConfigType;

/* 定义定点检测结果结构体 */
typedef struct {
    int16_t range_index;        /* 距离维粗糙索引(0-based) */
    int16_t velocity_index;     /* 速度维粗糙索引(0-based) */
    int16_t snr;                /* 信噪比(dB整数值) (Q8定点数：高8位整数，低8位小数。实际值 = range_fine/256.0)*/
    uint32_t noise;             /* 噪声估计值 (Q8格式: 物理值 = noise / 256.0)*/
    uint32_t amplitude;         /* 检测点的幅度 (Q8格式: 物理值 = amplitude / 256.0)*/
    int16_t range_fine;         /* 距离维精细值 (Q8定点数：高8位整数，低8位小数。实际值 = range_fine/256.0) */
    int16_t velocity_fine;      /* 速度维精细值 (Q8定点数：实际值 = velocity_fine/256.0) */
} DetectResultType;
#endif

#include "radar_dbf.h"
#include "radar_cfar.h"

#if defined(__cplusplus)
}
#endif

#endif
