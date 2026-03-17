#ifndef RADAR_SVM_PRIVATE_INCLUDE
#define RADAR_SVM_PRIVATE_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar_struct.h"
#include "radar_svm_config.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)
/* 算法操作成功 */
#define RADAR_ALG_SUCCESS                           (0)

/* 算法操作失败 */
#define RADAR_ALG_FAIL                              (-1)

/* 空指针错误 */
#define ALG_NULL_PTR                                (-2)

/* 无效长度错误 */
#define ALG_INVALID_LEN                             (-3)

/* 雷达原始数据结构体 */
typedef struct {
    float r;                                        /* 距离 */
    float v;                                        /* 速度 */
    float a;                                        /* 角度 */
    float p;                                        /* snr */
} RadarPoints;

/* 时间序列。输出的时间序列: 用于保存若干帧的R/V/A/P序列 */
typedef struct {
    int   n;                                        /* 事件序列点数 */
    float a[RADAR_MAX_LEN];                         /* 角度 */
    float r[RADAR_MAX_LEN];                         /* 距离 */
    float v[RADAR_MAX_LEN];                         /* 速度 */
    float p[RADAR_MAX_LEN];                         /* SNR */
} TimeSeries;

/* 算法参数结构体 */
typedef struct {
    int   type;                                     /* 算法类型:0 base;1:anti */
    float rTh;                                      /* 识别距离门限 */
    float angleRangeTh;                             /* 角度极差门限 */
    float snrTh;                                    /* SNR门限 */
    float vldPtNumMin;                              /* 最小有效点个数 */
    float feaWinLen;                                /* 特征窗口长度 */
    float blockInvldMax;                            /* 特征块最大无效点数 */
    float blockMinNgap;                             /* 特征块最小间隔 */
    float blockFltN0;                               /* 特征块滤波器窗口 */
    float rThSelMin;                                /* 距离选择门限下限 */
    float rThSelMax;                                /* 距离选择门限上限 */ 
    float v0SelMin;                                 /* 速度0点选择门限下限 */ 
} GrParams;
#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)
/************************ 算法状态码 (未改变) ************************/
/* 算法操作成功 */
#define RADAR_ALG_SUCCESS                           (0)

/* 算法操作失败 */
#define RADAR_ALG_FAIL                              (-1)

/* 空指针错误 */
#define ALG_NULL_PTR                                (-2)

/* 无效长度错误 */
#define ALG_INVALID_LEN                             (-3)

/************************ 数据结构体定点化转换 ************************/

/**
 * 雷达点特征结构体 (定点化)
 * 所有物理量均采用 Q8 格式 (物理值 * 256)
 */
typedef struct {
    int16_t r;                                      /* 距离 (Q8) */
    int16_t v;                                      /* 速度 (Q8) */
    int16_t a;                                      /* 角度 (Q8) */
    int16_t p;                                      /* SNR  (Q8) */
} RadarPoints;

/**
 * 时间序列结构体 (定点化)
 * 用于保存若干帧的特征序列
 */
typedef struct {
    int16_t n;                                      /* 有效序列点数 */
    int16_t a[RADAR_MAX_LEN];                       /* 角度序列 (Q8) */
    int16_t r[RADAR_MAX_LEN];                       /* 距离序列 (Q8) */
    int16_t v[RADAR_MAX_LEN];                       /* 速度序列 (Q8) */
    int16_t p[RADAR_MAX_LEN];                       /* SNR序列  (Q8) */
} TimeSeries;

/**
 * 手势识别算法参数结构体 (定点化)
 */
typedef struct {
    int16_t type;                                   /* 算法类型: 0 base; 1: anti */
    int16_t rTh;                                    /* 识别距离门限 (Q8) */
    int16_t angleRangeTh;                           /* 角度极差门限 (Q8) */
    int16_t snrTh;                                  /* SNR门限 (Q8) */
    int16_t vldPtNumMin;                            /* 最小有效点个数 (整数) */
    int16_t feaWinLen;                              /* 特征窗口长度 (整数) */
    int16_t blockInvldMax;                          /* 特征块最大无效点数 (整数) */
    int16_t blockMinNgap;                           /* 特征块最小间隔 (整数) */
    int16_t blockFltN0;                             /* 特征块滤波器窗口 (整数) */
    int16_t rThSelMin;                              /* 距离选择门限下限 (Q8) */
    int16_t rThSelMax;                              /* 距离选择门限上限 (Q8) */ 
    int16_t v0SelMin;                               /* 速度0点选择门限下限 (整数/Q8) */ 
} GrParams;
#endif
#if defined(__cplusplus)
}
#endif

#endif
