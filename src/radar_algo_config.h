#ifndef RADAR_ALGO_CONFIG_INCLUDE
#define RADAR_ALGO_CONFIG_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

/******************** 数据类型定义 ********************/
#define CONFIG_ALG_DINT                     (0)
#define CONFIG_ALG_DFLOAT                   (1)
#define CONFIG_ALG_DTYPE                    CONFIG_ALG_DFLOAT
//#define CONFIG_TEST_ALG_INT //用于测试定点数与浮点数的结果精度差，数据源自浮点数运算
/******************************************************/

/********************** 算法配置 **********************/
/* 使能手势识别 */
#define CONFIG_GESTURE_ENABLE

/* 是否打印算法方法耗时时间 */
#define CONFIG_PRINT_ALGO_CAST
/******************************************************/

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)
/******************** 雷达配置参数 ********************/
/* PI的定义 */
#if !defined(M_PI)
#define M_PI                                (3.14159265358979323846)
#endif

/* 最大目标数 */
#define CONFIG_OBJECT_MAX                   (32)

/* 使用的距离bin数 */
#define RANGE_BINS                          (20)

/* 雷达工作波长(m) */
#define RADAR_LAMDA                         (0.0125f)

/* 帧时间间隔(s) */
#define RADAR_T_FRAME                       (0.05f)

/* 工作带宽(MHz) */
#define RADAR_BANDWIDTH                     (1650)

/* 距离分辨率(m) */
#define RADAR_RANGE_RESOLUTION              (150.0f/RADAR_BANDWIDTH)

/* 速度分辨率(m/s) */
#define RADAR_VELOCITY_RESOLUTION           (RADAR_LAMDA / (2 * RADAR_T_FRAME))
/******************************************************/

/******************** CFAR配置参数 ********************/
/* CFAR阈值检测阈值，低于噪声这么多倍的点不检测 */
#define CONFIG_CFAR_TH_OFFSET               (10.0f)

/* CFAR幅度阈值 */
#define CONFIG_CFAR_TH_AMP                  (5.0f)

/* CFAR保护单元大小 - 速度维度 */
#define CONFIG_CFAR_NUM_GUARD_VEL           (2)

/* CFAR保护单元大小 - 距离维度 */
#define CONFIG_CFAR_NUM_GUARD_RANGE         (1)

/* CFAR训练单元大小 - 速度维度 */
#define CONFIG_CFAR_NUM_TRAIN_VEL           (3)

/* CFAR训练单元大小 - 距离维度 */
#define CONFIG_CFAR_NUM_TRAIN_RANGE         (2)

/* CFAR距离维度起始bin索引 */
#define CONFIG_CFAR_RBIN_START_IDX          (1)

/* CFAR类型：0 classiy; 1:cut */
#define CONFIG_CFAR_TYPE                    (0)

/* CFAR底噪平均系数*/
#define CONFIG_CFAR_NCOEF                   (0.02)
/******************************************************/
#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)
/******************** 雷达配置参数 (宏定义保持原样) ********************/
#if !defined(M_PI)
#define M_PI                                (3.14159265358979323846)
#endif
#define CONFIG_OBJECT_MAX                   (32)
#define RANGE_BINS                          (20)
#define RADAR_LAMDA                         (0.0125f)
#define RADAR_T_FRAME                       (0.05f)
#define RADAR_BANDWIDTH                     (1650)
/* 雷达分辨率换算的 Q16 定点乘数 */
/* 距离分辨率 = 150/1650 = 0.090909; 转为Q16: 0.090909 * 65536 = 5958 */
#define RADAR_RANGE_RES_Q16         (5958)
#define RADAR_VELOCITY_RESOLUTION           (RADAR_LAMDA / (2 * RADAR_T_FRAME))

/******************** CFAR配置参数 (针对M0转为定点数) ********************/
/* CFAR阈值检测偏移量，使用 Q4 定点数表示: 10.0 -> 160 */
#define CONFIG_CFAR_TH_OFFSET_Q4            (160)

/* CFAR幅度阈值，使用 Q4 定点数表示: 5.0 -> 80 */
#define CONFIG_CFAR_TH_AMP_Q4               (80)

#define CONFIG_CFAR_NUM_GUARD_VEL           (2)
#define CONFIG_CFAR_NUM_GUARD_RANGE         (1)
#define CONFIG_CFAR_NUM_TRAIN_VEL           (3)
#define CONFIG_CFAR_NUM_TRAIN_RANGE         (2)
#define CONFIG_CFAR_RBIN_START_IDX          (1)
#define CONFIG_CFAR_TYPE                    (0)
/* CFAR底噪平均系数，采用 Q15 定点数表示: 0.02 * 32768 ≈ 655 */
#define CONFIG_CFAR_NCOEF_Q15               (655)
#endif

#if defined(__cplusplus)
}
#endif

#endif
