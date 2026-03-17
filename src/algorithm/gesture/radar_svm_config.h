#ifndef RADAR_SVM_CONFIG_INCLUDE
#define RADAR_SVM_CONFIG_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar_struct.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)
/************************ 雷达数据处理相关常量 ************************/
/* 原始特征个数 */
#define FEAR_ORI_NUM                            (4)

/* 处理后的特征个数 */
#define FEAR_EXP_NUM                            (23)

/* 最大FIFO长度(与FIFO_LEN关联) */
#define RADAR_MAX_FIFOLEN                       (40)

/* 特征窗口长度(默认值, 与NvldMax关联) */
#define FEAR_WIN_LEN                            (30)

#define RADAR_MAX_LEN                           (RADAR_MAX_FIFOLEN)

/* 无效值(用于标记无效数据点) */
#define FEAR_INV_DATA                           (-1e-3f)

#define FEAR_FLT_MAX_N0                         (15)
#define FEAR_FLT_MAX_LEN                        FEAR_WIN_LEN

/* 数据筛选。0: SNRmax; 1: rangeMin */
#define FEAR_SEL_TYPE                           (0)

/* 浮点数比较容差 */
#define EPS                                     (0.000001f)
/**********************************************************************/

/*********************** 原始特征提取参数默认值 ***********************/
/* 目标筛选距离门限(与rThTar关联) */
#define FEAR_TAR_RTH                            (0.5f)  /* 筛选距离保护 */

/* 目标筛选速度门限(与vThTar关联) */
#define FEAR_TAR_VTH_MAX                        (4.0f)
#define FEAR_TAR_VTH_MIN                        (0.151f)

/* 目标筛选角度门限(与aThTar关联) */
#define FEAR_TAR_ATH                            (90.0f)
/**********************************************************************/

/************ 默认块检测参数。与getRvapBlock函数的参数对应 ************/
/* 默认块最小有效点数(与NvldMin关联) */
#define BLOCK_NVLD_MIN                          (6-1)

/* 默认块最大有效点数(与NvldMax关联) */
#define BLOCK_NVLD_MAX                          FEAR_WIN_LEN

/* 默认块最大无效点数(与NinvldMax关联) */
#define BLOCK_NINVLD_MAX                        (2)

/* 默认块最小间隔(与Ngap关联) */
#define BLOCK_MIN_NGAP                          (2)

/* 默认块内中值滤波点数 */
#define BLOCK_FLT_N0                            (4)

/* 默认手势识别结果门禁 */
#define GR_SNR_VLD_TH                           (38.0f)

#define GR_V0_SEL_MIN                           (1)
/**********************************************************************/

/************************** 手势识别相关参数 **************************/
/* 默认手势识别距离门限 */
#define GR_RTH_IN                               (0.3f)

/* 默认手势识别距离门限保护值 */
#define GR_RTH_ADD                              (0.05f+0.015f) /* 对应取最小距离值 */
#define GR_RTH_FIFO_ADD                         (0.20f-0.00f)  /* 对应取最小距离值 */

/* 默认手势识别角度门限保护值 */
#define GR_ATH_RANGE                            (40.0f)

#define GR_ATH_RANGE_MIN                        (50.0f-10.0f)
#define GR_ATH_RANGE_MAX                        (70.0f-10.0f)
#define GR_SWAP_DIS_RANGE                       (0.35f)
#define GR_SWAP_DIS_RANGE_HALF                  (GR_SWAP_DIS_RANGE*0.5f)

/* 默认手势识别角度最小值门限 */
#define GR_ATH_NEG_MAX                          (-0.0f)

/* 默认手势识别角度最大值门限 */
#define GR_ATH_POS_MIN                          (0.0f)

/* 默认手势识别信噪比比例特征门限 */
#define GR_SNR_FEA_TH                           (18.0f)

/* 默认手势识别有效点数门限保护值 */
#define GR_VLD_BLOCK_PTS                        (BLOCK_NVLD_MIN+BLOCK_MIN_NGAP)

/* 手势识别最小距离门限: 暂定 */
#define GR_RTH_IN_MIN                           (0.05f)

/* 手势识别最大距离门限: 暂定 */
#define GR_RTH_IN_MAX                           (0.3f)

/* 手势类别个数 */
#define GR_CLASS_NUM                            (3)

/* 手势识别: 右挥手(从左往右) */
#define GR_HAND_RIGHT                           (-1)

/* 手势识别: 左挥手(从右往左) */
#define GR_HAND_LEFT                            (1)

/* 手势识别: 无效手势 */
#define GR_HAND_INVLD                           (0)
/**********************************************************************/
#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)
/************************ 雷达数据处理相关常量 (未改变) ************************/
/* 原始特征个数 */
#define FEAR_ORI_NUM                            (4)

/* 处理后的特征个数 */
#define FEAR_EXP_NUM                            (23)

/* 最大FIFO长度 */
#define RADAR_MAX_FIFOLEN                       (40)

/* 特征窗口长度 */
#define FEAR_WIN_LEN                            (30)

#define RADAR_MAX_LEN                           (RADAR_MAX_FIFOLEN)

#define FEAR_FLT_MAX_N0                         (15)
#define FEAR_FLT_MAX_LEN                        FEAR_WIN_LEN

/* 数据筛选。0: SNRmax; 1: rangeMin */
#define FEAR_SEL_TYPE                           (0)

/************************ 定点化转换常量 (Q8格式) ************************/
/* 无效值标记 (原 -1e-3f, Q8下取0或-1) 
   代码中通常判断 p > EPS_Q8，故此处设为0 */
#define FEAR_INV_DATA_Q8                        (0)

/* 定点数比较容差 (1/256 ≈ 0.0039) */
#define EPS_Q8                                  (1)

/*********************** 原始特征提取参数 (Q8格式转换) ***********************/
/* 目标筛选距离门限 (原 0.5f * 256 = 128) */
#define FEAR_TAR_RTH_Q8                         (128)

/* 目标筛选速度门限上限 (原 4.0f * 256 = 1024) */
#define FEAR_TAR_VTH_MAX_Q8                     (1024)

/* 目标筛选速度门限下限 (原 0.151f * 256 = 38.6 -> 39) */
#define FEAR_TAR_VTH_MIN_Q8                     (39)

/* 目标筛选角度门限 (原 90.0f * 256 = 23040) */
#define FEAR_TAR_ATH_Q8                         (23040)

/************ 块检测参数 (整数部分未变，浮点部分已转换) ************/
/* 默认块最小有效点数 */
#define BLOCK_NVLD_MIN                          (5)  /* 6-1 */

/* 默认块最大有效点数 */
#define BLOCK_NVLD_MAX                          FEAR_WIN_LEN

/* 默认块最大无效点数 */
#define BLOCK_NINVLD_MAX                        (2)

/* 默认块最小间隔 */
#define BLOCK_MIN_NGAP                          (2)

/* 默认块内中值滤波点数 */
#define BLOCK_FLT_N0                            (4)

/* 默认手势识别结果门禁 SNR (原 38.0f * 256 = 9728) */
#define GR_SNR_VLD_TH_Q8                        (9728)

#define GR_V0_SEL_MIN                           (1)

/************************** 手势识别相关参数 (Q8格式转换) **************************/
/* 默认手势识别距离门限 (原 0.3f * 256 = 77) */
#define GR_RTH_IN_Q8                            (77)

/* 默认手势识别距离门限保护值 (原 0.065f * 256 = 16.6 -> 17) */
#define GR_RTH_ADD_Q8                           (17)

/* 默认手势识别距离门限保护值FIFO (原 0.20f * 256 = 51) */
#define GR_RTH_FIFO_ADD_Q8                      (51)

/* 默认手势识别角度极差 (原 40.0f * 256 = 10240) */
#define GR_ATH_RANGE_Q8                         (10240)

/* 角度门限最小值 (原 40.0f * 256 = 10240) */
#define GR_ATH_RANGE_MIN_Q8                     (10240)

/* 角度门限最大值 (原 60.0f * 256 = 15360) */
#define GR_ATH_RANGE_MAX_Q8                     (15360)

/* 角度滑动距离窗口一半 (原 0.35f * 0.5 * 256 = 44.8 -> 45) */
#define GR_SWAP_DIS_RANGE_HALF_Q8               (45)

/* 默认手势识别角度最小值/最大值门限 (保持0) */
#define GR_ATH_NEG_MAX_Q8                       (0)
#define GR_ATH_POS_MIN_Q8                       (0)

/* 默认手势识别信噪比比例特征门限 (原 18.0f * 256 = 4608) */
#define GR_SNR_FEA_TH_Q8                        (4608)

/* 默认手势识别有效点数门限保护值 (整数未变) */
#define GR_VLD_BLOCK_PTS                        (BLOCK_NVLD_MIN + BLOCK_MIN_NGAP)

/* 手势识别最小距离门限 (原 0.05f * 256 = 13) */
#define GR_RTH_IN_MIN_Q8                        (13)

/* 手势识别最大距离门限 (原 0.3f * 256 = 77) */
#define GR_RTH_IN_MAX_Q8                        (77)

/* 手势类别与方向定义 (整数未变) */
#define GR_CLASS_NUM                            (3)
#define GR_HAND_RIGHT                           (-1)
#define GR_HAND_LEFT                            (1)
#define GR_HAND_INVLD                           (0)
#endif

#if defined(__cplusplus)
}
#endif

#endif
