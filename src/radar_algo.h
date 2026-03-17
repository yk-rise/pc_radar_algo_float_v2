#ifndef RADAR_ALGORITHM_INCLUDE
#define RADAR_ALGORITHM_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar_struct.h"

#define POINT_TYPE_DBSCAN           (0)     /* 聚类信息 */
#define POINT_TYPE_CLUSTER          (1)     /* 点云信息 */
#define POINT_TYPE_TRACKER          (2)     /* 跟踪信息 */

/**
 * 函数名称: radar_algorithm_probe
 * 功能描述: 雷达算法注册
 * 输入参数: 无
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algorithm_probe(void);

/**
 * 函数名称: radar_algorithm_manager
 * 功能描述: 雷达算法管理(已经在循环中调用了，这里不可死循环)
 * 输入参数: 无
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algorithm_manager(void);

/**
 * 函数名称: radar_algo_gesture_hook
 * 功能描述: 手势分类数据回调
 * 输入参数: classify -- -1: 右挥手手势(从左往右)；0: 无效手势；1: 左挥手手势(从右往左)
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algo_gesture_hook(const int classify);

/**
 * 函数名称: radar_algo_target_hook
 * 功能描述: 雷达算法输出目标信息回调
 * 输入参数: dtype    -- 信息类型(POINT_TYPE_DBSCAN、POINT_TYPE_CLUSTER、POINT_TYPE_TRACKER)
 *           count    -- 目标数
 *           clusters -- 目标信息结果
 *           scr_rng  -- 亮屏距离阈值(cm)
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algo_target_hook(const uint8_t dtype, const int count, const TargetPointType *clusters, const uint16_t scr_rng);

/**
 * 函数名称: radar_algo_gesture_rthres
 * 功能描述: 手势距离阈值设置
 * 输入参数: range -- 距离阈值(cm)
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algo_gesture_rthres(const uint16_t range);

/**
 * 函数名称: radar_algo_bright_screent_rthres
 * 功能描述: 亮屏距离阈值设置
 * 输入参数: range -- 距离阈值(cm)
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_algo_bright_screent_rthres(const uint16_t range);

#if defined(__cplusplus)
}
#endif

#endif
