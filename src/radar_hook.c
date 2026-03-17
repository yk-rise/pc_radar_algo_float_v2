#include "radar_algo.h"

/**
 * 函数名称: radar_algo_gesture_hook
 * 功能描述: 手势分类数据回调
 * 输入参数: classify -- -1: 右挥手手势(从左往右)；0: 无效手势；1: 左挥手手势(从右往左)
 * 输出参数: 无
 * 返回说明: 无
 */
__WEAK void radar_algo_gesture_hook(const int classify)
{

}

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
__WEAK void radar_algo_target_hook(const uint8_t dtype, const int count, const TargetPointType *clusters, const uint16_t scr_rng)
{

}
