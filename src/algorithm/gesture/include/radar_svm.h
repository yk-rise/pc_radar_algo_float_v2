#ifndef RADAR_SVM_INCLUDE
#define RADAR_SVM_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar.h"
#include "radar_svm_config.h"

/**
 * 函数名称: radar_gesture_classify
 * 功能描述: 雷达SVM手势分类
 * 输入参数: dcount   -- 目标数
 *           pts      -- 目标点信息
 *           rTh      -- 用户界面距离阈值(cm)
 * 输出参数: classify -- 输出结果(0:无手势，1:左挥手手势，-1:右挥手手势)
 * 返回说明: 返回状态码，0表示成功，其他值表示失败
 */
int radar_gesture_classify(const int dcount, const TargetPointType *pts, const uint16_t rTh, int *classify);

#if defined(__cplusplus)
}
#endif

#endif
