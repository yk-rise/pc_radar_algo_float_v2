#ifndef RADAR_DBF_INCLUDE
#define RADAR_DBF_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar_struct.h"

/**
 * 函数名称: radar_dbf_estimation
 * 功能描述: 测量目标点角度
 * 输入参数: fft2d_input   -- 2DFFT数据
 *           result        -- CFAR配置参数
 *           cfar_points   -- CFAR输出点数
 * 输出参数: points_output -- 目标信息列表(不为空的话，需要调用方释放内存)
 * 返回说明: 返回目标数量
 * 其他说明: 无
 */
int radar_dbf_estimation(const void *fft2d_input, const DetectResultType *result, int cfar_points, TargetPointType **points_output);

#if defined(__cplusplus)
}
#endif

#endif
