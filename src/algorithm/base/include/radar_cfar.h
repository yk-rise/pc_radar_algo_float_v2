#ifndef RADAR_CFAR_INCLUDE
#define RADAR_CFAR_INCLUDE

#if defined(__cplusplus)
extern "C" {
#endif

#include "radar_struct.h"

/**
 * 函数名称: radar_execute_cfar
 * 功能描述: 对2DFFT结果进行CFAR目标检测和插值
 * 输入参数: fft2d_input -- 2DFFT数据
 * 输出参数: result      -- 用于返回详细检测列表(可选)
 * 返回说明: 成功返回检测目标数，失败返回0
 * 其他说明: 无
 */
int radar_execute_cfar(const void *fft2d_input, DetectResultType **result);

#if defined(__cplusplus)
}
#endif

#endif
