#ifndef PC_RADAR_DEBUG_INCLUDE
#define PC_RADAR_DEBUG_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#include "radar_struct.h"

#ifndef PC_RADAR_DEBUG_ENABLE
#define PC_RADAR_DEBUG_ENABLE 1
#endif

void pc_radar_debug_print_detect_results(const DetectResultType *result, int dcount);
void pc_radar_debug_print_target_points(const TargetPointType *points, int dcount);
void pc_radar_debug_print_gesture_result(int retcode, int classify);
void pc_radar_debug_print_float_array(const char *title, const float *values, int count);

#if PC_RADAR_DEBUG_ENABLE
#define PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount) pc_radar_debug_print_detect_results((result), (dcount))
#define PC_RADAR_DEBUG_TARGET_POINTS(points, dcount)  pc_radar_debug_print_target_points((points), (dcount))
#define PC_RADAR_DEBUG_GESTURE_RESULT(retcode, classify) pc_radar_debug_print_gesture_result((retcode), (classify))
#define PC_RADAR_DEBUG_FLOAT_ARRAY(title, values, count) pc_radar_debug_print_float_array((title), (values), (count))
#else
#define PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount) ((void)0)
#define PC_RADAR_DEBUG_TARGET_POINTS(points, dcount)  ((void)0)
#define PC_RADAR_DEBUG_GESTURE_RESULT(retcode, classify) ((void)0)
#define PC_RADAR_DEBUG_FLOAT_ARRAY(title, values, count) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif
