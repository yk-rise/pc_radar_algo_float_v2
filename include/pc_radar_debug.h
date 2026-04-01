#ifndef PC_RADAR_DEBUG_INCLUDE
#define PC_RADAR_DEBUG_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#include "radar_struct.h"

#ifndef PC_RADAR_DEBUG_ENABLE
#define PC_RADAR_DEBUG_ENABLE 1
#endif

void pc_radar_debug_print_detect_results(const DetectResultType *result, int dcount);
void pc_radar_debug_print_target_points(const TargetPointType *points, int dcount);
void pc_radar_debug_print_gesture_result(int retcode, int classify);
void pc_radar_debug_print_float_array(const char *title, const float *values, int count);
void pc_radar_debug_print_cfar_input_i16_preview(const void *fft2d_input, int value_count);
void pc_radar_debug_print_cfar_complex_preview(const char *title, const float *complex_buffer, int pair_count);
void pc_radar_debug_print_cfar_map_preview(const char *title, const float *map, int rows, int cols, int preview_rows, int preview_cols);
void pc_radar_debug_print_byte_preview(const char *title, const int8_t *values, size_t count, size_t preview_count);

#if PC_RADAR_DEBUG_ENABLE
#define PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount) pc_radar_debug_print_detect_results((result), (dcount))
#define PC_RADAR_DEBUG_TARGET_POINTS(points, dcount)  pc_radar_debug_print_target_points((points), (dcount))
#define PC_RADAR_DEBUG_GESTURE_RESULT(retcode, classify) pc_radar_debug_print_gesture_result((retcode), (classify))
#define PC_RADAR_DEBUG_FLOAT_ARRAY(title, values, count) pc_radar_debug_print_float_array((title), (values), (count))
#define PC_RADAR_DEBUG_CFAR_INPUT_I16(fft2d_input, value_count) pc_radar_debug_print_cfar_input_i16_preview((fft2d_input), (value_count))
#define PC_RADAR_DEBUG_CFAR_COMPLEX(title, complex_buffer, pair_count) pc_radar_debug_print_cfar_complex_preview((title), (complex_buffer), (pair_count))
#define PC_RADAR_DEBUG_CFAR_MAP(title, map, rows, cols, preview_rows, preview_cols) pc_radar_debug_print_cfar_map_preview((title), (const float *)(map), (rows), (cols), (preview_rows), (preview_cols))
#define PC_RADAR_DEBUG_BYTE_PREVIEW(title, values, count, preview_count) pc_radar_debug_print_byte_preview((title), (values), (count), (preview_count))
#else
#define PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount) ((void)0)
#define PC_RADAR_DEBUG_TARGET_POINTS(points, dcount)  ((void)0)
#define PC_RADAR_DEBUG_GESTURE_RESULT(retcode, classify) ((void)0)
#define PC_RADAR_DEBUG_FLOAT_ARRAY(title, values, count) ((void)0)
#define PC_RADAR_DEBUG_CFAR_INPUT_I16(fft2d_input, value_count) ((void)0)
#define PC_RADAR_DEBUG_CFAR_COMPLEX(title, complex_buffer, pair_count) ((void)0)
#define PC_RADAR_DEBUG_CFAR_MAP(title, map, rows, cols, preview_rows, preview_cols) ((void)0)
#define PC_RADAR_DEBUG_BYTE_PREVIEW(title, values, count, preview_count) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif


