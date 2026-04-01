#ifndef PC_RADAR_ALGO_FLOAT_V2_INCLUDE
#define PC_RADAR_ALGO_FLOAT_V2_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define PC_RADAR_PRINTF_RESULT_ENABLE 0
typedef int16_t radar_ifadc_type_t[2][64][128];
typedef int16_t radar_fftxd_type_t[2][20][64][2];

int pc_radar_run_from_radar_mem(const int8_t *frame_bytes, size_t frame_bytes_count);
int pc_radar_run_from_2dfft_frame(const radar_fftxd_type_t *frame);
int pc_radar_run_from_2dfft_frames(const radar_fftxd_type_t *frames, int frame_count);
int pc_radar_run_from_2dfft_file(const char *path);
int pc_radar_run_from_1dfft_file(const char *path);
int pc_radar_run_from_adc_file(const char *path);

#ifdef __cplusplus
}
#endif

#endif

