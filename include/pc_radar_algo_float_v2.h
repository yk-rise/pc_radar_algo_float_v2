#ifndef PC_RADAR_ALGO_FLOAT_V2_INCLUDE
#define PC_RADAR_ALGO_FLOAT_V2_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef int16_t radar_ifadc_type_t[2][64][128];
typedef int16_t radar_fftxd_type_t[2][20][64][2];

int pc_radar_run_from_2dfft_file(const char *path);
int pc_radar_run_from_1dfft_file(const char *path);
int pc_radar_run_from_adc_file(const char *path);

#ifdef __cplusplus
}
#endif

#endif
