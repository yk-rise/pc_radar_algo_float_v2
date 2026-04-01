#include "pc_radar_debug.h"

static void pc_radar_debug_print_section(const char *title)
{
    printf("\n============================================================\n");
    printf("%s\n", title);
    printf("============================================================\n");
}

void pc_radar_debug_print_detect_results(const DetectResultType *result, int dcount)
{
    pc_radar_debug_print_section("CFAR Result");
    printf("count = %d\n", dcount);

    if ((result == NULL) || (dcount <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int i = 0; i < dcount; ++i) {
        printf("[%02d] range_idx=%-3d vel_idx=%-3d snr=%8.2f noise=%10.2f amp=%10.2f range_fine=%8.2f vel_fine=%8.2f\n",
            i,
            result[i].range_index,
            result[i].velocity_index,
            result[i].snr,
            result[i].noise,
            result[i].amplitude,
            result[i].range_fine,
            result[i].velocity_fine);
    }
}

void pc_radar_debug_print_target_points(const TargetPointType *points, int dcount)
{
    pc_radar_debug_print_section("DBF Result");
    printf("count = %d\n", dcount);

    if ((points == NULL) || (dcount <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int i = 0; i < dcount; ++i) {
        printf("[%02d] range=%8.3f angle=%8.3f velocity=%8.3f snr=%8.3f powerdb=%8.3f amplitude=%10.3f\n",
            i,
            points[i].range,
            points[i].angle,
            points[i].velocity,
            points[i].snr,
            points[i].powerdb,
            points[i].amplitude);
    }
}

void pc_radar_debug_print_gesture_result(int retcode, int classify)
{
    pc_radar_debug_print_section("Gesture Result");
    printf("retcode  = %d\n", retcode);
    printf("classify = %d\n", classify);
}

void pc_radar_debug_print_float_array(const char *title, const float *values, int count)
{
    pc_radar_debug_print_section(title != NULL ? title : "Float Array");
    printf("count = %d\n", count);

    if ((values == NULL) || (count <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int i = 0; i < count; ++i) {
        printf("[%02d] %12.6f\n", i, values[i]);
    }
}

void pc_radar_debug_print_cfar_input_i16_preview(const void *fft2d_input, int value_count)
{
    const int16_t *values = (const int16_t *)fft2d_input;
    const int preview = (value_count < 16) ? value_count : 16;

    pc_radar_debug_print_section("CFAR Input int16 Preview");
    printf("value_count = %d\n", value_count);

    if ((values == NULL) || (value_count <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int i = 0; i < preview; ++i) {
        printf("[%02d] %6d%s", i, values[i], ((i % 8) == 7 || i == preview - 1) ? "\n" : "  ");
    }
}

void pc_radar_debug_print_cfar_complex_preview(const char *title, const float *complex_buffer, int pair_count)
{
    const int preview = (pair_count < 8) ? pair_count : 8;

    pc_radar_debug_print_section(title != NULL ? title : "CFAR Complex Preview");
    printf("pair_count = %d\n", pair_count);

    if ((complex_buffer == NULL) || (pair_count <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int i = 0; i < preview; ++i) {
        printf("[%02d] re=%10.3f im=%10.3f\n", i, complex_buffer[i * 2 + 0], complex_buffer[i * 2 + 1]);
    }
}

void pc_radar_debug_print_cfar_map_preview(const char *title, const float *map, int rows, int cols, int preview_rows, int preview_cols)
{
    const int rlim = (rows < preview_rows) ? rows : preview_rows;
    const int clim = (cols < preview_cols) ? cols : preview_cols;

    pc_radar_debug_print_section(title != NULL ? title : "CFAR Map Preview");
    printf("rows = %d, cols = %d\n", rows, cols);

    if ((map == NULL) || (rows <= 0) || (cols <= 0)) {
        printf("<empty>\n");
        return;
    }

    for (int r = 0; r < rlim; ++r) {
        printf("row[%02d]", r);
        for (int c = 0; c < clim; ++c) {
            printf(" %10.3f", map[r * cols + c]);
        }
        printf("\n");
    }
}

void pc_radar_debug_print_byte_preview(const char *title, const int8_t *values, size_t count, size_t preview_count)
{
    size_t preview = (count < preview_count) ? count : preview_count;

    pc_radar_debug_print_section(title != NULL ? title : "Byte Preview");
    printf("byte_count = %zu\n", count);

    if ((values == NULL) || (count == 0U)) {
        printf("<empty>\n");
        return;
    }

    for (size_t i = 0; i < preview; ++i) {
        printf("[%02zu] %4d%s", i, values[i], (((i % 16U) == 15U) || (i == (preview - 1U))) ? "\n" : "  ");
    }
}
