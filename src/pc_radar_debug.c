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
