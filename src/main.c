#include "pc_radar_algo_float_v2.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <adc|1dfft|2dfft> <frame.txt|frame.bin>\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "2dfft") == 0) {
        return pc_radar_run_from_2dfft_file(argv[2]);
    }

    if (strcmp(argv[1], "1dfft") == 0) {
        return pc_radar_run_from_1dfft_file(argv[2]);
    }

    if (strcmp(argv[1], "adc") == 0) {
        return pc_radar_run_from_adc_file(argv[2]);
    }

    fprintf(stderr, "Unsupported mode: %s\n", argv[1]);
    return 1;
}
