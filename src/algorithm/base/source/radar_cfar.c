#include "radar_cfar.h"
#include "pc_radar_debug.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)

/* ???CFAR???????*/
static DetectResultType sDetectResult[CONFIG_OBJECT_MAX];

/* CFAR?????? */
static CfarConfigType sCFARParam = {
    .range_dim         = CONFIG_RAMP_RNGBIN,
    .velocity_dim      = CONFIG_RAMP_CHIRPS,
    .guard_cell        = {CONFIG_CFAR_NUM_GUARD_VEL, CONFIG_CFAR_NUM_GUARD_RANGE},
    .reference_cell    = {CONFIG_CFAR_NUM_TRAIN_VEL, CONFIG_CFAR_NUM_TRAIN_RANGE},
    .threshold_factors = CONFIG_CFAR_TH_AMP,
};

/* ???????????sort?????? */
static int udsf_compare_subfunc(const void *a, const void *b)
{
    if (*(const float *)a < *(const float *)b) {
        return -1;
    }

    if (*(const float *)a > *(const float *)b) {
        return 1;
    }

    return 0;
}

/* ???????????????????median(reference_cell) */
static float udsf_median_of_four(float a, float b, float c, float d)
{
    float temp[4] = {a, b, c, d};

    /* ??? */
    qsort(temp, 4, sizeof(float), udsf_compare_subfunc);

    /* ???????????????????2)?????????????????????????(temp[1]+temp[2])/2???MatLab?????????????? */
    return temp[2];
}

/* ?????????????????????????? */
static float udsf_max_of_four(float a, float b, float c, float d)
{
    float m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    if (d > m) m = d;
    return m;
}

/* ????????????????????????????? ??????????????0, size-1]????????????????????? */
static int udsf_get_circular_index(int index, int size)
{
    /* ????????0, size-1]?????*/
    if (index >= size) {
        return (index - size);
    }

    if (index < 0) {
        return (index + size);
    }

    return index;
}

/* ?????????????????????????????delta */
static float udsf_peak_interpolation_core(const float data_map[][CONFIG_RAMP_RNGBIN], int peakIdx, int fixedIdx, uint8_t isVelocityDim, int dimSize)
{
    /* ????????????3????????????0??? */
    if (dimSize < 3) {
        return 0.0f;
    }

    /* 0????? ???????????*/
    int currIdx = peakIdx;

    /* ???????????*/
    float A_prev = 0, A_curr = 0, A_next = 0;

    if (isVelocityDim) {
        /* ???????? data_map[???][???] */
        A_prev = data_map[udsf_get_circular_index(peakIdx - 1, dimSize)][fixedIdx];
        A_curr = data_map[currIdx][fixedIdx]; 
        A_next = data_map[udsf_get_circular_index(peakIdx + 1, dimSize)][fixedIdx];
    } else {
        if (peakIdx < 1) {
            A_curr = data_map[fixedIdx][currIdx];
            A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
            float del0 = A_next / (A_next + A_curr + DBL_EPSILON) * 0.5;
            return del0;    /* 0???bin?????? */
        }

        /* ???????? data_map[???][???] */
        A_prev = data_map[fixedIdx][udsf_get_circular_index(peakIdx - 1, dimSize)];
        A_curr = data_map[fixedIdx][currIdx]; 
        A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
    }

    /* ??????????????delta */
    float delta = 0.0;
    float denominator = A_prev - 2 * A_curr + A_next;

    /* ?????????????*/
    if (fabs(denominator) > DBL_EPSILON) {
        delta = 0.5 * (A_prev - A_next) / (denominator == 0 ? 1.0 : denominator);
    }

    /* ???????????-0.5, 0.5]???????????*/
    if (delta > 0.5) delta = 0.5;
    if (delta < -0.5) delta = -0.5;

    return delta;
}

/* ??FAR?????????????????????????????? */
static void udsf_refine_detections_interpolation(const float data_map[][CONFIG_RAMP_RNGBIN], DetectResultType *detections, int count, int velDim, int rangeDim)
{
    if ((count <= 0) || (detections == NULL)) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        int range_index = detections[i].range_index;        /* 0????????*/
        int velocity_index = detections[i].velocity_index;  /* 0???????? */

        /* ????????*/
        float delta_veloccity = udsf_peak_interpolation_core(data_map, velocity_index, range_index, 1, velDim);
        detections[i].velocity_fine = (float)velocity_index + delta_veloccity;

        /* ????????*/
        float delta_range = udsf_peak_interpolation_core(data_map, range_index, velocity_index, 0, rangeDim);
        detections[i].range_fine = (float)range_index + delta_range;
    }
}

/* ??????CFAR???????????????????? */
static int udsf_cfar2d_core(const float power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static float s_cfar_noise = -1.0f;
    const int guard_vel = sCFARParam.guard_cell[0];
    const int guard_range = sCFARParam.guard_cell[1];
    const int ref_vel = sCFARParam.reference_cell[0];
    const int ref_range = sCFARParam.reference_cell[1];
    const float thresholdFactor = sCFARParam.threshold_factors;

    /* ??????????????*/
    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* ??????????(?????????????????? */
    float noiseEstm = 0.0f;
    for (int i = N - 6; i < N; i++) {
        for (int j = M - 6; j < M; j++) {
            noiseEstm += power_map[udsf_get_circular_index(i, N)][udsf_get_circular_index(j, M)];
        }
    }
    noiseEstm /= 36.0f;
    if (s_cfar_noise > 0.0f) {
        noiseEstm = s_cfar_noise + CONFIG_CFAR_NCOEF * (noiseEstm - s_cfar_noise);
    }
    s_cfar_noise = noiseEstm;

    /* ??-based???1????????????????????????????????????????????MATLAB?????????????in */
    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const float currentAmp = power_map[v_idx][r_idx];
            /* ???????*/
            if (currentAmp <= (CONFIG_CFAR_TH_OFFSET * noiseEstm)) {
                continue;
            }

            /* ???????????????????*/
            float neighborMax;
            if (r_idx == 0) {
                neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx +1 ][r_idx + 1], power_map[v_idx][r_idx + 1]);
            } else {
                if ((1 == CONFIG_CFAR_RBIN_START_IDX) && (r_idx == 1)) {
                    neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx + 1][r_idx + 1], power_map[v_idx][r_idx + 1]);
                } else {
                    neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx][r_idx - 1], power_map[v_idx][r_idx + 1]);
                }
            }

            if (currentAmp < neighborMax) {
                continue;
            }

            /* ?????????????(????????????) */
            float sum_up = 0.0, sum_down = 0.0;
            float sum_left = 0.0, sum_right = 0.0;

            /* ????????)??????????*/
            for (int i = 0; i < ref_vel; i++) {
                /* ??????????Up): velIdx - guard_vel - ref_vel + i */
                int index_up = v_idx - guard_vel - ref_vel + i;
                sum_up += power_map[udsf_get_circular_index(index_up, N)][r_idx];

                /* ??????????Down): velIdx + guard_vel + 1 + i */
                int index_down = v_idx + guard_vel + 1 + i;
                sum_down += power_map[udsf_get_circular_index(index_down, N)][r_idx];
            }

            /* ????????)??????????*/
            for (int i = 0; i < ref_range; i++) {
                /* ??????????Left): rangeIdx - guard_range - ref_range + i */
                int index_left = r_idx - guard_range - ref_range + i;
                sum_left += power_map[v_idx][udsf_get_circular_index(index_left, M)];

                /* ??????????Right): rangeIdx + guard_range + 1 + i */
                int index_right = r_idx + guard_range + 1 + i;
                sum_right += power_map[v_idx][udsf_get_circular_index(index_right, M)];
            }

            /* ?????????????????(???????? */
            float mean_up = sum_up / ref_vel;
            float mean_down = sum_down / ref_vel;
            float mean_left = sum_left / ref_range;
            float mean_right = sum_right / ref_range;
            float noiseEst = udsf_median_of_four(mean_up, mean_down, mean_left, mean_right);

            /* ?????????????? */
            const float threshold = thresholdFactor * noiseEst;
            if ((currentAmp > threshold) && (dcount < CONFIG_OBJECT_MAX)) {
                /* ????????20 * log10(?????) */
                float snr = 0.0f;
                if (noiseEst > DBL_MIN) {   /* ??? DBL_MIN ??????????????*/
                    snr = 20.0 * log10(currentAmp / noiseEst);
                } else {
                    snr = 99.0f;
                }

                if ((int)snr != 99) {
                    /* ?????? */
                    sDetectResult[dcount].range_index = r_idx;           /* 0-based ?????? */
                    sDetectResult[dcount].velocity_index = v_idx;        /* 0-based ?????? */
                    sDetectResult[dcount].range_fine = (float)r_idx;     /* ???: ???????????? */
                    sDetectResult[dcount].velocity_fine = (float)v_idx;  /* ???: ???????????? */
                    sDetectResult[dcount].snr = snr;
                    sDetectResult[dcount].noise = noiseEst;
                    sDetectResult[dcount].amplitude = currentAmp;
                    dcount++;
                }
            }
        }
    }

    return dcount;
}

static int udsf_cfar2d_core_cut(const float power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static float s_cfar_noise = -1.0f;

    /* ??????????????*/
    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* ??????????(?????????????????? */
    float noiseEstm = 0.0f;
    for (int i = N - 6; i < N; i++) {
        for (int j = M - 6; j < M; j++) {
            noiseEstm += power_map[udsf_get_circular_index(i, N)][udsf_get_circular_index(j, M)];
        }
    }

    noiseEstm /= 36.0f;
    if (s_cfar_noise > 0.0f) {
        noiseEstm = s_cfar_noise + CONFIG_CFAR_NCOEF * (noiseEstm - s_cfar_noise);
    }
    s_cfar_noise = noiseEstm;

    float noiseEstInv = 1.0f / noiseEstm;
    float threshold = CONFIG_CFAR_TH_OFFSET * noiseEstm;

    /* ??-based???1????????????????????????????????????????????MATLAB?????????????in */
    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const float currentAmp = power_map[v_idx][r_idx];
            /* ???????*/
            if (currentAmp < threshold) {
                continue;
            }

            /* ???????????????????*/
            float neighborMax;
            if (r_idx == 0) {
                neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx +1 ][r_idx + 1], power_map[v_idx][r_idx + 1]);
            } else {
                if ((1 == CONFIG_CFAR_RBIN_START_IDX) && (r_idx == 1)) {
                    neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx + 1][r_idx + 1], power_map[v_idx][r_idx + 1]);
                } else {
                    neighborMax = udsf_max_of_four(power_map[v_idx - 1][r_idx], power_map[v_idx + 1][r_idx], power_map[v_idx][r_idx - 1], power_map[v_idx][r_idx + 1]);
                }
            }

            if (currentAmp < neighborMax) {
                continue;
            }

            /* ?????????????? */
            if (dcount < CONFIG_OBJECT_MAX) {
                /* ????????20 * log10(?????) */
                float snr = 0.0f;
                if (noiseEstm > DBL_MIN) {   /* ??? DBL_MIN ??????????????*/
                    snr = 20.0 * log10(currentAmp *noiseEstInv);
                } else {
                    snr = 99.0f;
                }

                if ((int)snr != 99) {
                    sDetectResult[dcount].range_index = r_idx;           /* 0-based ?????? */
                    sDetectResult[dcount].velocity_index = v_idx;        /* 0-based ?????? */
                    sDetectResult[dcount].range_fine = (float)r_idx;     /* ???: ???????????? */
                    sDetectResult[dcount].velocity_fine = (float)v_idx;  /* ???: ???????????? */
                    sDetectResult[dcount].snr = snr;
                    sDetectResult[dcount].noise = noiseEstm;
                    sDetectResult[dcount].amplitude = currentAmp;
                    dcount++;
                }
            }
        }
    }

    return dcount;
}

/**
 * ??????: radar_execute_cfar
 * ??????: ??DFFT??????CFAR????????????
 * ??????: fft2d_input -- 2DFFT???
 * ??????: result      -- ????????????????????
 * ??????: ??????????????????????
 * ??????: ??
 */
float temp_mag[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN];
float avg_amplitude_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN];
float complex_buffer[CONFIG_RAMP_CHIRPS * CONFIG_RAMP_RNGBIN * 2];
int radar_execute_cfar(const void *fft2d_input, DetectResultType **result)
{
    /* ????????????*/
    if ((sCFARParam.guard_cell[0] < 0) || (sCFARParam.guard_cell[1] < 0)
        || (sCFARParam.reference_cell[0] <= 0) || (sCFARParam.reference_cell[1] <= 0))
    {
        return 0;
    }

    int detect_count = 0;
    const uint8_t ants = CONFIG_RAMP_RXNUM;
    const uint8_t bins = CONFIG_RAMP_RNGBIN;
    const uint16_t chirps = CONFIG_RAMP_CHIRPS;
    const int16_t (*fft2dFrame)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2] = (const int16_t (*)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2])fft2d_input;
#if DEMO_FRAME_DEBUG
    PC_RADAR_DEBUG_CFAR_INPUT_I16(fft2d_input, ants * bins * chirps * 2);
#endif
    /* 1.????????????????(ncidata) */
    memset(temp_mag, 0x00, sizeof(temp_mag));
    memset(avg_amplitude_map, 0x00, sizeof(avg_amplitude_map));

    /* a.?????????????? */
    for (uint8_t ant = 0; ant < ants; ++ant) {
        memset(complex_buffer, 0x00, sizeof(complex_buffer));
        for (int i = 0; i < (chirps * bins); i++) {
            uint8_t rbin = i % bins;
            uint16_t chirp = i / bins;
            complex_buffer[i * 2 + 0] = (float)((*fft2dFrame)[ant][rbin][chirp][1]);   /* ??? */
            complex_buffer[i * 2 + 1] = (float)((*fft2dFrame)[ant][rbin][chirp][0]);   /* ??? */
        }
#if DEMO_FRAME_DEBUG
        PC_RADAR_DEBUG_CFAR_COMPLEX("CFAR Complex Preview", complex_buffer, chirps * bins);
#endif
        arm_cmplx_mag_f32(complex_buffer, (float32_t *)temp_mag[0], chirps * bins);
        for (uint16_t chirp = 0; chirp < chirps; ++chirp) {
            for (uint8_t rbin = 0; rbin < bins; ++rbin) {
                avg_amplitude_map[chirp][rbin] += temp_mag[chirp][rbin];
            }
        }
    }
#if DEMO_FRAME_DEBUG
    PC_RADAR_DEBUG_CFAR_MAP("CFAR Avg Amplitude Map Preview", avg_amplitude_map, chirps, bins, 4, 8);
#endif
    /* 2.??????????????????FAR????*/
    if (CONFIG_CFAR_TYPE == 1) {
        detect_count = udsf_cfar2d_core_cut(avg_amplitude_map, chirps, bins);
    } else {
        detect_count = udsf_cfar2d_core(avg_amplitude_map, chirps, bins);
    }

    /* 3.?????????????????????????(??????? */
    if (detect_count > 0) {
        /* ??????????????*/
        udsf_refine_detections_interpolation(avg_amplitude_map, sDetectResult, detect_count, chirps, bins);
        if (result) {
            *result = sDetectResult;
        }
    }

    /* ????????????????*/
    return detect_count;
}

#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)

/* ?????????????*/
DetectResultType sDetectResult[CONFIG_OBJECT_MAX];

/* CFAR?????? */
static CfarConfigType sCFARParam = {
    .range_dim         = CONFIG_RAMP_RNGBIN,
    .velocity_dim      = CONFIG_RAMP_CHIRPS,
    .guard_cell        = {CONFIG_CFAR_NUM_GUARD_VEL, CONFIG_CFAR_NUM_GUARD_RANGE},
    .reference_cell    = {CONFIG_CFAR_NUM_TRAIN_VEL, CONFIG_CFAR_NUM_TRAIN_RANGE},
    .threshold_factors = CONFIG_CFAR_TH_AMP_Q4,
};

/******************** ?????????????********************/

/* ========================================================
   1. ??????????????? & SNR ??? 
   ======================================================== */
static uint32_t udsf_log2_q16(uint32_t x) 
{
    if (x == 0) return 0;
    uint32_t n = 0;
    uint32_t temp = x;
    
    while (temp > 1) {
        temp >>= 1;
        n++;
    }
    
    uint32_t y = x << (31 - n);
    uint32_t frac = 0;
    
    for (int i = 0; i < 16; i++) {
        uint64_t y2 = ((uint64_t)y * y) >> 31;
        if (y2 >= 0x100000000ULL) { 
            frac |= (1U << (15 - i));
            y = (uint32_t)(y2 >> 1);
        } else {
            y = (uint32_t)y2;
        }
    }
    return (n << 16) | frac;
}

static int16_t udsf_exact_snr_q8(uint32_t amp, uint32_t noise) 
{
    /* ????????????????????????????? 99dB (99 * 256 = 25344) */
    if (noise == 0 || amp == 0) return 25344;
    
    uint32_t log2_amp = udsf_log2_q16(amp);
    uint32_t log2_noise = udsf_log2_q16(noise);
    
    /* ???????>= 0??????????????? amp > threshold > noise */
    int32_t diff_q16 = (int32_t)log2_amp - (int32_t)log2_noise;
    if (diff_q16 < 0) diff_q16 = 0;
    
    /* ??????Bug????????? 64 ????????????????????? 32 ???????????*/
    int32_t snr_q8 = (int32_t)(((int64_t)diff_q16 * 1541) >> 16);
    
    /* ??????????????int16_t ????????*/
    if (snr_q8 > 25344) snr_q8 = 25344; 
    
    return (int16_t)snr_q8;
}

/* ?????????????????????????????sort) */
static inline uint32_t udsf_median_of_four_u32(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    uint32_t temp;
    if (a > b) { temp = a; a = b; b = temp; }
    if (c > d) { temp = c; c = d; d = temp; }
    if (a > c) { temp = a; a = c; c = temp; }
    if (b > d) { temp = b; b = d; d = temp; }
    if (b > c) { temp = b; b = c; c = temp; }
    return c;
}

/* ???4??????????*/
static inline uint32_t udsf_max_of_four_u32(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    uint32_t m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    if (d > m) m = d;
    return m;
}

/* ???????og2????????8????????log2(2) = 256????????????20*log10 */
static int16_t fast_log2_q8(uint32_t x)
{
    if (x == 0) return 0;
    uint16_t int_part = 0;
    uint32_t temp = x;
    while (temp > 1) {
        temp >>= 1;
        int_part++;
    }
    /* ?????????(????????????????????*/
    uint16_t frac_part = 0;
    if (int_part >= 8) {
        frac_part = (x >> (int_part - 8)) & 0xFF;
    } else {
        frac_part = (x << (8 - int_part)) & 0xFF;
    }
    return (int_part << 8) | frac_part;
}

/* ???????????? */
static inline int udsf_get_circular_index(int index, int size)
{
    if (index >= size) return (index - size);
    if (index < 0) return (index + size);
    return index;
}

/* ========================================================
   1. ????????????????????????????? delta
   ?????? Q8 ???????????(128 ??? 0.5??-128 ??? -0.5)
   ======================================================== */
static int16_t udsf_peak_interpolation_core(const uint32_t data_map[][CONFIG_RAMP_RNGBIN], int peakIdx, int fixedIdx, uint8_t isVelocityDim, int dimSize)
{
    /* ???????????? 3???????????? 0 ??? */
    if (dimSize < 3) {
        return 0;
    }

    /* ??? 32 ??????????????? 16 ????????*/
    uint32_t A_prev = 0, A_curr = 0, A_next = 0;
    int currIdx = peakIdx;

    if (isVelocityDim) {
        /* ???????? data_map[???][???] */
        A_prev = data_map[udsf_get_circular_index(peakIdx - 1, dimSize)][fixedIdx];
        A_curr = data_map[currIdx][fixedIdx]; 
        A_next = data_map[udsf_get_circular_index(peakIdx + 1, dimSize)][fixedIdx];
    } else {
        if (peakIdx < 1) {
            /* 0 ??? bin ???????????
             * ????????: delta = 0.5 * A_next / (A_next + A_curr) 
             */
            A_curr = data_map[fixedIdx][currIdx];
            A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
            
            uint32_t den_spec = A_next + A_curr;
            if (den_spec != 0) {
                /* ??? 128 ????????? 0.5???????????Q8 ???????????*/
                return (int16_t)((A_next * 128) / den_spec); 
            }
            return 0;
        }

        /* ??????????? data_map[???][???] */
        A_prev = data_map[fixedIdx][udsf_get_circular_index(peakIdx - 1, dimSize)];
        A_curr = data_map[fixedIdx][currIdx]; 
        A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
    }

    /* ?????????????? delta 
     * ??????: delta = 0.5 * (A_prev - A_next) / (A_prev - 2*A_curr + A_next)
     */
     
    /* ????????32 ?????????????????????????????*/
    int32_t num = (int32_t)A_prev - (int32_t)A_next;
    int32_t den = (int32_t)A_prev - 2 * (int32_t)A_curr + (int32_t)A_next;
    
    int16_t delta_q8 = 0;

    /* ??????????????? 0 ?????*/
    if (den != 0) {
        /* (num * 128) / den ????? (num / den) * 0.5 * 256 (256??8???) */
        delta_q8 = (int16_t)((num * 128) / den); 
    }

    /* ???????????? [-0.5, 0.5] ????????Q8 ?????[-128, 128]???????????*/
    if (delta_q8 > 128) delta_q8 = 128;
    if (delta_q8 < -128) delta_q8 = -128;

    return delta_q8;
}

/* ========================================================
   ??CFAR ??????????????????????????????
   ======================================================== */
static void udsf_refine_detections_interpolation(const uint32_t data_map[][CONFIG_RAMP_RNGBIN], DetectResultType *detections, int count, int velDim, int rangeDim)
{
    if ((count <= 0) || (detections == NULL)) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        /* ??? 0-based ????????*/
        int range_index = detections[i].range_index;        
        int velocity_index = detections[i].velocity_index;  

        /* --- ????????--- */
        int16_t delta_velocity = udsf_peak_interpolation_core(data_map, velocity_index, range_index, 1, velDim);
        
        /* ?????Q8 ??????(????????? 8 ?? + ????????*/
        detections[i].velocity_fine = (int16_t)((velocity_index << 8) + delta_velocity);

        /* --- ????????--- */
        int16_t delta_range = udsf_peak_interpolation_core(data_map, range_index, velocity_index, 0, rangeDim);
        
        /* ?????Q8 ??????(????????? 8 ?? + ????????*/
        detections[i].range_fine = (int16_t)((range_index << 8) + delta_range);
    }
}

/* ========================================================
   ????????SNR ?????CFAR ??????
   ======================================================== */
static int udsf_cfar2d_core(const uint32_t power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static uint32_t s_cfar_noise = 0;
    static uint8_t s_noise_init = 0;

    const int guard_vel = sCFARParam.guard_cell[0];
    const int guard_range = sCFARParam.guard_cell[1];
    const int ref_vel = sCFARParam.reference_cell[0];
    const int ref_range = sCFARParam.reference_cell[1];
    const uint32_t thresholdFactor_Q4 = sCFARParam.threshold_factors; 

    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    uint32_t noise_sum = 0;
    for (int i = N - 6; i < N; i++) {
        for (int j = M - 6; j < M; j++) {
            noise_sum += power_map[udsf_get_circular_index(i, N)][udsf_get_circular_index(j, M)];
        }
    }
    uint32_t noiseEstm = (noise_sum + 18) / 36;
    
    if (s_noise_init) {
        int32_t diff = (int32_t)noiseEstm - (int32_t)s_cfar_noise;
        noiseEstm = s_cfar_noise + (uint32_t)((diff * CONFIG_CFAR_NCOEF_Q15) >> 15);
        s_cfar_noise = noiseEstm;
    } else {
        s_cfar_noise = noiseEstm;
        s_noise_init = 1;
    }

    uint32_t quick_threshold = (CONFIG_CFAR_TH_OFFSET_Q4 * noiseEstm) >> 4;

    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const uint32_t currentAmp = power_map[v_idx][r_idx];
            
            if (currentAmp <= quick_threshold) continue;

            uint32_t neighborMax;
            if (r_idx == 0) {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx+1][r_idx+1], power_map[v_idx][r_idx+1]);
            } else if ((1 == CONFIG_CFAR_RBIN_START_IDX) && (r_idx == 1)) {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx+1][r_idx+1], power_map[v_idx][r_idx+1]);
            } else {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx][r_idx-1], power_map[v_idx][r_idx+1]);
            }

            if (currentAmp < neighborMax) continue;

            uint32_t sum_up = 0, sum_down = 0, sum_left = 0, sum_right = 0;

            for (int i = 0; i < ref_vel; i++) {
                sum_up += power_map[udsf_get_circular_index(v_idx - guard_vel - ref_vel + i, N)][r_idx];
                sum_down += power_map[udsf_get_circular_index(v_idx + guard_vel + 1 + i, N)][r_idx];
            }
            for (int i = 0; i < ref_range; i++) {
                sum_left += power_map[v_idx][udsf_get_circular_index(r_idx - guard_range - ref_range + i, M)];
                sum_right += power_map[v_idx][udsf_get_circular_index(r_idx + guard_range + 1 + i, M)];
            }
            
            
            /* 1. ?????????????????????(16????? */
            uint32_t mean_up = (sum_up + (ref_vel >> 1)) / ref_vel;
            uint32_t mean_down = (sum_down + (ref_vel >> 1)) / ref_vel;
            uint32_t mean_left = (sum_left + (ref_range >> 1)) / ref_range;
            uint32_t mean_right = (sum_right + (ref_range >> 1)) / ref_range;
            uint32_t noiseEst = udsf_median_of_four_u32(mean_up, mean_down, mean_left, mean_right);
            uint32_t threshold = (noiseEst * thresholdFactor_Q4) >> 4;
            if ((currentAmp > threshold) && (dcount < CONFIG_OBJECT_MAX)) {
                
                /* 2. ?????? SNR ????????? (Q8 - 256????? */
                uint32_t hp_mean_up = ((sum_up << 4) + (ref_vel >> 1)) / ref_vel;
                uint32_t hp_mean_down = ((sum_down << 4) + (ref_vel >> 1)) / ref_vel;
                uint32_t hp_mean_left = ((sum_left << 4) + (ref_range >> 1)) / ref_range;
                uint32_t hp_mean_right = ((sum_right << 4) + (ref_range >> 1)) / ref_range;
                uint32_t hp_noiseEst = udsf_median_of_four_u32(hp_mean_up, hp_mean_down, hp_mean_left, hp_mean_right);

                /* 3. ??? SNR???????????Q8 (256?????
                   ??16??? currentAmp ??? 4?????? 256??*/
                int16_t snr_q8 = udsf_exact_snr_q8(currentAmp << 4, hp_noiseEst);

                /* 4. ????????? */
                if (snr_q8 != 25344) {
                    sDetectResult[dcount].range_index = r_idx;           
                    sDetectResult[dcount].velocity_index = v_idx;        
                    sDetectResult[dcount].range_fine = r_idx << 8;       
                    sDetectResult[dcount].velocity_fine = v_idx << 8;    
                    
                    sDetectResult[dcount].snr = snr_q8; // Q8 ???

                    /* 5. ???????????? (????????Q8 ???) */
                    sDetectResult[dcount].amplitude = (uint32_t)(currentAmp << 4); // 16x16 = 256(Q8)
                    sDetectResult[dcount].noise = (uint32_t)hp_noiseEst;           // ?????256(Q8)
                    
                    dcount++;
                }
            }
        }
    }
    return dcount;
}

/**
 * ??????: udsf_cfar2d_core_cut
 * ??????: ???????? CFAR ????(??????/??????)
 * ??????: power_map -- ???????????(32???16?????
 *           N         -- ????????
 *           M         -- ????????
 * ?????   ?????????????
 */
static int udsf_cfar2d_core_cut(const uint32_t power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static uint32_t s_cfar_noise = 0;       
    static uint8_t s_noise_init = 0;

    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* 1. ????????? (16????? */
    uint32_t noise_sum = 0;
    for (int i = N - 6; i < N; i++) {
        for (int j = M - 6; j < M; j++) {
            noise_sum += power_map[udsf_get_circular_index(i, N)][udsf_get_circular_index(j, M)];
        }
    }
    uint32_t noiseEstm_16x = (noise_sum + 18) / 36;
    
    /* 2. IIR ??? (?????????????? */
    if (s_noise_init) {
        int32_t diff = (int32_t)noiseEstm_16x - (int32_t)s_cfar_noise;
        int32_t update = (diff * (int32_t)CONFIG_CFAR_NCOEF_Q15) >> 15;
        noiseEstm_16x = (uint32_t)((int32_t)s_cfar_noise + update);
        s_cfar_noise = noiseEstm_16x;
    } else {
        s_cfar_noise = noiseEstm_16x;
        s_noise_init = 1;
    }

    /* 3. ?????? (??? CONFIG_CFAR_TH_OFFSET_Q4 ??core ?????threshold_factors ???? */
    uint32_t threshold_16x = (CONFIG_CFAR_TH_OFFSET_Q4 * noiseEstm_16x) >> 4;

    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const uint32_t currentAmp_16x = power_map[v_idx][r_idx];
            
            if (currentAmp_16x < threshold_16x) continue;

            /* 4. ??????????*/
            uint32_t neighborMax;
            if (r_idx == 0) {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx+1][r_idx+1], power_map[v_idx][r_idx+1]);
            } else if ((1 == CONFIG_CFAR_RBIN_START_IDX) && (r_idx == 1)) {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx+1][r_idx+1], power_map[v_idx][r_idx+1]);
            } else {
                neighborMax = udsf_max_of_four_u32(power_map[v_idx-1][r_idx], power_map[v_idx+1][r_idx], power_map[v_idx][r_idx-1], power_map[v_idx][r_idx+1]);
            }

            if (currentAmp_16x < neighborMax) continue;

            if (dcount < CONFIG_OBJECT_MAX) {
                /* 5. ??? SNR??????????????4 ????????Q8 (256?? */
                int16_t snr_q8 = udsf_exact_snr_q8(currentAmp_16x << 4, noiseEstm_16x << 4);

                if (snr_q8 != 25344) {
                    sDetectResult[dcount].range_index = r_idx;           
                    sDetectResult[dcount].velocity_index = v_idx;        
                    sDetectResult[dcount].range_fine = (int16_t)(r_idx << 8); 
                    sDetectResult[dcount].velocity_fine = (int16_t)(v_idx << 8);    
                    sDetectResult[dcount].snr = snr_q8;
                    
                    /* ???????????? (Q8) */
                    sDetectResult[dcount].noise = (uint32_t)(noiseEstm_16x << 4);
                    sDetectResult[dcount].amplitude = (uint32_t)(currentAmp_16x << 4);
                    
                    dcount++;
                }
            }
        }
    }
    return dcount;
}

/* ========================================================
   ????????????????(???????????
   ======================================================== */
static inline uint32_t udsf_fast_isqrt(uint32_t val) 
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; /* ?????????????*/
    
    while (bit > val) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

uint32_t avg_amplitude_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN];

/* ========================================================
   ??? CFAR ????????
   ======================================================== */
int radar_execute_cfar(const void *fft2d_input, DetectResultType **result)
{
    /* ????????????*/
    if ((sCFARParam.guard_cell[0] < 0) || (sCFARParam.guard_cell[1] < 0)
        || (sCFARParam.reference_cell[0] <= 0) || (sCFARParam.reference_cell[1] <= 0))
    {
        return 0;
    }

    int detect_count = 0;
    const uint8_t ants = CONFIG_RAMP_RXNUM;
    const uint8_t bins = CONFIG_RAMP_RNGBIN;
    const uint16_t chirps = CONFIG_RAMP_CHIRPS;
    
    /* ?????? 16????????? */
    const int16_t (*fft2dFrame)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2] = 
        (const int16_t (*)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2])fft2d_input;

    /* ??? 32??????????*/
    memset(avg_amplitude_map, 0x00, sizeof(avg_amplitude_map));

    /* 1. ??????????????????????????*/
    for (uint8_t ant = 0; ant < ants; ++ant) {
        for (uint16_t chirp = 0; chirp < chirps; ++chirp) {
            for (uint8_t rbin = 0; rbin < bins; ++rbin) {
                /* ??? Q (???/???) ??I (???/???) */
                int16_t q = (*fft2dFrame)[ant][rbin][chirp][0]; 
                int16_t i = (*fft2dFrame)[ant][rbin][chirp][1]; 
                
                /* ????????*/
                uint32_t abs_i = (i > 0) ? i : -i;
                uint32_t abs_q = (q > 0) ? q : -q;
                
                /* ??? I^2 + Q^2 (??? abs_i/q ????32767??????????? 21.4????????? 32 ???????? */
                uint32_t sum_sq = (abs_i * abs_i) + (abs_q * abs_q);
                uint32_t mag_scaled_by_16;
                
                /* ?????????????????
                 * 1. ????????(sum_sq < 2^24)????????? 256 ??<<8)??????????????????????????????16 ??????????????
                 * 2. ?????????????????????????????????????16 ??<<4)???????????32 ???????
                 */
                if (sum_sq < 0x01000000) { 
                    mag_scaled_by_16 = udsf_fast_isqrt(sum_sq << 8);
                } else {
                    mag_scaled_by_16 = udsf_fast_isqrt(sum_sq) << 4;
                }
                
                /* ??????????????????????????? */
                avg_amplitude_map[chirp][rbin] += mag_scaled_by_16;
            }
        }
    }

    /* 2. ??"???????? ????????CFAR ????*/
    if (CONFIG_CFAR_TYPE == 1) {
        detect_count = udsf_cfar2d_core_cut(avg_amplitude_map, chirps, bins);
    } else {
        detect_count = udsf_cfar2d_core(avg_amplitude_map, chirps, bins);
    }

    /* 3. ??????????????????(????????????*/
    if (detect_count > 0) {
        udsf_refine_detections_interpolation(avg_amplitude_map, sDetectResult, detect_count, chirps, bins);
        if (result) {
            *result = sDetectResult;
        }
    }

    /* ????????????????*/
    return detect_count;
}

#endif
