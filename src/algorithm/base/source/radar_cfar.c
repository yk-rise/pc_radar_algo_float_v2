#include "radar_cfar.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)

/* 存储CFAR检测结果 */
static DetectResultType sDetectResult[CONFIG_OBJECT_MAX];

/* CFAR配置参数 */
static CfarConfigType sCFARParam = {
    .range_dim         = CONFIG_RAMP_RNGBIN,
    .velocity_dim      = CONFIG_RAMP_CHIRPS,
    .guard_cell        = {CONFIG_CFAR_NUM_GUARD_VEL, CONFIG_CFAR_NUM_GUARD_RANGE},
    .reference_cell    = {CONFIG_CFAR_NUM_TRAIN_VEL, CONFIG_CFAR_NUM_TRAIN_RANGE},
    .threshold_factors = CONFIG_CFAR_TH_AMP,
};

/* 比较函数，用于qsort排序数组 */
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

/* 找到四个数的中位数。用于 median(reference_cell) */
static float udsf_median_of_four(float a, float b, float c, float d)
{
    float temp[4] = {a, b, c, d};

    /* 排序 */
    qsort(temp, 4, sizeof(float), udsf_compare_subfunc);

    /* 返回排序后的第三个值(索引2)作为中位数近似。(如果严格中位数是(temp[1]+temp[2])/2，但MatLab逻辑通常取第三个值) */
    return temp[2];
}

/* 找到四个数中的最大值(用于峰值分组) */
static float udsf_max_of_four(float a, float b, float c, float d)
{
    float m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    if (d > m) m = d;
    return m;
}

/* 处理循环移位边界，返回合法的索引。注意: 该函数返回的索引是[0, size-1]之间的，可以直接用于数组访问 */
static int udsf_get_circular_index(int index, int size)
{
    /* 确保索引在[0, size-1]范围内 */
    if (index >= size) {
        return (index - size);
    }

    if (index < 0) {
        return (index + size);
    }

    return index;
}

/* 对峰值点进行二次插值，计算亚单元偏移量delta */
static float udsf_peak_interpolation_core(const float data_map[][CONFIG_RAMP_RNGBIN], int peakIdx, int fixedIdx, uint8_t isVelocityDim, int dimSize)
{
    /* 如果维度长度不足3，无法插值，返回0偏移 */
    if (dimSize < 3) {
        return 0.0f;
    }

    /* 0基索引: 左右邻点的索引 */
    int currIdx = peakIdx;

    /* 获取三点幅度值 */
    float A_prev = 0, A_curr = 0, A_next = 0;

    if (isVelocityDim) {
        /* 速度维插值: data_map[速度][距离] */
        A_prev = data_map[udsf_get_circular_index(peakIdx - 1, dimSize)][fixedIdx];
        A_curr = data_map[currIdx][fixedIdx]; 
        A_next = data_map[udsf_get_circular_index(peakIdx + 1, dimSize)][fixedIdx];
    } else {
        if (peakIdx < 1) {
            A_curr = data_map[fixedIdx][currIdx];
            A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
            float del0 = A_next / (A_next + A_curr + DBL_EPSILON) * 0.5;
            return del0;    /* 0距离bin特殊处理 */
        }

        /* 距离维插值: data_map[速度][距离] */
        A_prev = data_map[fixedIdx][udsf_get_circular_index(peakIdx - 1, dimSize)];
        A_curr = data_map[fixedIdx][currIdx]; 
        A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
    }

    /* 二次插值计算偏移量delta */
    float delta = 0.0;
    float denominator = A_prev - 2 * A_curr + A_next;

    /* 避免除零或极小值 */
    if (fabs(denominator) > DBL_EPSILON) {
        delta = 0.5 * (A_prev - A_next) / (denominator == 0 ? 1.0 : denominator);
    }

    /* 限制偏移量范围[-0.5, 0.5]，防止异常插值 */
    if (delta > 0.5) delta = 0.5;
    if (delta < -0.5) delta = -0.5;

    return delta;
}

/* 对CFAR检测到的目标进行二次插值细化(亚单元估计) */
static void udsf_refine_detections_interpolation(const float data_map[][CONFIG_RAMP_RNGBIN], DetectResultType *detections, int count, int velDim, int rangeDim)
{
    if ((count <= 0) || (detections == NULL)) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        int range_index = detections[i].range_index;        /* 0基距离索引 */
        int velocity_index = detections[i].velocity_index;  /* 0基速度索引 */

        /* 速度维插值 */
        float delta_veloccity = udsf_peak_interpolation_core(data_map, velocity_index, range_index, 1, velDim);
        detections[i].velocity_fine = (float)velocity_index + delta_veloccity;

        /* 距离维插值 */
        float delta_range = udsf_peak_interpolation_core(data_map, range_index, velocity_index, 0, rangeDim);
        detections[i].range_fine = (float)range_index + delta_range;
    }
}

/* 核心二维CFAR检测，用于单个天线的功率图 */
static int udsf_cfar2d_core(const float power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static float s_cfar_noise = -1.0f;
    const int guard_vel = sCFARParam.guard_cell[0];
    const int guard_range = sCFARParam.guard_cell[1];
    const int ref_vel = sCFARParam.reference_cell[0];
    const int ref_range = sCFARParam.reference_cell[1];
    const float thresholdFactor = sCFARParam.threshold_factors;

    /* 初始化检测结果缓存 */
    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* 简化噪声估计 (用于峰值分组的快速排除) */
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

    /* 从0-based索引1到倒数第2个索引(避免边界检查的复杂性)。距离维从1开始，排除MATLAB逻辑中的第0个距离bin */
    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const float currentAmp = power_map[v_idx][r_idx];
            /* 快速排除 */
            if (currentAmp <= (CONFIG_CFAR_TH_OFFSET * noiseEstm)) {
                continue;
            }

            /* 峰值分组/局部最大值检查 */
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

            /* 提取参考单元数据 (循环移位处理边界) */
            float sum_up = 0.0, sum_down = 0.0;
            float sum_left = 0.0, sum_right = 0.0;

            /* 速度维(上下)参考单元求和 */
            for (int i = 0; i < ref_vel; i++) {
                /* 上侧参考单元(Up): velIdx - guard_vel - ref_vel + i */
                int index_up = v_idx - guard_vel - ref_vel + i;
                sum_up += power_map[udsf_get_circular_index(index_up, N)][r_idx];

                /* 下侧参考单元(Down): velIdx + guard_vel + 1 + i */
                int index_down = v_idx + guard_vel + 1 + i;
                sum_down += power_map[udsf_get_circular_index(index_down, N)][r_idx];
            }

            /* 距离维(左右)参考单元求和 */
            for (int i = 0; i < ref_range; i++) {
                /* 左侧参考单元(Left): rangeIdx - guard_range - ref_range + i */
                int index_left = r_idx - guard_range - ref_range + i;
                sum_left += power_map[v_idx][udsf_get_circular_index(index_left, M)];

                /* 右侧参考单元(Right): rangeIdx + guard_range + 1 + i */
                int index_right = r_idx + guard_range + 1 + i;
                sum_right += power_map[v_idx][udsf_get_circular_index(index_right, M)];
            }

            /* 合并参考单元并估计噪声(中位数估计) */
            float mean_up = sum_up / ref_vel;
            float mean_down = sum_down / ref_vel;
            float mean_left = sum_left / ref_range;
            float mean_right = sum_right / ref_range;
            float noiseEst = udsf_median_of_four(mean_up, mean_down, mean_left, mean_right);

            /* 检测判决与结果存储 */
            const float threshold = thresholdFactor * noiseEst;
            if ((currentAmp > threshold) && (dcount < CONFIG_OBJECT_MAX)) {
                /* 信噪比计算(20 * log10(幅度比)) */
                float snr = 0.0f;
                if (noiseEst > DBL_MIN) {   /* 使用 DBL_MIN 避免除以零或极小值 */
                    snr = 20.0 * log10(currentAmp / noiseEst);
                } else {
                    snr = 99.0f;
                }

                if ((int)snr != 99) {
                    /* 存储结果 */
                    sDetectResult[dcount].range_index = r_idx;           /* 0-based 距离索引 */
                    sDetectResult[dcount].velocity_index = v_idx;        /* 0-based 速度索引 */
                    sDetectResult[dcount].range_fine = (float)r_idx;     /* 新增: 初始化为粗糙索引 */
                    sDetectResult[dcount].velocity_fine = (float)v_idx;  /* 新增: 初始化为粗糙索引 */
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

    /* 初始化检测结果缓存 */
    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* 简化噪声估计 (用于峰值分组的快速排除) */
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

    /* 从0-based索引1到倒数第2个索引(避免边界检查的复杂性)。距离维从1开始，排除MATLAB逻辑中的第0个距离bin */
    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const float currentAmp = power_map[v_idx][r_idx];
            /* 快速排除 */
            if (currentAmp < threshold) {
                continue;
            }

            /* 峰值分组/局部最大值检查 */
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

            /* 检测判决与结果存储 */
            if (dcount < CONFIG_OBJECT_MAX) {
                /* 信噪比计算(20 * log10(幅度比)) */
                float snr = 0.0f;
                if (noiseEstm > DBL_MIN) {   /* 使用 DBL_MIN 避免除以零或极小值 */
                    snr = 20.0 * log10(currentAmp *noiseEstInv);
                } else {
                    snr = 99.0f;
                }

                if ((int)snr != 99) {
                    sDetectResult[dcount].range_index = r_idx;           /* 0-based 距离索引 */
                    sDetectResult[dcount].velocity_index = v_idx;        /* 0-based 速度索引 */
                    sDetectResult[dcount].range_fine = (float)r_idx;     /* 新增: 初始化为粗糙索引 */
                    sDetectResult[dcount].velocity_fine = (float)v_idx;  /* 新增: 初始化为粗糙索引 */
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
 * 函数名称: radar_execute_cfar
 * 功能描述: 对2DFFT结果进行CFAR目标检测和插值
 * 输入参数: fft2d_input -- 2DFFT数据
 * 输出参数: result      -- 用于返回详细检测列表(可选)
 * 返回说明: 成功返回检测目标数，失败返回0
 * 其他说明: 无
 */
float temp_mag[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN];
float avg_amplitude_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN];
float complex_buffer[CONFIG_RAMP_CHIRPS * CONFIG_RAMP_RNGBIN * 2];
int radar_execute_cfar(const void *fft2d_input, DetectResultType **result)
{
    /* 检查参数合理性 */
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

    /* 1.创建并计算"平均幅度图"(ncidata) */
    memset(temp_mag, 0x00, sizeof(temp_mag));
    memset(avg_amplitude_map, 0x00, sizeof(avg_amplitude_map));

    /* a.累加所有天线的幅度 */
    for (uint8_t ant = 0; ant < ants; ++ant) {
        memset(complex_buffer, 0x00, sizeof(complex_buffer));
        for (int i = 0; i < (chirps * bins); i++) {
            uint8_t rbin = i % bins;
            uint16_t chirp = i / bins;
            complex_buffer[i * 2 + 0] = (float)((*fft2dFrame)[ant][rbin][chirp][1]);   /* 实部 */
            complex_buffer[i * 2 + 1] = (float)((*fft2dFrame)[ant][rbin][chirp][0]);   /* 虚部 */
        }

        arm_cmplx_mag_f32(complex_buffer, (float32_t *)temp_mag[0], chirps * bins);
        for (uint16_t chirp = 0; chirp < chirps; ++chirp) {
            for (uint8_t rbin = 0; rbin < bins; ++rbin) {
                avg_amplitude_map[chirp][rbin] += temp_mag[chirp][rbin];
            }
        }
    }

    /* 2.在"平均幅度图"上执行核心CFAR检测 */
    if (CONFIG_CFAR_TYPE == 1) {
        detect_count = udsf_cfar2d_core_cut(avg_amplitude_map, chirps, bins);
    } else {
        detect_count = udsf_cfar2d_core(avg_amplitude_map, chirps, bins);
    }

    /* 3.将检测结果应用于所有天线的二值图(如果需要) */
    if (detect_count > 0) {
        /* 对检测结果进行细化 */
        udsf_refine_detections_interpolation(avg_amplitude_map, sDetectResult, detect_count, chirps, bins);
        if (result) {
            *result = sDetectResult;
        }
    }

    /* 返回检测到的目标数量 */
    return detect_count;
}

#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)

/* 全局检测结果缓存 */
DetectResultType sDetectResult[CONFIG_OBJECT_MAX];

/* CFAR配置参数 */
static CfarConfigType sCFARParam = {
    .range_dim         = CONFIG_RAMP_RNGBIN,
    .velocity_dim      = CONFIG_RAMP_CHIRPS,
    .guard_cell        = {CONFIG_CFAR_NUM_GUARD_VEL, CONFIG_CFAR_NUM_GUARD_RANGE},
    .reference_cell    = {CONFIG_CFAR_NUM_TRAIN_VEL, CONFIG_CFAR_NUM_TRAIN_RANGE},
    .threshold_factors = CONFIG_CFAR_TH_AMP_Q4,
};

/******************** 快速定点辅助函数 ********************/

/* ========================================================
   1. 高精度纯整型对数函数 & SNR 计算 
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
    /* 如果底噪为0，直接返回系统最大支持信噪比 99dB (99 * 256 = 25344) */
    if (noise == 0 || amp == 0) return 25344;
    
    uint32_t log2_amp = udsf_log2_q16(amp);
    uint32_t log2_noise = udsf_log2_q16(noise);
    
    /* 差值必然 >= 0，因为能进到这里必然 amp > threshold > noise */
    int32_t diff_q16 = (int32_t)log2_amp - (int32_t)log2_noise;
    if (diff_q16 < 0) diff_q16 = 0;
    
    /* 修复溢出Bug：强制升级为 64 位乘法，防止信号极差过大导致 32 位乘法变成负数 */
    int32_t snr_q8 = (int32_t)(((int64_t)diff_q16 * 1541) >> 16);
    
    /* 封顶钳位，防止超出 int16_t 的安全范围 */
    if (snr_q8 > 25344) snr_q8 = 25344; 
    
    return (int16_t)snr_q8;
}

/* 极速求取4个整型的中位数(替代原先耗时的qsort) */
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

/* 求取4个数的最大值 */
static inline uint32_t udsf_max_of_four_u32(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    uint32_t m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    if (d > m) m = d;
    return m;
}

/* 快速定点Log2估算（返回Q8格式，例如 log2(2) = 256），替代原先浮点20*log10 */
static int16_t fast_log2_q8(uint32_t x)
{
    if (x == 0) return 0;
    uint16_t int_part = 0;
    uint32_t temp = x;
    while (temp > 1) {
        temp >>= 1;
        int_part++;
    }
    /* 提取小数部分(最高位之后的8位)并线性插值 */
    uint16_t frac_part = 0;
    if (int_part >= 8) {
        frac_part = (x >> (int_part - 8)) & 0xFF;
    } else {
        frac_part = (x << (8 - int_part)) & 0xFF;
    }
    return (int_part << 8) | frac_part;
}

/* 处理循环移位边界 */
static inline int udsf_get_circular_index(int index, int size)
{
    if (index >= size) return (index - size);
    if (index < 0) return (index + size);
    return index;
}

/* ========================================================
   1. 对峰值点进行二次插值，计算亚单元偏移量 delta
   返回值为 Q8 格式的定点小数 (128 代表 0.5， -128 代表 -0.5)
   ======================================================== */
static int16_t udsf_peak_interpolation_core(const uint32_t data_map[][CONFIG_RAMP_RNGBIN], int peakIdx, int fixedIdx, uint8_t isVelocityDim, int dimSize)
{
    /* 如果维度长度不足 3，无法插值，返回 0 偏移 */
    if (dimSize < 3) {
        return 0;
    }

    /* 使用 32 位无符号整型接收放大 16 倍的幅度值 */
    uint32_t A_prev = 0, A_curr = 0, A_next = 0;
    int currIdx = peakIdx;

    if (isVelocityDim) {
        /* 速度维插值: data_map[速度][距离] */
        A_prev = data_map[udsf_get_circular_index(peakIdx - 1, dimSize)][fixedIdx];
        A_curr = data_map[currIdx][fixedIdx]; 
        A_next = data_map[udsf_get_circular_index(peakIdx + 1, dimSize)][fixedIdx];
    } else {
        if (peakIdx < 1) {
            /* 0 距离 bin 的特殊边界处理 
             * 浮点原逻辑: delta = 0.5 * A_next / (A_next + A_curr) 
             */
            A_curr = data_map[fixedIdx][currIdx];
            A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
            
            uint32_t den_spec = A_next + A_curr;
            if (den_spec != 0) {
                /* 乘以 128 就等价于乘以 0.5，并将其转换为 Q8 格式的定点小数 */
                return (int16_t)((A_next * 128) / den_spec); 
            }
            return 0;
        }

        /* 距离维常规插值: data_map[速度][距离] */
        A_prev = data_map[fixedIdx][udsf_get_circular_index(peakIdx - 1, dimSize)];
        A_curr = data_map[fixedIdx][currIdx]; 
        A_next = data_map[fixedIdx][udsf_get_circular_index(peakIdx + 1, dimSize)];
    }

    /* 二次插值计算偏移量 delta 
     * 浮点公式: delta = 0.5 * (A_prev - A_next) / (A_prev - 2*A_curr + A_next)
     */
     
    /* 转为有符号 32 位进行差值运算，防止减法溢出变巨量正数 */
    int32_t num = (int32_t)A_prev - (int32_t)A_next;
    int32_t den = (int32_t)A_prev - 2 * (int32_t)A_curr + (int32_t)A_next;
    
    int16_t delta_q8 = 0;

    /* 避免除零，若分母不为 0 则计算 */
    if (den != 0) {
        /* (num * 128) / den 等价于: (num / den) * 0.5 * 256 (256是Q8的底) */
        delta_q8 = (int16_t)((num * 128) / den); 
    }

    /* 限制偏移量范围在 [-0.5, 0.5] 之间，对应 Q8 格式的 [-128, 128]，防止异常插值 */
    if (delta_q8 > 128) delta_q8 = 128;
    if (delta_q8 < -128) delta_q8 = -128;

    return delta_q8;
}

/* ========================================================
   对 CFAR 检测到的目标进行二次插值细化(亚单元估计)
   ======================================================== */
static void udsf_refine_detections_interpolation(const uint32_t data_map[][CONFIG_RAMP_RNGBIN], DetectResultType *detections, int count, int velDim, int rangeDim)
{
    if ((count <= 0) || (detections == NULL)) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        /* 获取 0-based 的粗糙索引 */
        int range_index = detections[i].range_index;        
        int velocity_index = detections[i].velocity_index;  

        /* --- 速度维细化 --- */
        int16_t delta_velocity = udsf_peak_interpolation_core(data_map, velocity_index, range_index, 1, velDim);
        
        /* 拼接为 Q8 定点数：(整数部分左移 8 位) + 小数偏移量 */
        detections[i].velocity_fine = (int16_t)((velocity_index << 8) + delta_velocity);

        /* --- 距离维细化 --- */
        int16_t delta_range = udsf_peak_interpolation_core(data_map, range_index, velocity_index, 0, rangeDim);
        
        /* 拼接为 Q8 定点数：(整数部分左移 8 位) + 小数偏移量 */
        detections[i].range_fine = (int16_t)((range_index << 8) + delta_range);
    }
}

/* ========================================================
   带有高精度 SNR 输出的 CFAR 核心函数
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
            
            
            /* 1. 计算用于阈值判决的普通噪声 (16倍精度) */
            uint32_t mean_up = (sum_up + (ref_vel >> 1)) / ref_vel;
            uint32_t mean_down = (sum_down + (ref_vel >> 1)) / ref_vel;
            uint32_t mean_left = (sum_left + (ref_range >> 1)) / ref_range;
            uint32_t mean_right = (sum_right + (ref_range >> 1)) / ref_range;
            uint32_t noiseEst = udsf_median_of_four_u32(mean_up, mean_down, mean_left, mean_right);
            uint32_t threshold = (noiseEst * thresholdFactor_Q4) >> 4;
            if ((currentAmp > threshold) && (dcount < CONFIG_OBJECT_MAX)) {
                
                /* 2. 计算用于 SNR 的高精度噪声 (Q8 - 256倍精度) */
                uint32_t hp_mean_up = ((sum_up << 4) + (ref_vel >> 1)) / ref_vel;
                uint32_t hp_mean_down = ((sum_down << 4) + (ref_vel >> 1)) / ref_vel;
                uint32_t hp_mean_left = ((sum_left << 4) + (ref_range >> 1)) / ref_range;
                uint32_t hp_mean_right = ((sum_right << 4) + (ref_range >> 1)) / ref_range;
                uint32_t hp_noiseEst = udsf_median_of_four_u32(hp_mean_up, hp_mean_down, hp_mean_left, hp_mean_right);

                /* 3. 计算 SNR：确保分子也是 Q8 (256倍精度)
                   将 16倍的 currentAmp 左移 4位，变成 256倍 */
                int16_t snr_q8 = udsf_exact_snr_q8(currentAmp << 4, hp_noiseEst);

                /* 4. 拦截无效目标 */
                if (snr_q8 != 25344) {
                    sDetectResult[dcount].range_index = r_idx;           
                    sDetectResult[dcount].velocity_index = v_idx;        
                    sDetectResult[dcount].range_fine = r_idx << 8;       
                    sDetectResult[dcount].velocity_fine = v_idx << 8;    
                    
                    sDetectResult[dcount].snr = snr_q8; // Q8 格式

                    /* 5. 存储高精度物理量 (全部存储为 Q8 格式) */
                    sDetectResult[dcount].amplitude = (uint32_t)(currentAmp << 4); // 16x16 = 256(Q8)
                    sDetectResult[dcount].noise = (uint32_t)hp_noiseEst;           // 已经是 256(Q8)
                    
                    dcount++;
                }
            }
        }
    }
    return dcount;
}

/**
 * 函数名称: udsf_cfar2d_core_cut
 * 功能描述: 简化版二维 CFAR 检测 (基于全局/区域底噪)
 * 输入参数: power_map -- 底层幅度功率图 (32位，16倍精度)
 *           N         -- 速度维大小
 *           M         -- 距离维大小
 * 返回值:   检测到的目标数量
 */
static int udsf_cfar2d_core_cut(const uint32_t power_map[CONFIG_RAMP_CHIRPS][CONFIG_RAMP_RNGBIN], const int N, const int M)
{
    int dcount = 0;
    static uint32_t s_cfar_noise = 0;       
    static uint8_t s_noise_init = 0;

    memset(sDetectResult, 0x00, sizeof(sDetectResult));

    /* 1. 全局噪声估算 (16倍精度) */
    uint32_t noise_sum = 0;
    for (int i = N - 6; i < N; i++) {
        for (int j = M - 6; j < M; j++) {
            noise_sum += power_map[udsf_get_circular_index(i, N)][udsf_get_circular_index(j, M)];
        }
    }
    uint32_t noiseEstm_16x = (noise_sum + 18) / 36;
    
    /* 2. IIR 更新 (修正后的有符号运算) */
    if (s_noise_init) {
        int32_t diff = (int32_t)noiseEstm_16x - (int32_t)s_cfar_noise;
        int32_t update = (diff * (int32_t)CONFIG_CFAR_NCOEF_Q15) >> 15;
        noiseEstm_16x = (uint32_t)((int32_t)s_cfar_noise + update);
        s_cfar_noise = noiseEstm_16x;
    } else {
        s_cfar_noise = noiseEstm_16x;
        s_noise_init = 1;
    }

    /* 3. 判定门限 (确保 CONFIG_CFAR_TH_OFFSET_Q4 与 core 版本的 threshold_factors 一致) */
    uint32_t threshold_16x = (CONFIG_CFAR_TH_OFFSET_Q4 * noiseEstm_16x) >> 4;

    for (int r_idx = CONFIG_CFAR_RBIN_START_IDX; r_idx < M - 1; r_idx++) {
        for (int v_idx = 1; v_idx < N - 1; v_idx++) {
            const uint32_t currentAmp_16x = power_map[v_idx][r_idx];
            
            if (currentAmp_16x < threshold_16x) continue;

            /* 4. 局部峰值校验 */
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
                /* 5. 计算 SNR：分子和分母都左移 4 位，对齐到 Q8 (256倍) */
                int16_t snr_q8 = udsf_exact_snr_q8(currentAmp_16x << 4, noiseEstm_16x << 4);

                if (snr_q8 != 25344) {
                    sDetectResult[dcount].range_index = r_idx;           
                    sDetectResult[dcount].velocity_index = v_idx;        
                    sDetectResult[dcount].range_fine = (int16_t)(r_idx << 8); 
                    sDetectResult[dcount].velocity_fine = (int16_t)(v_idx << 8);    
                    sDetectResult[dcount].snr = snr_q8;
                    
                    /* 存储高精度物理量 (Q8) */
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
   极速纯整型平方根函数 (无浮点，高精度)
   ======================================================== */
static inline uint32_t udsf_fast_isqrt(uint32_t val) 
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; /* 从最高偶数位开始 */
    
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
   雷达 CFAR 主执行函数
   ======================================================== */
int radar_execute_cfar(const void *fft2d_input, DetectResultType **result)
{
    /* 检查参数合理性 */
    if ((sCFARParam.guard_cell[0] < 0) || (sCFARParam.guard_cell[1] < 0)
        || (sCFARParam.reference_cell[0] <= 0) || (sCFARParam.reference_cell[1] <= 0))
    {
        return 0;
    }

    int detect_count = 0;
    const uint8_t ants = CONFIG_RAMP_RXNUM;
    const uint8_t bins = CONFIG_RAMP_RNGBIN;
    const uint16_t chirps = CONFIG_RAMP_CHIRPS;
    
    /* 严格基于 16位原输入解析 */
    const int16_t (*fft2dFrame)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2] = 
        (const int16_t (*)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2])fft2d_input;

    /* 清空 32位 底层累加图 */
    memset(avg_amplitude_map, 0x00, sizeof(avg_amplitude_map));

    /* 1. 提取信号并计算精确的高精度物理模值 */
    for (uint8_t ant = 0; ant < ants; ++ant) {
        for (uint16_t chirp = 0; chirp < chirps; ++chirp) {
            for (uint8_t rbin = 0; rbin < bins; ++rbin) {
                /* 提取 Q (实部/虚部) 和 I (虚部/实部) */
                int16_t q = (*fft2dFrame)[ant][rbin][chirp][0]; 
                int16_t i = (*fft2dFrame)[ant][rbin][chirp][1]; 
                
                /* 转为绝对值 */
                uint32_t abs_i = (i > 0) ? i : -i;
                uint32_t abs_q = (q > 0) ? q : -q;
                
                /* 计算 I^2 + Q^2 (由于 abs_i/q 最大 32767，平方和最大约 21.4亿，完美适配 32 位无符号数) */
                uint32_t sum_sq = (abs_i * abs_i) + (abs_q * abs_q);
                uint32_t mag_scaled_by_16;
                
                /* 智能动态精度保留算法：
                 * 1. 若信号较小 (sum_sq < 2^24)，面积先放大 256 倍(<<8)，再开方。开方后刚好就是真实的幅度放大 16 倍，完美保留小数。
                 * 2. 若信号极大，直接开方求出真实幅度，然后再外部放大 16 倍(<<4)，防止位移溢出 32 位极限。
                 */
                if (sum_sq < 0x01000000) { 
                    mag_scaled_by_16 = udsf_fast_isqrt(sum_sq << 8);
                } else {
                    mag_scaled_by_16 = udsf_fast_isqrt(sum_sq) << 4;
                }
                
                /* 将该天线的该点幅度累加到平均幅度图中 */
                avg_amplitude_map[chirp][rbin] += mag_scaled_by_16;
            }
        }
    }

    /* 2. 在 "平均幅度图" 上执行核心 CFAR 检测 */
    if (CONFIG_CFAR_TYPE == 1) {
        detect_count = udsf_cfar2d_core_cut(avg_amplitude_map, chirps, bins);
    } else {
        detect_count = udsf_cfar2d_core(avg_amplitude_map, chirps, bins);
    }

    /* 3. 对检测到的目标执行亚单元(细粒度)二次插值 */
    if (detect_count > 0) {
        udsf_refine_detections_interpolation(avg_amplitude_map, sDetectResult, detect_count, chirps, bins);
        if (result) {
            *result = sDetectResult;
        }
    }

    /* 返回检测到的目标数量 */
    return detect_count;
}

#endif