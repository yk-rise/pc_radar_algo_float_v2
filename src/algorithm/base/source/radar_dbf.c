#include "radar_dbf.h"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)

/* 目标点缓存 */
static TargetPointType sTargetPoints[CONFIG_OBJECT_MAX];

/**
 * 计算2天线相位差θ值(返回角度制)
 * 参数：
 * I1, Q1: 第一个天线复数的实部和虚部(float类型)
 * I2, Q2: 第二个天线复数的实部和虚部(float类型)
 * 返回值：
 * 计算得到的θ值(角度制，范围[-90, 90]度)
 */
float udsf_calculate_theta(float I1, float Q1, float I2, float Q2)
{
    float arg1 = I1 * Q2 - I2 * Q1;
    float arg2 = I1 * I2 + Q1 * Q2;

    /* 直接在计算中融入弧度转角度的转换，步骤更紧凑 */
    /* 使用asinf(atan2f(arg1, arg2) / M_PI) * (180.0f / M_PI) */

    float32_t result = 0.0f;
    if (arm_atan2_f32(arg1, arg2, &result) == ARM_MATH_SUCCESS) {
        return asinf(result / M_PI) * (180.0f / M_PI);
    }

    return result;
}

/**
 * 函数名称: radar_dbf_estimation
 * 功能描述: 测量目标点角度
 * 输入参数: fft2d_input   -- 2DFFT数据
 *           result        -- CFAR配置参数
 *           cfar_points   -- CFAR输出点数
 * 输出参数: points_output -- 目标信息列表(不为空的话，需要调用方释放内存)
 * 返回说明: 返回目标数量
 * 其他说明: 无
 */
int radar_dbf_estimation(const void *fft2d_input, const DetectResultType *result, int cfar_points, TargetPointType **points_output)
{
    /* 检查输入和初始化扫描向量 */
    if ((cfar_points <= 0) || (result == NULL)) {
        return 0;
    }

    int target_count = 0;
    const uint8_t ants = CONFIG_RAMP_RXNUM;
    const uint16_t chirps = CONFIG_RAMP_CHIRPS;
    const float rRes = RADAR_RANGE_RESOLUTION;
    const float vRes = RADAR_VELOCITY_RESOLUTION;
    const int16_t (*fft2dFrame)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2] = (const int16_t (*)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2])fft2d_input;

    if (ants != 2) {
        return 0;
    }

    memset(sTargetPoints, 0x00, sizeof(sTargetPoints));
    cfar_points = cfar_points > CONFIG_OBJECT_MAX ? CONFIG_OBJECT_MAX : cfar_points;

    for (int i = 0; i < cfar_points; ++i) {
        const DetectResultType *detect = (const DetectResultType *)&result[i];

        /* 提取插值后的索引 */
        int rangeIdx = detect->range_index;
        int velocityIdx = detect->velocity_index;
        float rangeFine = detect->range_fine - 0;
        float velocityFine = detect->velocity_fine;

        /* 提取信号向量(两个天线上同一(R, V) 单元的复数数据) */
        float I1 = (*fft2dFrame)[0][rangeIdx][velocityIdx][1];      /* ant1实部 */
        float Q1 = (*fft2dFrame)[0][rangeIdx][velocityIdx][0];      /* ant1虚部 */
        float I2 = (*fft2dFrame)[1][rangeIdx][velocityIdx][1];      /* ant2实部 */
        float Q2 = (*fft2dFrame)[1][rangeIdx][velocityIdx][0];      /* ant2虚部 */

        /* 执行相位差测角 */
        float a = udsf_calculate_theta(I1, Q1, I2, Q2);

        /* 计算物理R, V, P */
        float r = rangeFine * rRes;                                 /* 距离 */
        float v = (velocityFine - (float)chirps * 0.5f) * vRes;     /* 速度 */
        float power_db = 20.0f * log10(detect->amplitude);          /* 目标幅度功率(dB) */

        /* 存储到TargetPointType结构体 */
        if (r > 0) {
            sTargetPoints[target_count].angle     = a;
            sTargetPoints[target_count].range     = r;
            sTargetPoints[target_count].velocity  = v;
            sTargetPoints[target_count].snr       = detect->snr;
            sTargetPoints[target_count].powerdb   = power_db;
            sTargetPoints[target_count].amplitude = detect->amplitude;
            target_count++;
        }
    }

    if (points_output) {
        *points_output = sTargetPoints;
    }

    return target_count;
}

#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)

static TargetPointType sTargetPoints[CONFIG_OBJECT_MAX];

/* ========================================================
   局部辅助函数：对数计算（用于求解功率 dB）
   ======================================================== */
static uint32_t udsf_dbf_log2_q16(uint32_t x) 
{
    if (x == 0) return 0;
    uint32_t n = 0;
    uint32_t temp = x;
    while (temp > 1) { temp >>= 1; n++; }
    
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

/* ========================================================
   局部辅助函数：CORDIC 极速反正切算法 (16次迭代满精度版)
   ======================================================== */
static int32_t udsf_atan2_cordic_q15(int32_t y, int32_t x) 
{
    if (x == 0 && y == 0) return 0;
    
    int32_t angle = 0;
    if (x < 0) {
        x = -x;
        y = -y;
        angle = 32768; /* Q15 格式下的 PI */
    }
    
    int32_t cur_x, cur_y;
    
    /* 16 次迭代的角度表: atan(2^-i) / PI * 32768 */
    static const uint16_t atan_table[16] = {
        8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1, 0
    };
    
    for (int i = 0; i < 16; i++) {
        if (y > 0) {
            cur_x = x + (y >> i);
            cur_y = y - (x >> i);
            angle += atan_table[i];
        } else {
            cur_x = x - (y >> i);
            cur_y = y + (x >> i);
            angle -= atan_table[i];
        }
        x = cur_x;
        y = cur_y;
    }
    
    if (angle > 32768) angle -= 65536;
    return angle;
}

/* ========================================================
   核心算角函数：极高精度 65点 LUT 定点测角
   ======================================================== */
static int16_t udsf_calculate_theta_q8(int16_t I1, int16_t Q1, int16_t I2, int16_t Q2) 
{
    /* 1. 计算交叉乘积，使用 64 位防溢出 */
    int64_t arg1_64 = (int64_t)I1 * Q2 - (int64_t)I2 * Q1;
    int64_t arg2_64 = (int64_t)I1 * I2 + (int64_t)Q1 * Q2;

    if (arg1_64 == 0 && arg2_64 == 0) return 0;

    /* 2. 获取最大绝对值，用于安全动态拉升（绝对安全，无位移UB）*/
    uint64_t abs_arg1 = arg1_64 > 0 ? arg1_64 : -arg1_64;
    uint64_t abs_arg2 = arg2_64 > 0 ? arg2_64 : -arg2_64;
    uint64_t max_arg = abs_arg1 > abs_arg2 ? abs_arg1 : abs_arg2;

    if (max_arg > 0) {
        /* 拉升至 2^28 附近，让 CORDIC 达到极限计算精度 */
        while (max_arg > 536870912ULL) {
            arg1_64 /= 2;
            arg2_64 /= 2;
            max_arg >>= 1;
        }
        while (max_arg < 268435456ULL) {
            arg1_64 *= 2;
            arg2_64 *= 2;
            max_arg <<= 1;
        }
    }

    /* 3. 获取高精度相位差，Q15 格式 (32768 = PI) */
    int32_t phase = udsf_atan2_cordic_q15((int32_t)arg1_64, (int32_t)arg2_64);
    
    int32_t abs_phase = phase > 0 ? phase : -phase;
    if (abs_phase > 32768) abs_phase = 32768; 
    
    /* 4. 终极精准 65 点反正弦查找表 (Q8 格式: arcsin(i/64) * 180/PI * 256) 
          通过高密度切分，彻底消除了线性插值的弓形误差 */
    static const uint16_t asin_lut[65] = {
        0, 229, 458, 688, 917, 1147, 1378, 1608,
        1838, 2069, 2301, 2533, 2766, 2999, 3233, 3469,
        3706, 3943, 4183, 4422, 4663, 4905, 5148, 5393,
        5639, 5887, 6137, 6389, 6643, 6898, 7156, 7416,
        7680, 7946, 8215, 8487, 8763, 9042, 9325, 9613,
        9904, 10200, 10500, 10806, 11117, 11436, 11762, 12095,
        12437, 12789, 13150, 13523, 13908, 14308, 14723, 15157,
        15611, 16089, 16598, 17141, 17728, 18372, 19095, 19946,
        23040
    };
    
    /* 插值计算：区间缩小了一倍，右移位数由 10 变为 9 (32768/64 = 512 = 2^9) */
    int32_t idx = abs_phase >> 9;
    int32_t frac = abs_phase & 511;
    int32_t angle_q8 = 0;
    
    if (idx >= 64) {
        angle_q8 = 23040;  /* 90 度对应 Q8: 90 * 256 = 23040 */
    } else {
        int32_t y0 = asin_lut[idx];
        int32_t y1 = asin_lut[idx + 1];
        /* 极其精密的线性插值 */
        angle_q8 = y0 + (((y1 - y0) * frac) >> 9);
    }
    
    return (int16_t)(phase > 0 ? angle_q8 : -angle_q8);
}

/* ========================================================
   主函数：波束成形角估计入口
   ======================================================== */
int radar_dbf_estimation(const void *fft2d_input, const DetectResultType *result, int cfar_points, TargetPointType **points_output)
{
    /* 检查输入和初始化扫描向量 */
    if ((cfar_points <= 0) || (result == NULL)) {
        return 0;
    }

    int target_count = 0;
    const uint8_t ants = CONFIG_RAMP_RXNUM;
    const uint16_t chirps = CONFIG_RAMP_CHIRPS;
    
    const int16_t (*fft2dFrame)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2] = 
        (const int16_t (*)[CONFIG_RAMP_RXNUM][CONFIG_RAMP_RNGBIN][CONFIG_RAMP_CHIRPS][2])fft2d_input;

    if (ants != 2) {
        return 0;
    }

    memset(sTargetPoints, 0x00, sizeof(sTargetPoints));
    cfar_points = cfar_points > CONFIG_OBJECT_MAX ? CONFIG_OBJECT_MAX : cfar_points;

    for (int i = 0; i < cfar_points; ++i) {
        const DetectResultType *detect = &result[i];

        /* 获取粗糙索引 */
        int rangeIdx = detect->range_index;
        int velocityIdx = detect->velocity_index;

        /* 获取 Q8 格式的精细坐标 (承接 CFAR 模块的结果) */
        int16_t rangeFine_Q8 = detect->range_fine;
        int16_t velocityFine_Q8 = detect->velocity_fine;

        /* 从 16位 FFT 图中提取双天线 I/Q 数据 */
        int16_t I1 = (*fft2dFrame)[0][rangeIdx][velocityIdx][1];
        int16_t Q1 = (*fft2dFrame)[0][rangeIdx][velocityIdx][0];
        int16_t I2 = (*fft2dFrame)[1][rangeIdx][velocityIdx][1];
        int16_t Q2 = (*fft2dFrame)[1][rangeIdx][velocityIdx][0];

        /* 执行纯定点的高精度测角 */
        int16_t angle_q8 = udsf_calculate_theta_q8(I1, Q1, I2, Q2);

        /* 计算物理距离 Q8 (相乘再右移，完美降解回 Q8 格式且加入四舍五入防截断) */
        int32_t r_q8 = ((int32_t)rangeFine_Q8 * RADAR_RANGE_RES_Q16 + 32768) >> 16;

        /* 计算物理速度 Q8 (公式: (v_fine - chirps/2) * 0.125) 
         * 注意: chirps/2 转化为 Q8 是左移 7 位。乘以 0.125 即是除以 8 */
        int32_t v_temp = (int32_t)velocityFine_Q8 - (chirps << 7);
        int32_t v_q8 = v_temp / 8;

        /* 根据对数公式: log2(A * 256) = log2(A) + 8 */
        uint32_t log2_amp_q16 = udsf_dbf_log2_q16(detect->amplitude);
        /* 减去 8 (即 8 << 16) 抵消掉 Q8 的 256 倍增益，还原到物理幅度的 log2 */
        int32_t log2_phys = (int32_t)log2_amp_q16 - (8 << 16);
        /* 计算 20 * log10(A) */
        int16_t powerdb_q8 = (int16_t)(((int64_t)log2_phys * 1541) >> 16);

        /* 存储到全新的定点结构体 */
        if (r_q8 > 0) {
            sTargetPoints[target_count].angle     = angle_q8;
            sTargetPoints[target_count].range     = (int16_t)r_q8;
            sTargetPoints[target_count].velocity  = (int16_t)v_q8;
            sTargetPoints[target_count].snr       = detect->snr;
            sTargetPoints[target_count].powerdb   = powerdb_q8;
            sTargetPoints[target_count].amplitude = detect->amplitude;
            target_count++;
        }
    }

    if (points_output) {
        *points_output = sTargetPoints;
    }

    return target_count;
}
#endif