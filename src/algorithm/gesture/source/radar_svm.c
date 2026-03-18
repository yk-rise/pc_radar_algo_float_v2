/**
 * 鏂囦欢鍚嶇О: radar_svm.c
 * 鍔熻兘鎻忚堪: 闆疯揪鏁版嵁澶勭悊鏍稿績妯″潡锛屽寘鍚暟鎹澶勭悊銆佹湁鏁堝潡妫€娴嬨€佺壒寰佹彁鍙栫瓑鍔熻兘
 *           涓昏瀹炵幇闆疯揪鍘熷鏁版嵁鐨勬护娉€佹棤鏁堝€煎鐞嗗強鍏抽敭鏁版嵁鍧楃殑璇嗗埆锛屽熀浜嶴VM瀹炵幇闆疯揪鎵嬪娍璇嗗埆鍔熻兘
 * 
 * 鐗堟潈澹版槑: Copyright (C) [2025-2099] [瑗跨數骞跨爺闄?涓版捣绉戞妧/寮犲▉]
 *           鏈枃浠跺彈鐗堟潈淇濇姢锛屾湭缁忚鍙笉寰楁搮鑷鍒躲€佷慨鏀规垨鍒嗗彂
 * 
 * 寮€鍙戠幆澧? Windows 10 + GCC 9.2.0 / Linux Ubuntu 20.04 + GCC 9.4.0
 * 渚濊禆鏂囦欢: radar_svm.h, radar_svm_config.h, , radar_svm_pri.h, radar_svm_param.c
 * 
 * 鍘嗗彶淇敼璁板綍:
 * ----------------------------------------------------------------------
 * 鏃ユ湡        鐗堟湰  淇敼浜?    淇敼鍐呭
 * 2025-11-15  v1.0  ZhangWei  鍒濆浼樺寲鐗堟湰
 * ----------------------------------------------------------------------
 */
#include "radar_svm.h"
#include "pc_radar_debug.h"

#if defined(CONFIG_GESTURE_ENABLE)
#include "radar_svm_private.h"
#include "radar_svm_param.c"

#if (CONFIG_ALG_DTYPE == CONFIG_ALG_DFLOAT)
static RadarPoints g_ptFifo_base[RADAR_MAX_FIFOLEN] = {0};
static RadarPoints g_ptFifo_base2[RADAR_MAX_FIFOLEN] = {0};
static RadarPoints g_ptFifo_base3[RADAR_MAX_FIFOLEN] = {0};

/* 娴偣鏁版瘮杈冨ぇ灏?*/
static int udsf_cmp_float(const void *a, const void *b)
{
    float diff = *(float *)a - *(float *)b;
    if (diff < 0) {
        return -1;
    } else if (diff > 0) {
        return 1;
    }

    return 0;
}

/* 璁＄畻鍧囧€?*/
static float udsf_calc_mean(const float *arr, int n)
{
    if ((n <= 0) || (arr == NULL)) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }

    return sum / n;
}

/* 鍙栫粰瀹氬簭鍒椾腑鐨勪腑浣嶅€?*/
static float udsf_median_value(const float *arr, int n)
{
    if ((n <= 0) || (arr == NULL)) {
        return 0.0f;
    }

    if (n == 1) {
        return arr[0];
    }

    /* 鎷疯礉鏁扮粍锛屾敼鍙樺師鏁扮粍鏁版嵁 */
    float temp[RADAR_MAX_LEN];
    for (int i = 0; i < n; i++) {
        temp[i] = arr[i];
    }

    /* 蹇€熸帓搴?*/
    qsort(temp, n, sizeof(float), udsf_cmp_float);

    /* 鍋舵暟椤?*/
    if ((n % 2) == 0) {
        return (temp[n / 2 - 1] + temp[n / 2]) / 2.0f;
    }

    /* 濂囨暟椤?*/
    return temp[n / 2];
}

/* 鏍囧噯宸?*/
static float udsf_std_value(const float *arr, int n)
{
    if ((n <= 1) || (arr == NULL)) {
        return 0.0f;
    }

    float sum = 0.0f, mean = 0.0f, sum_sq = 0.0f;

    /* 姹傚钩鍧囧€?*/
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    mean = sum / n;

    /* 姹傚樊鐨勫钩鏂瑰拰(浼樺寲: 閬垮厤浣跨敤pow锛岀洿鎺ョ浉涔樻洿楂樻晥) */
    for (int i = 0; i < n; i++) {
        float diff = arr[i] - mean;
        /* 鏇夸唬pow(diff, 2)锛岀簿搴︽洿楂樹笖楂樻晥 */
        sum_sq += diff * diff;
    }

    /* 杩斿洖鏍锋湰鏍囧噯宸?涓嶮ATLAB std榛樿琛屼负涓€鑷? */
    return sqrt(sum_sq / (n - 1));      /* 闄ゆ暟鏀逛负n-1 */
}

/* 鏋佸樊 */
static float udsf_range_value(const float *arr, int n)
{
    if ((n <= 1) || (arr == NULL)) {
        return 0.0f;
    }

    /* 閬嶅巻鑾峰緱鏈€澶у€煎拰鏈€灏忓€?*/
    float max_v = arr[0], min_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) {
            max_v = arr[i];
        }

        if (arr[i] < min_v) {
            min_v = arr[i];
        }
    }

    /* 杩斿洖鏋佸樊 */
    return (max_v - min_v);
}

/* 鏂滅巼 */
static float udsf_slope_feature(const float *arr, int n)
{
    if ((n <= 1) || (arr == NULL)) {
        return 0.0f;
    }

    int max_i = 0, min_i = 0;
    float max_v = arr[0], min_v = arr[0];

    /* 閬嶅巻鑾峰緱鏈€澶у€煎拰鏈€灏忓€?*/
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) {
             max_v = arr[i];
             max_i = i;
        }

        if (arr[i] < min_v) {
            min_v = arr[i];
            min_i = i;
        }
    }

    /* 鍒ゆ柇闄ゆ暟鏄惁涓洪浂 */
    if (max_i == min_i) {
        return 0.0f;
    }

    /* 杩斿洖鏂滅巼 */
    return (max_v - min_v) / (max_i - min_i) * n;
}

static float udsf_calcFeaExtend_sign(const float *data, int len)
{
    if (len <= 1) {
        return (len == 1) ? data[0] : 0.0f;
    }

    /* 璁＄畻鍧囧€煎苟涓績鍖?*/
    float mean = udsf_calc_mean(data, len);
    float centered[RADAR_MAX_LEN];
    for (int i = 0; i < len; i++) {
        centered[i] = data[i] - mean;
    }

    /* 璁＄畻鏍囧噯宸苟鏍囧噯鍖?*/
    float std_val = udsf_std_value(centered, len);
    float normalized[RADAR_MAX_LEN];
    for (int i = 0; i < len; i++) {
        normalized[i] = centered[i] / (std_val + 1e-6f);
    }

    /* 璁＄畻n鍊?*/
    int n = (len + 1) / 2;

    /* 璁＄畻鍓峮涓厓绱犲拰涓庡悗n涓厓绱犲拰鐨勫樊鍊?*/
    float sum_first = 0.0f;
    for (int i = 0; i < n; i++) {
        sum_first += normalized[i];
    }

    float sum_second = 0.0f;
    for (int i = n; i < len; i++) {
        sum_second += normalized[i];
    }

    /* 璁＄畻鏈€缁堢粨鏋?*/
    return (sum_first - sum_second) / len;
}

/* 鏈€灏忓€?*/
static float udsf_min_value(const float *arr, int n)
{
    if ((n < 1) || (arr == NULL)) {
        return 0.0f;
    }

    /* 閬嶅巻鑾峰緱鏈€澶у€煎拰鏈€灏忓€?*/
    float min_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] < min_v) {
            min_v = arr[i];
        }
    }

    return min_v;
}

/* 鏈€澶у€?*/
static float udsf_max_value(const float *arr, int n)
{
    if ((n < 1) || (arr == NULL)) {
        return 0.0f;
    }

    /* 閬嶅巻鑾峰緱鏈€澶у€煎拰鏈€灏忓€?*/
    float max_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) {
            max_v = arr[i];
        }
    }

    return max_v;
}

/* 缁濆鍊兼渶澶у€?*/
static float udsf_absmax_value(const float *arr, int n)
{
    if ((n < 1) || (arr == NULL)) {
        return 0.0f;
    }

    /* 閬嶅巻鑾峰緱鏈€澶у€煎拰鏈€灏忓€?*/
    float max_v = fabs(arr[0]);
    for (int i = 1; i < n; i++) {
        float tmp = fabs(arr[i]);
        if (tmp > max_v) {
            max_v = tmp;
        }
    }

    return max_v;
}

/* 楂樻柉褰掍竴鍖?*/
static int udsf_gaus_normalization(float *feature)
{
    if (!feature) {
        return ALG_NULL_PTR;
    }

    for (int i = 0; i < FEAR_EXP_NUM; i++) {
        feature[i] = (feature[i] - g_mu[i]) / g_sigma[i];
    }

    return RADAR_ALG_SUCCESS;
}

/* 鎵撳垎 */
static int udsf_calc_svm_scores(float *features, float *decisions)
{
    if (!features || !decisions) {
        return ALG_NULL_PTR;
    }

    for (int d = 0; d < GR_CLASS_NUM; d++) {
        decisions[d] = 0.0f;
        for (int f = 0; f < FEAR_EXP_NUM; f++) {
            decisions[d] += g_svms_w[d][f] * features[f];
        }
        decisions[d] += g_svms_b[d];
    }

    return RADAR_ALG_SUCCESS;
}

/* 鏌ユ壘鏁扮粍涓殑鏈€澶у€肩储寮?*/
static int udsf_find_max_index(const float *scores, uint8_t length)
{
    if (length == 0) {
        return ALG_INVALID_LEN;
    }

    uint8_t maxIdx = 0;
    float maxVal = scores[0];

    for (uint8_t i = 1; i < length; i++) {
        if (scores[i] > maxVal) {
            maxIdx = i;
            maxVal = scores[i];
        }
    }

    return maxIdx;
}

/* 棰勬祴鍑芥暟: 鏍规嵁scores鍚戦噺鑾峰彇瀵瑰簲鐨勯娴嬬被鍒?*/
static int udsf_predict_classify(const float *scores, uint8_t scoresLength, int *classify)
{
    /* 绫诲埆鏁扮粍锛屽搴擬ATLAB涓殑classNames = [-1, 0, 1] */
    const int classNames[] = {GR_HAND_RIGHT, GR_HAND_INVLD, GR_HAND_LEFT};
    const uint8_t classCount = (uint8_t)(sizeof(classNames) / sizeof(classNames[0]));
    if (!scores || !classify) {
        return ALG_NULL_PTR;
    }

    /* 妫€鏌cores闀垮害鏄惁涓庣被鍒暟閲忓尮閰?*/
    if (scoresLength != classCount) {
        /* 闀垮害涓嶅尮閰嶉敊璇?*/
        return ALG_INVALID_LEN;
    }

    /* 鎵惧埌scores涓渶澶у€肩殑绱㈠紩 */
    int maxIdx = udsf_find_max_index(scores, scoresLength);
    if (maxIdx == RADAR_ALG_FAIL) {
        /* 鏃犳晥绱㈠紩閿欒 */
        return ALG_INVALID_LEN;
    }

    /* 杩斿洖瀵瑰簲鐨勭被鍒?*/
    *classify = classNames[maxIdx];
    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍚嶇О: udsf_block_invalid
 * 鍔熻兘鎻忚堪: 棰勭瓫閫夋棤鏁堟墜鍔垮潡锛岃閬夸笉杩炵画鎵嬪娍鍒ゅ喅缁撴灉杈撳嚭
 * 杈撳叆鍙傛暟: p    -- 杈撳叆鏁版嵁鏁扮粍
 *           ngap -- 鏁版嵁鍧椾腑鍏佽鐨勬渶澶ч棿闅旀暟
 * 杈撳嚭鍙傛暟: 鏃?
 * 杩斿洖璇存槑: 杩斿洖true琛ㄧず鏈夋晥锛宖alse琛ㄧず鏃犳晥
 */
static bool udsf_block_invalid(float *p, int ngap, int feaWinLen)
{
    if ((p == NULL) || (ngap >= (feaWinLen - 1)) || (ngap <= 0)) {
        return true;
    }

    bool pjsum = true;
    for (int i = 0; i < ngap; i++) {
        pjsum &= (p[i] < EPS);
    }

    return pjsum & (p[ngap] < EPS);
}

static bool udsf_block_full(float *p, int ngap)
{
    if ((p == NULL) || (ngap <= 0)) {
        return true;
    }

    bool pjsum = true;
    for (int i = 0; i < ngap; i++) {
        pjsum &= (p[i] < EPS);
    }

    return pjsum & (p[ngap] > EPS);
}

/**
 * 鍑芥暟鍚嶇О: udsf_struct_to_array
 * 鍔熻兘鎻忚堪: 缁撴瀯浣撹浆鏁扮粍
 * 杈撳叆鍙傛暟: pts -- 鏁版嵁鐐?
 *           len -- 鏁版嵁鐐规暟閲?
 * 杈撳嚭鍙傛暟: ts  -- 鏃堕棿搴忓垪
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_struct_to_array(const RadarPoints *pts, int len, TimeSeries *ts)
{
    if ((pts == NULL) || (ts == NULL)) {
        return ALG_NULL_PTR;
    }

    for (int i = 0; i < len; i++) {
        ts->r[i] = pts[i].r;
        ts->v[i] = pts[i].v;
        ts->a[i] = pts[i].a;
        ts->p[i] = pts[i].p;
    }
    ts->n = len;

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍔熻兘: 璁＄畻瑙掑害鏂瑰悜鍙樺寲娆℃暟(楠岃瘉鎵嬪娍鍛ㄦ湡鎬?
 * 杈撳叆鍙傛暟: angle_seq -- 瑙掑害搴忓垪鏁扮粍(float绫诲瀷)
 *           seq_len   -- 瑙掑害搴忓垪闀垮害
 * 杈撳嚭鍙傛暟: 鏂瑰悜鍙樺寲娆℃暟锛?
 *            - 鏈夋晥鎸ユ墜锛氳繑鍥?锛堝崟涓€鍛ㄦ湡锛?
 *            - 鏃犳晥鎵嬪娍锛氳繑鍥炩墵1鐨勫€硷紙闀垮害涓嶈冻3鏃惰繑鍥?锛?
 */
static int udsf_calc_dir_changes(const float *angle_seq, int seq_len)
{
    /* 鍒濆鍖栨柟鍚戝彉鍖栨鏁颁负0 */
    int dir_changes = 0;

    /* 搴忓垪闀垮害妫€鏌? 涓嶈冻3鐩存帴杩斿洖0 */
    if (seq_len < 3) {
        return dir_changes;
    }

    /* 璁＄畻绗﹀彿鍙樺寲 */
    for (int i = 1; i < seq_len; i++) {
        /* 鍒ゆ柇褰撳墠宸垎涓庡墠涓€涓樊鍒嗙殑涔樼Н鏄惁<0(绗﹀彿鐩稿弽=鏂瑰悜鍙樺寲) */
        if ((angle_seq[i] * angle_seq[i - 1]) < 0.0f) {
            dir_changes++;
        }
    }

    return dir_changes;
}

/**
 * 鍑芥暟鍔熻兘: 璁＄畻瑙掑害宸垎绗﹀彿姣斾緥(鍖哄垎宸﹀彸鎸ユ墜鏂瑰悜)
 * 杈撳叆鍙傛暟: angle_seq -- 瑙掑害搴忓垪鏁扮粍(float绫诲瀷)
 *           seq_len   -- 瑙掑害搴忓垪闀垮害
 * 杈撳嚭鍙傛暟: 姝ｅ€兼瘮渚?[0,1]锛?
 *            - 鎺ヨ繎1锛氬乏鎸ユ墜(瑙掑害澧炲姞)
 *            - 鎺ヨ繎0锛氬彸鎸ユ墜(瑙掑害鍑忓皯)
 *            - 鎺ヨ繎0.5锛氭棤鏁堟墜鍔?
 */
static float udsf_calc_sign_prop(const float *angle_seq, int seq_len)
{
    const float INVAD_PAS = 0.5f;

    /* 杈圭晫妫€鏌? 搴忓垪闀垮害涓嶈冻3鏃惰繑鍥為粯璁ゅ€?*/
    if (seq_len < 3) {
        return INVAD_PAS;
    }

    /* 璁＄畻涓€闃跺樊鍒嗗苟缁熻姝ｅ€兼暟閲?*/
    int pos_count = 0;
    int diff_len = seq_len - 1;  /* 涓€闃跺樊鍒嗛暱搴?= 鍘熷簭鍒楅暱搴?1 */

    for (int i = 0; i < diff_len; i++) {
        /* 涓€闃跺樊鍒?*/
        float diff = angle_seq[i+1] - angle_seq[i];
        if (diff > 0.0f) {  /* 缁熻姝ｅ€兼暟閲?*/
            pos_count++;
        }
    }

    /* 璁＄畻姝ｅ€兼瘮渚?娴偣闄ゆ硶锛屾樉寮忚浆鎹负float) */
    float prop_angle_sign = (float)pos_count / diff_len;
    return prop_angle_sign;
}

/**
 * 鍑芥暟鍔熻兘: 璁＄畻閫熷害灞€閮ㄥ嘲鍊兼暟閲?纭鎵嬪娍瑙勫緥鎬?
 * 杈撳叆鍙傛暟: vel_seq 閫熷害搴忓垪鏁扮粍(float绫诲瀷)
 *           seq_len 閫熷害搴忓垪闀垮害
 * 杈撳嚭鍙傛暟: 灞€閮ㄥ嘲鍊兼暟閲忥細
 *            - 鏈夋晥鎸ユ墜锛氳繑鍥?
 *            - 鏃犳晥鎵嬪娍锛氳繑鍥炩墵1鐨勫€?闀垮害涓嶈冻3鏃惰繑鍥?)
 */
static int udsf_calc_peaks_num(const float *vel_seq, int seq_len)
{
    int vel_peaks = 0;

    /* 搴忓垪闀垮害妫€鏌ワ細涓嶈冻3鐩存帴杩斿洖0 */
    if (seq_len < 3) {
        return vel_peaks;
    }

    /* 閬嶅巻涓棿鍏冪礌(k浠?鍒發ength-1锛屽搴擟鐨勭储寮?鍒皊eq_len-2) */
    for (int i = 1; i < (seq_len - 1); i++) {
        /* 鍒ゆ柇褰撳墠鍏冪礌鏄惁涓哄眬閮ㄥ嘲鍊?澶т簬宸﹀彸鐩搁偦鍏冪礌) */
        if ((vel_seq[i] > vel_seq[i - 1]) && (vel_seq[i] > vel_seq[i + 1])) {
            vel_peaks++;
        }
    }

    return vel_peaks;
}

/**
 * 鍑芥暟鍔熻兘: 璁＄畻楂楽NR鏍锋湰姣斾緥(杩囨护鍣０骞叉壈)
 * 杈撳叆鍙傛暟: snr_seq SNR搴忓垪鏁扮粍(float绫诲瀷)
 *           seq_len 鍘熷SNR搴忓垪闀垮害
 *           snr_threshold 杈撳叆鐨凷NR闃堝€?dB)
 * 杈撳嚭鍙傛暟: 楂楽NR鏍锋湰姣斾緥锛?
 *            - 鏈夋晥鎸ユ墜锛氳繑鍥?0.7
 *            - 鏃犳晥鎵嬪娍锛氳繑鍥?0.5
 *            - 鏍锋湰鏁颁笉瓒?鍏ㄤ负鏃犳晥鍊硷細杩斿洖0
 */
static float udsf_calc_snr_ratio(const float *snr_seq, int seq_len, float snr_threshold)
{
    float snr_ratio = 0.0f;
    int valid_count = seq_len;
    const float MIN_VALID_SAMPLES = 4.0f;

    /* 鏍锋湰鏁版鏌?*/
    if (valid_count < MIN_VALID_SAMPLES) {
        return 0.0f;
    }

    /* 璁＄畻楂楽NR姣斾緥 */
    int high_snr_count = 0;
    for (int i = 0; i < valid_count; i++) {
        if (snr_seq[i] > snr_threshold) {
            high_snr_count++;
        }
    }

    snr_ratio = (float)high_snr_count / valid_count;
    return snr_ratio;
}

/**
 * 鍑芥暟鍚嶇О: udsf_calc_feature
 * 鍔熻兘鎻忚堪: 璁＄畻鐗瑰緛
 * 杈撳叆鍙傛暟: ts      -- 鏃堕棿搴忓垪
 * 杈撳嚭鍙傛暟: feature -- 鐗瑰緛淇℃伅
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_calc_feature(const TimeSeries *ts, float *feature)
{
    if (!ts || !feature) {
        return ALG_NULL_PTR;
    }

    feature[0] = udsf_median_value(ts->r, ts->n);
    feature[1] = udsf_std_value(ts->r, ts->n);
    feature[2] = udsf_range_value(ts->r, ts->n);
    feature[3] = udsf_slope_feature(ts->r, ts->n);

    feature[4] = udsf_median_value(ts->v, ts->n);
    feature[5] = udsf_std_value(ts->v, ts->n);
    feature[6] = udsf_range_value(ts->v, ts->n);
    feature[7] = udsf_slope_feature(ts->v, ts->n);

    feature[8] = udsf_median_value(ts->a, ts->n);
    feature[9] = udsf_std_value(ts->a, ts->n);
    feature[10] = udsf_range_value(ts->a, ts->n);
    feature[11] = udsf_slope_feature(ts->a, ts->n);

    feature[12] = udsf_median_value(ts->p, ts->n);
    feature[13] = udsf_std_value(ts->p, ts->n);
    feature[14] = udsf_range_value(ts->p, ts->n);
    feature[15] = udsf_slope_feature(ts->p, ts->n);

    feature[16] = udsf_calc_sign_prop(ts->v, ts->n);
    feature[17] = udsf_calc_sign_prop(ts->a, ts->n);
    feature[18] = udsf_calc_dir_changes(ts->v, ts->n);
    feature[19] = udsf_calc_dir_changes(ts->a, ts->n);
    feature[20] = udsf_calc_peaks_num(ts->v, ts->n);
    feature[21] = udsf_calc_peaks_num(ts->a, ts->n);
    feature[22] = udsf_calc_snr_ratio(ts->p, ts->n,GR_SNR_FEA_TH);

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍚嶇О: udsf_calc_feaTh
 * 鍔熻兘鎻忚堪: 璁＄畻鐗瑰緛闂ㄩ檺
 * 杈撳叆鍙傛暟: ts        -- 鏃堕棿搴忓垪
 * 杈撳嚭鍙傛暟: featureTh -- 鐗瑰緛闂ㄩ檺淇℃伅
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_calc_feaTh(const TimeSeries *ts, float *featureTh)
{
    if (!ts || !featureTh) {
        return ALG_NULL_PTR;
    }

    featureTh[0] = udsf_min_value(ts->r, ts->n);
    featureTh[1] = udsf_min_value(ts->a, ts->n);
    featureTh[2] = udsf_max_value(ts->a, ts->n);
    featureTh[3] = featureTh[2] - featureTh[1];
    featureTh[4] = udsf_calc_dir_changes(ts->v, ts->n);
    featureTh[5] = udsf_calc_dir_changes(ts->a, ts->n);
    featureTh[6] = udsf_median_value(ts->p, ts->n);
    featureTh[7] = udsf_absmax_value(ts->v, ts->n);

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍚嶇О: udsf_get_rvap_block
 * 鍔熻兘鎻忚堪: 妫€娴嬮浄杈炬暟鎹腑鐨勬湁鏁堝潡(瀵瑰簲鍘烳ATLAB鐨刧etRvapBlock鍑芥暟)
 * 杈撳叆鍙傛暟: p        -- 杈撳叆鏁版嵁鏁扮粍(閫氬父涓哄鐞嗗悗鐨勬暟鎹?
 *           len      -- 鏁版嵁闀垮害
 * 杈撳嚭鍙傛暟: startIdx -- 鏈夋晥鍧楄捣濮嬬储寮?0鍩虹储寮?
 *           endIdx   -- 鏈夋晥鍧楃粨鏉熺储寮?0鍩虹储寮?
 *           vldFlag  -- 鏈夋晥鍧楁爣蹇?1琛ㄧず鏈夋晥锛?琛ㄧず鏃犳晥)
 * 杩斿洖璇存槑: 0琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_get_rvap_block(const float *p, int len, int *startIdx, int *endIdx, int *vldFlag, int blockNVldMin, int blockNVldMax, int blockNInvldMax, int blockMinNGap)
{
    if ((p == NULL) || (startIdx == NULL) || (endIdx == NULL) || (vldFlag == NULL)) {
        return ALG_NULL_PTR;
    }

    /* 鍒濆鍖栬緭鍑哄弬鏁?0鍩虹储寮曪紝-1琛ㄧず鏃犳湁鏁堢偣) */
    *endIdx = -1;
    *vldFlag = 0;
    *startIdx = -1;

    /* 鎵惧埌绗竴涓湁鏁堢偣(p>0)锛?鍩虹储寮?*/
    for (int n = 0; n < len; n++) {
        if (p[n] > 0) {
            /* 0鍩鸿捣濮嬬储寮?*/
            *startIdx = n;
            break;
        }
    }

    if (*startIdx == -1) {
        /* 鏃犳湁鏁堢偣锛岀洿鎺ヨ繑鍥?*/
        return RADAR_ALG_SUCCESS;
    }

    int vldCnt = 1;     /* 鏈夋晥鐐硅鏁?鍒濆鍖呭惈绗竴涓湁鏁堢偣) */
    int invldCnt = 0;   /* 鏃犳晥鐐硅鏁?*/

    /* 浠庣涓€涓湁鏁堢偣鐨勪笅涓€涓綅缃紑濮嬮亶鍘?杩樺師鍘熷閫昏緫鐨勯亶鍘嗚寖鍥? */
    /* 鍘熷閫昏緫涓?鍩簊tartIdx鐨勪笅涓€涓綅缃槸startIdx(1鍩?锛屽搴?鍩轰负*startIdx */
    for (int n = (*startIdx) + 1; n < len; n++) {
        /* 璺宠繃绗竴涓湁鏁堢偣(宸茶鍏ldCnt)锛屼粠涓嬩竴涓偣寮€濮嬪鐞?*/
        if (p[n] > 0) {     /* 鏈夋晥鐐?*/
            vldCnt++;

            /* 妫€鏌ョ粓姝㈡潯浠? 鏈夋晥鐐规暟閲忓湪鑼冨洿鍐咃紝涓斿悗缁璑gap涓偣鍧囨棤鏁?*/
            if ((vldCnt >= blockNVldMin) && (vldCnt <= blockNVldMax)) {
                bool gapAllInvalid = true;
                for (int k = 1; k <= blockMinNGap; k++) {
                    /* 瓒呭嚭鑼冨洿瑙嗕负鏃犳晥 */
                    if ((n + k) >= len) {
                        continue;
                    }

                    /* 瀛樺湪鏈夋晥鐐癸紝涓嶆弧瓒抽棿闅旀潯浠?*/
                    if (p[n + k] > 0) {
                        gapAllInvalid = false;
                        break;
                    }
                }

                if (gapAllInvalid) {
                    *endIdx = n;    /* 0鍩虹粨鏉熺储寮?*/
                    *vldFlag = 1;
                    return RADAR_ALG_SUCCESS;
                }
            }
        } else {            /* 鏃犳晥鐐?*/
            invldCnt++;

            /* 鏃犳晥鐐硅繃澶氾紝缁堟 */
            if (invldCnt > blockNInvldMax) {
                *vldFlag = 0;
                return RADAR_ALG_SUCCESS;
            }
        }
    }

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍔熻兘: 璁＄畻瑙掑害銆侀€熷害鏂瑰悜鍙樺寲娆℃暟2鐨勭储寮?
 * 杈撳叆鍙傛暟: grFeaVld -- 闆疯揪鐐逛簯鐗瑰緛鏁扮粍
 *           seq_len   -- 瑙掑害搴忓垪闀垮害
 * 杈撳嚭鍙傛暟: 鏂瑰悜鍙樺寲娆℃暟2鐨勭储寮曪細
 *            - 鏃?娆★細seq_len锛?
 *            - 鏈?娆★細杩斿洖2娆＄储寮?
 */
static int udsf_calc_dir_changes_inx2(const RadarPoints *grFeaVld, int seq_len)
{
    /* 鍒濆鍖栨柟鍚戝彉鍖栨鏁颁负0 */
    int dir_changes = 0;
    int dir_changes2 = 0;
    int dir_changes_inx2 = seq_len;

    /* 搴忓垪闀垮害妫€鏌? 涓嶈冻3鐩存帴杩斿洖0 */
    if (seq_len < 3) {
        return seq_len;
    }

    /* 璁＄畻绗﹀彿鍙樺寲 */
    for (int i = 1; i < seq_len; i++) {
        /* 鍒ゆ柇褰撳墠宸垎涓庡墠涓€涓樊鍒嗙殑涔樼Н鏄惁<0(绗﹀彿鐩稿弽=鏂瑰悜鍙樺寲) */
        if ((grFeaVld[i].a * grFeaVld[i - 1].a) < 0.0f) {
           dir_changes++;
        }

        if ((grFeaVld[i].v * grFeaVld[i - 1].v) < 0.0f) {
            dir_changes2++;
        }

        if ((dir_changes == 2) || (dir_changes2 == 2)) {
            dir_changes_inx2 = i + 1; 
            break;
        }
    }

    return dir_changes_inx2;
}

/**
 * 鍑芥暟鍚嶇О: udsf_get_valid_rvap
 * 鍔熻兘鎻忚堪: 鎻愬彇鏈夋晥鍘熷鐗瑰緛骞惰繘琛屽€掔疆澶勭悊
 * 杈撳叆鍙傛暟: ptFifo   -- 鍘熷鐗瑰緛鏁扮粍鎸囬拡
 *           startIdx -- 璧峰绱㈠紩
 *           endIdx   -- 缁撴潫绱㈠紩(涓嶅寘鍚?
 * 杈撳嚭鍙傛暟: grFeaVld -- 杈撳嚭鐨勫鐞嗗悗鐗瑰緛鏁扮粍(闇€纭繚鑷冲皯鑳藉绾矪LOCK_NVLD_MAX涓厓绱?
 * 杩斿洖璇存槑: 鏈夋晥鐗瑰緛鏁伴噺nVld
 */
static int udsf_get_valid_rvap(const RadarPoints *ptFifo, int startIdx, int endIdx, RadarPoints *grFeaVld, int blockNVldMax)
{
    if ((ptFifo == NULL) || (grFeaVld == NULL) || (startIdx >= endIdx)) {
        return 0;
    }

    /* 鎻愬彇鏈夋晥鍘熷鐗瑰緛: 姝ｅ簭 */
    int nVld = 0;
    for (int i = startIdx; i <= endIdx; i++) {
        if (nVld >= blockNVldMax) {
            /* 闃叉鏁扮粍瓒婄晫 */
            break;
        }

        if (ptFifo[i].p > EPS) {
            grFeaVld[nVld] = ptFifo[i];
            nVld++;
        }
    }

    /* 娑堥櫎鎷栧熬鏁版嵁锛侊紒锛?*/
    // nVld = calc_dir_changes_inx2(grFeaVld, nVld);

    /* 灏嗘湁鏁堢壒寰佸€掔疆 */
    if (nVld > 1) {
        for (int i = 0; i < (nVld / 2); i++) {
            RadarPoints temp = grFeaVld[i];
            grFeaVld[i] = grFeaVld[nVld - 1 - i];
            grFeaVld[nVld - 1 - i] = temp;
        }
    }

    return nVld;
}

/**
 * 鍑芥暟鍚嶇О: udsf_point_get
 * 鍔熻兘鎻忚堪: 鑾峰彇闆疯揪鐐逛簯鐨勬墜鍔跨洰鏍囩偣
 * 杈撳叆鍙傛暟: points    -- 杈撳叆鐐逛簯缁撴瀯浣撴暟缁?
 *           pointsNum -- 杈撳叆鐐逛簯鏁?
 *           rThFifo   -- 鏈夋晥鎵嬪娍鐐逛簯鐨勫緞鍚戣窛绂婚棬闄?
 * 杈撳嚭鍙傛暟: ptOut     -- 鏈夋晥鎵嬪娍鐐逛簯
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_point_get(const TargetPointType *points, const int pointsNum, float rThFifoMax, float rThFifoMin, RadarPoints *ptOut)
{
    /* 鍚堟硶鎬ф鏌?*/
    if ((ptOut == NULL) || (points == NULL)) {
        return ALG_NULL_PTR;
    }

    /* CFAR缁撴灉涓虹┖锛屽垯杩欏抚涓烘棤鏁堢偣 */
    if (pointsNum <= 0) {
        ptOut->r = ptOut->v = ptOut->a = ptOut->p = FEAR_INV_DATA;
        return RADAR_ALG_SUCCESS;
    }

    int index = -1;
    float rangeMin = 999;
    float max = FEAR_INV_DATA;
    for (int i = 0; i < pointsNum; i++) {
        /* 瀵绘壘鏈夋晥鑼冨洿鐨勭偣 */
        if ((points[i].range > rThFifoMax)
            || (points[i].range < rThFifoMin)
            || (fabs(points[i].velocity) > FEAR_TAR_VTH_MAX)
            || (fabs(points[i].velocity) < FEAR_TAR_VTH_MIN)
            || (fabs(points[i].angle) > FEAR_TAR_ATH))
        {
            continue;
        }

        if (FEAR_SEL_TYPE == 0) {
            /* 鏈夋晥鑼冨洿鍐呬笖骞呭€兼渶澶х殑鐐?*/
            if (points[i].snr > max) {
                max = points[i].snr;
                index = i;
            }
        } else if (FEAR_SEL_TYPE == 1) {
            /* 鏈夋晥鑼冨洿鍐呬笖璺濈鏈€灏忕殑鐐?*/
            if (points[i].range < rangeMin) {
                rangeMin = points[i].range;
                index = i;
            }
        }

    }

    if (index < 0) {
        ptOut->r = ptOut->v = ptOut->a = ptOut->p = FEAR_INV_DATA;
    } else {
        ptOut->p = points[index].snr;
        ptOut->r = points[index].range;
        ptOut->a = points[index].angle;
        ptOut->v = points[index].velocity;
    }

    return RADAR_ALG_SUCCESS;
}

/* 涓綅鏁拌绠楀嚱鏁?浣跨敤RadarPoints绫诲瀷) */
static RadarPoints udsf_find_median_point(RadarPoints *arr, int size)
{
    TimeSeries ts = {0};
    int retcode = udsf_struct_to_array(arr, size, &ts);
    if (retcode != RADAR_ALG_SUCCESS) {
        return (RadarPoints){0};
    }

    RadarPoints meVelue;
    meVelue.a = udsf_median_value(ts.a, ts.n);
    meVelue.r = ts.r[(ts.n) / 2];
    meVelue.v = ts.v[(ts.n) / 2];
    meVelue.v = udsf_median_value(ts.v, ts.n);

    /* 杩斿洖涓綅鏁?*/
    return meVelue;
}

/**
 * 鍑芥暟鍚嶇О: udsf_feature_filter(new)
 * 鍔熻兘鎻忚堪: 鍘熷鐗瑰緛婊ゆ尝
 * 杈撳叆鍙傛暟: a   -- 杈撳叆鐗瑰緛鏁扮粍
 *           len -- 杈撳叆鐗瑰緛鏁扮粍闀垮害
 *           n0  -- 婊ゆ尝绐楀彛闀垮害
 * 杈撳嚭鍙傛暟: b   -- 杈撳嚭鐗瑰緛鏁扮粍
 * 杩斿洖璇存槑: 鏃?
 * */
static int udsf_feature_filter(RadarPoints *a, int len, int n0, RadarPoints *b)
{
    if ((a == NULL) || (b == NULL)) {
        return ALG_NULL_PTR;
    }

    if (FEAR_FLT_MAX_N0 < n0) {
        return ALG_INVALID_LEN;
    }

    int n = n0 / 2;
    /* 妫€鏌ラ暱搴﹂檺鍒?*/
    if (len < n) {
        n = len;
    }

    /* 闈欐€佹暟缁勫瓨鍌ㄦ墿灞曞悗鐨刟a(浣跨敤瀹忓畾涔夌殑鏈€澶ч暱搴? */
    RadarPoints aa[RADAR_MAX_LEN + 2 * FEAR_FLT_MAX_N0];

    /* 鏋勫缓鎵╁睍鏁扮粍aa */
    /* 鍓峮涓厓绱狅細a(n:-1:1) */
    for (int i = 0; i < n; i++) {
        aa[i] = a[n - 1 - i];
    }

    /* 涓棿len涓厓绱狅細a鏈韩 */
    for (int i = 0; i < len; i++) {
        aa[n + i] = a[i];
    }

    /* 鍚巒涓厓绱狅細a(end:-1:end-n+1) */
    for (int i = 0; i < n; i++) {
        aa[n + len + i] = a[len - 1 - i];
    }

    /* 鏍堝唴瀛樺瓨鍌ㄤ复鏃剁獥鍙ｆ暟鎹?*/
    RadarPoints tmp[FEAR_FLT_MAX_N0];

    /* 璁＄畻姣忎釜浣嶇疆鐨勪腑浣嶆暟 */
    for (int k = 0; k < len; k++) {
        /* 鎻愬彇涓存椂绐楀彛鏁版嵁 */
        for (int i = 0; i < n0; i++) {
            tmp[i] = aa[k + i];
        }

        /* 璁＄畻涓綅鏁板苟瀛樺偍 */
        b[k] = udsf_find_median_point(tmp, n0);
    }

    return RADAR_ALG_SUCCESS;
}

static float udsf_anglerange_thrcalc(float r, float grAngleRangeMin)
{
    if (r <= 0) {
        return 0.0f;
    }

    float sita = 0.0f;
    if (arm_atan2_f32(GR_SWAP_DIS_RANGE_HALF, r, &sita) == ARM_MATH_SUCCESS) {
        sita = sita * 2 * (180.0f / M_PI);
        if (sita > GR_ATH_RANGE_MAX) {
            sita = GR_ATH_RANGE_MAX;
        }

        if (sita < grAngleRangeMin) {
            sita = grAngleRangeMin;
        }
    }

    return sita;
}

static int udsf_gr_rthcalc(float rThIn, float *rThGr, float *rThFifo)
{
    if ((rThGr == NULL) || (rThFifo == NULL)) {
        return ALG_NULL_PTR;
    }

    if ((rThIn < GR_RTH_IN_MIN) || (rThIn > GR_RTH_IN_MAX)) {
        *rThGr = GR_RTH_IN + GR_RTH_ADD;
    } else {
        *rThGr = rThIn + GR_RTH_ADD;
    }

    *rThFifo = *rThGr + GR_RTH_FIFO_ADD;
    if (*rThFifo > FEAR_TAR_RTH) {
        *rThFifo = FEAR_TAR_RTH;
    }

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍑芥暟鍚嶇О: radar_gesture_classify_base
 * 鍔熻兘鎻忚堪: 闆疯揪SVM鎵嬪娍鍒嗙被
 * 杈撳叆鍙傛暟: dcount   -- 鐩爣鏁?
 *           pts      -- 鐩爣鐐逛俊鎭?
 *           para     -- 杈撳叆鍙傛暟
 * 杈撳嚭鍙傛暟: classify -- 杈撳嚭缁撴灉(0:鏃犳墜鍔匡紝1:宸︽尌鎵嬫墜鍔匡紝-1:鍙虫尌鎵嬫墜鍔?
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
static int udsf_radar_gesture_classify_base(const int dcount, const TargetPointType *pts, const GrParams para, RadarPoints *ptFifo, int *classify)
{
    /* 绌烘寚閽堝垽鏂?*/
    if ((pts == NULL) || (classify == NULL)) {
        return ALG_NULL_PTR;
    }

    *classify = GR_HAND_INVLD;
    if (dcount < 0) {
        return ALG_INVALID_LEN;
    }

    int type = para.type;
    int retcode = RADAR_ALG_SUCCESS;

    /* 闆疯揪璺濈闂ㄩ檺鑾峰彇 */
    float rThInner = para.rTh;
    float rThFifoMin = para.rThSelMin;
    float rThFifoMax = para.rThSelMax;

    /* 鎻愬彇鎵嬪娍鐩爣 */
    RadarPoints ptOut;
    retcode = udsf_point_get(pts, dcount, rThFifoMax, rThFifoMin, &ptOut);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    int feaWinLen = para.feaWinLen;
    /* 鎵嬪娍鐐圭洰鏍囩紦瀛?*/
    for (int i = feaWinLen - 1; i > 0; i--) {
        ptFifo[i] = ptFifo[i - 1];
        // pr_info("%2d: r: %6.2f, v: %6.2f, a: %6.2f, p: %6.2f", i, ptFifo[i].r, ptFifo[i].v, ptFifo[i].a, ptFifo[i].p);
    }
    ptFifo[0] = ptOut;

    /* 鍘熷鐗瑰緛棰勬彁鍙?*/
    float p[RADAR_MAX_LEN] = {0};
    for (int i = 0; i < feaWinLen; i++) {
        p[i] = ptFifo[i].p;
    }

    int blockMinNgap = para.blockMinNgap;
    if (udsf_block_invalid(p, blockMinNgap, feaWinLen)) {
        return RADAR_ALG_SUCCESS;
    }

    /* 鎻愬彇鎵嬪娍鏁版嵁鍧?*/
    int startIdx, endIdx, vldFlag;
    int blockNVldMax = para.feaWinLen;
    int blockNVldMin = para.vldPtNumMin;
    int blockMinNGap = para.blockMinNgap;
    int blockNInvldMax = para.blockInvldMax;

    retcode = udsf_get_rvap_block(p, feaWinLen, &startIdx, &endIdx, &vldFlag, blockNVldMin, blockNVldMax, blockNInvldMax, blockMinNGap);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    if (vldFlag == 0) {
        return RADAR_ALG_SUCCESS;
    }

    /* 鎻愬彇鏈夋晥鍘熷鐗瑰緛: 姝ｅ簭 */
    RadarPoints grFeaVld[RADAR_MAX_LEN] = {0};
    int nVld = udsf_get_valid_rvap(ptFifo, startIdx, endIdx, grFeaVld, blockNVldMax);
    if (nVld < blockNVldMin) {
        return RADAR_ALG_SUCCESS;
    }

    /* 鐗瑰緛婊ゆ尝 */
    RadarPoints grFeaFilt[RADAR_MAX_LEN] = {0};
    retcode = udsf_feature_filter(grFeaVld, nVld, para.blockFltN0, grFeaFilt);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 琛嶇敓鐗瑰緛鎻愬彇 */
    TimeSeries ts = {0};
    retcode = udsf_struct_to_array(grFeaFilt, nVld, &ts);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    float feature[FEAR_EXP_NUM] = {0};
    retcode = udsf_calc_feature(&ts, feature);
    PC_RADAR_DEBUG_FLOAT_ARRAY("Gesture Feature", feature, FEAR_EXP_NUM);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    float featureTh[FEAR_EXP_NUM] = {0};
    retcode = udsf_calc_feaTh(&ts, featureTh);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 鎸ユ墜璇嗗埆闂ㄧ1 */
    float gr_athrange_th = udsf_anglerange_thrcalc(featureTh[0], para.angleRangeTh);
    if ((featureTh[0] > rThInner)) {
        return RADAR_ALG_SUCCESS;
    }

    if ((featureTh[3] < gr_athrange_th)) {
        return RADAR_ALG_SUCCESS;
    }

    /* 閫熷害鐗瑰緛闂ㄧ */
    if ((featureTh[4] < para.v0SelMin)) {
        return RADAR_ALG_SUCCESS;
    }

    /* SNR鐗瑰緛闂ㄧ */
    if (featureTh[6] > para.snrTh) {
        return RADAR_ALG_SUCCESS;
    }

    /* 閫熷害鐗瑰緛闂ㄧ */
    if (featureTh[7] < 0.5) {
        return RADAR_ALG_SUCCESS;
    } 

#if 0
    /* 瑙掑害鐗瑰緛闂ㄧ */
    if ((featureTh[5] > 2) || (featureTh[5] < 1)) {
        return RADAR_ALG_SUCCESS;
    }

    /* 鎸ユ墜璇嗗埆闂ㄧ2 */
    if ((featureTh[1] > GR_ATH_NEG_MAX) || (featureTh[2] < GR_ATH_POS_MIN)) {
        return RADAR_ALG_SUCCESS;
    }

    /* 鎸ユ墜璇嗗埆闂ㄧ3: 鍙兘涓庨€熷害鐗瑰緛闂ㄧ鍐茬獊 */
    if ((type == 0) && (!udsf_block_full(p, blockMinNGap))) {
        if (nVld < GR_VLD_BLOCK_PTS) {
            return RADAR_ALG_SUCCESS;
        }
    }
#endif

    /* 鐗瑰緛褰掍竴鍖?*/
    retcode = udsf_gaus_normalization(feature);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* SVM鎵嬪娍璇嗗埆 */
    float decisions[GR_CLASS_NUM];
    retcode = udsf_calc_svm_scores(feature, decisions);
    PC_RADAR_DEBUG_FLOAT_ARRAY("Gesture Decisions", decisions, GR_CLASS_NUM);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    retcode = udsf_predict_classify(decisions, GR_CLASS_NUM, classify);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* FIFO寮哄埗鏇存柊 */
    int grType = *classify;
    RadarPoints ptFifoTmp = {0};
    if (grType != GR_HAND_INVLD) {
        for (int i = startIdx; i < endIdx; i++) {
            g_ptFifo_base[i] = ptFifoTmp;
            g_ptFifo_base2[i] = ptFifoTmp;
            g_ptFifo_base3[i] = ptFifoTmp;
        }
    }

    return retcode;
}

/**
 * 鍑芥暟鍚嶇О: radar_gesture_classify
 * 鍔熻兘鎻忚堪: 闆疯揪SVM鎵嬪娍鍒嗙被
 * 杈撳叆鍙傛暟: dcount   -- 鐩爣鏁?
 *           pts      -- 鐩爣鐐逛俊鎭?
 *           rTh      -- 鐢ㄦ埛鐣岄潰璺濈闃堝€?
 * 杈撳嚭鍙傛暟: classify -- 杈撳嚭缁撴灉(0:鏃犳墜鍔匡紝1:宸︽尌鎵嬫墜鍔匡紝-1:鍙虫尌鎵嬫墜鍔?
 * 杩斿洖璇存槑: 杩斿洖鐘舵€佺爜锛?琛ㄧず鎴愬姛锛屽叾浠栧€艰〃绀哄け璐?
 */
int radar_gesture_classify(const int dcount, const TargetPointType *pts, const uint16_t rTh, int *classify)
{
    /* 绌烘寚閽堝垽鏂?*/
    if ((pts == NULL) || (classify == NULL)) {
        return ALG_NULL_PTR;
    }

    *classify = GR_HAND_INVLD;
    if (dcount < 0) {
        return ALG_INVALID_LEN;
    }

    /* 鍏ュ彛鍙傛暟杈圭晫淇濇姢 */
    float rTht = rTh / 100.0f;
    float rThIn = rTht;
    if (rTht < GR_RTH_IN_MIN) {
        rThIn = GR_RTH_IN_MIN;
    }
    if (rTht > GR_RTH_IN_MAX) {
        rThIn = GR_RTH_IN_MAX;
    }

    /* 鎸ユ墜璇嗗埆 */
    int retcode;
    int classifBase3;
    GrParams paraBase3 = {  /* 鎸ユ墜璇嗗埆鍙傛暟 */
        .type          = 0,
        .rTh           = rThIn + GR_RTH_ADD,
        .angleRangeTh  = GR_ATH_RANGE_MIN,
        .snrTh         = GR_SNR_VLD_TH,
        .vldPtNumMin   = BLOCK_NVLD_MIN,
        .feaWinLen     = FEAR_WIN_LEN,
        .blockInvldMax = BLOCK_NINVLD_MAX,
        .blockMinNgap  = BLOCK_MIN_NGAP,
        .blockFltN0    = BLOCK_FLT_N0,
        .rThSelMin     = 0.0f,
        .rThSelMax     = rThIn + 0.05f,
        .v0SelMin      = GR_V0_SEL_MIN,
    };
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase3, g_ptFifo_base3, &classifBase3);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }
    if (classifBase3 != GR_HAND_INVLD) {
        *classify = classifBase3;
        return retcode;
    }

    int classifBase2;
    GrParams paraBase2 = {  /* 鎸ユ墜璇嗗埆鍙傛暟 */
        .type          = 0,
        .rTh           = rThIn + GR_RTH_ADD,
        .angleRangeTh  = GR_ATH_RANGE_MIN,
        .snrTh         = GR_SNR_VLD_TH,
        .vldPtNumMin   = BLOCK_NVLD_MIN,
        .feaWinLen     = FEAR_WIN_LEN,
        .blockInvldMax = BLOCK_NINVLD_MAX,
        .blockMinNgap  = BLOCK_MIN_NGAP,
        .blockFltN0    = BLOCK_FLT_N0,
        .rThSelMin     = 0.0f,
        .rThSelMax     = rThIn + 0.125f,
        .v0SelMin      = GR_V0_SEL_MIN,
    };
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase2, g_ptFifo_base2, &classifBase2);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }
    if (classifBase2 != GR_HAND_INVLD) {
        *classify = classifBase2;
        return retcode;
    }

    int classifBase1;
    GrParams paraBase1 = {  /* 鎸ユ墜璇嗗埆鍙傛暟 */
        .type          = 0,
        .rTh           = rThIn + GR_RTH_ADD,
        .angleRangeTh  = GR_ATH_RANGE_MIN,
        .snrTh         = GR_SNR_VLD_TH,
        .vldPtNumMin   = BLOCK_NVLD_MIN,
        .feaWinLen     = FEAR_WIN_LEN,
        .blockInvldMax = BLOCK_NINVLD_MAX,
        .blockMinNgap  = BLOCK_MIN_NGAP,
        .blockFltN0    = BLOCK_FLT_N0,
        .rThSelMin     = 0.0f,
        .rThSelMax     = rThIn + GR_RTH_FIFO_ADD,
        .v0SelMin      = GR_V0_SEL_MIN,
    };
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase1, g_ptFifo_base, &classifBase1);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }
    if (classifBase1 != GR_HAND_INVLD) {
        *classify = classifBase1;
        return retcode;
    }

    return retcode;
}
#elif (CONFIG_ALG_DTYPE == CONFIG_ALG_DINT)

/* --- 鍩虹瀹氱偣杈呭姪鍑芥暟 --- */

/* 闈欐€?FIFO 缂撳瓨 (Q8 鏍煎紡) */
static RadarPoints g_ptFifo_base[RADAR_MAX_FIFOLEN] = {0};
static RadarPoints g_ptFifo_base2[RADAR_MAX_FIFOLEN] = {0};
static RadarPoints g_ptFifo_base3[RADAR_MAX_FIFOLEN] = {0};

/* 鎻掑叆鎺掑簭锛氶拡瀵瑰皬鏁扮粍(FEAR_WIN_LEN=30)姣攓sort鏇寸渷鏍堢┖闂翠笖閫熷害鏇村揩 */
static void udsf_insertion_sort_q8(int16_t *arr, int n) {
    for (int i = 1; i < n; i++) {
        int16_t key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/* 璁＄畻鍧囧€?Q8 */
static int16_t udsf_calc_mean_q8(const int16_t *arr, int n) {
    if (n <= 0) return 0;
    int32_t sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return (int16_t)(sum / n);
}

/* 鍙栧簭鍒椾腑鐨勪腑浣嶅€?Q8 */
static int16_t udsf_median_value_q8(const int16_t *arr, int n) {
    if (n <= 0) return 0;
    if (n == 1) return arr[0];

    int16_t temp[FEAR_WIN_LEN];
    memcpy(temp, arr, n * sizeof(int16_t));
    udsf_insertion_sort_q8(temp, n);

    if ((n % 2) == 0) {
        return (temp[n / 2 - 1] + temp[n / 2]) / 2;
    }
    return temp[n / 2];
}

/* ========================================================
   鏋侀€熺函鏁村瀷骞虫柟鏍瑰嚱鏁?(鏃犳诞鐐癸紝楂樼簿搴?
   ======================================================== */
static inline uint32_t udsf_fast_isqrt(uint32_t val) 
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; /* 浠庢渶楂樺伓鏁颁綅寮€濮?*/
    
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

/* 鏍囧噯宸?Q8 */
static int16_t udsf_std_value_q8(const int16_t *arr, int n) {
    if (n <= 1) return 0;
    int16_t mean = udsf_calc_mean_q8(arr, n);
    int64_t sum_sq = 0;
    for (int i = 0; i < n; i++) {
        int32_t diff = (int32_t)arr[i] - mean;
        sum_sq += (int64_t)diff * diff; /* Q16 */
    }
    /* sum_sq/n-1 渚濈劧鏄?Q16, 寮€鏂瑰悗鍥炲埌 Q8 */
    return (int16_t)udsf_fast_isqrt((uint32_t)(sum_sq / (n - 1)));
}

/* 鏋佸樊 Q8 */
static int16_t udsf_range_value_q8(const int16_t *arr, int n) {
    if (n <= 1) return 0;
    int16_t max_v = arr[0], min_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) max_v = arr[i];
        if (arr[i] < min_v) min_v = arr[i];
    }
    return (max_v - min_v);
}

/* 鏂滅巼 Q8 */
static int16_t udsf_slope_feature_q8(const int16_t *arr, int n) {
    if (n <= 1) return 0;
    int max_i = 0, min_i = 0;
    int16_t max_v = arr[0], min_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) { max_v = arr[i]; max_i = i; }
        if (arr[i] < min_v) { min_v = arr[i]; min_i = i; }
    }
    if (max_i == min_i) return 0;
    /* 鐗╃悊鍚箟: (max_v - min_v) / (max_i - min_i) * n */
    return (int16_t)(((int32_t)(max_v - min_v) * n) / (max_i - min_i));
}

/* 瑙掑害宸垎绗﹀彿姣斾緥 (Q8妯℃嫙 0~1.0, 鍗?0~256) */
static int16_t udsf_calc_sign_prop_q8(const int16_t *angle_seq, int seq_len) {
    if (seq_len < 3) return 128; /* 0.5f */
    int pos_count = 0;
    int diff_len = seq_len - 1;
    for (int i = 0; i < diff_len; i++) {
        if ((angle_seq[i + 1] - angle_seq[i]) > 0) pos_count++;
    }
    return (int16_t)((pos_count << 8) / diff_len);
}

/* 瑙掑害鏂瑰悜鍙樺寲娆℃暟 */
static int udsf_calc_dir_changes(const int16_t *angle_seq, int seq_len) {
    int dir_changes = 0;
    if (seq_len < 3) return 0;
    for (int i = 1; i < seq_len; i++) {
        /* 鍒ゆ柇璺ㄨ秺 0 鐐?(绗﹀彿鍙樺寲) */
        if (((int32_t)angle_seq[i] * angle_seq[i - 1]) < 0) {
            dir_changes++;
        }
    }
    return dir_changes;
}

/* 閫熷害灞€閮ㄥ嘲鍊兼暟閲?*/
static int udsf_calc_peaks_num(const int16_t *vel_seq, int seq_len) {
    int vel_peaks = 0;
    if (seq_len < 3) return 0;
    for (int i = 1; i < (seq_len - 1); i++) {
        if ((vel_seq[i] > vel_seq[i - 1]) && (vel_seq[i] > vel_seq[i + 1])) {
            vel_peaks++;
        }
    }
    return vel_peaks;
}

/* 楂?SNR 鏍锋湰姣斾緥 (杩斿洖 Q8) */
static int16_t udsf_calc_snr_ratio_q8(const int16_t *snr_seq, int seq_len, int16_t threshold_q8) {
    if (seq_len < 4) return 0;
    int high_snr_count = 0;
    for (int i = 0; i < seq_len; i++) {
        if (snr_seq[i] > threshold_q8) high_snr_count++;
    }
    return (int16_t)((high_snr_count << 8) / seq_len);
}

/* 鑾峰彇搴忓垪鏈€灏忓€?(Q8) */
static int16_t udsf_min_value_q8(const int16_t *arr, int n)
{
    if ((n < 1) || (arr == NULL)) return 0;

    int16_t min_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] < min_v) {
            min_v = arr[i];
        }
    }
    return min_v;
}

/* 鑾峰彇搴忓垪鏈€澶у€?(Q8) */
static int16_t udsf_max_value_q8(const int16_t *arr, int n)
{
    if ((n < 1) || (arr == NULL)) return 0;

    int16_t max_v = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max_v) {
            max_v = arr[i];
        }
    }
    return max_v;
}

/* 鑾峰彇搴忓垪缁濆鍊肩殑鏈€澶у€?(Q8) */
static int16_t udsf_absmax_value_q8(const int16_t *arr, int n)
{
    if ((n < 1) || (arr == NULL)) return 0;

    uint16_t max_v = (arr[0] < 0) ? -arr[0] : arr[0];
    for (int i = 1; i < n; i++) {
        uint16_t tmp = (arr[i] < 0) ? -arr[i] : arr[i];
        if (tmp > max_v) {
            max_v = tmp;
        }
    }
    return (int16_t)max_v;
}

/**
 * 鍔熻兘鎻忚堪: 棰勭瓫閫夋棤鏁堟墜鍔垮潡 (Q8瀹氱偣鐗?
 * 妫€鏌?FIFO 涓墠 ngap+1 涓偣鏄惁閮戒綆浜庨槇鍊?EPS_Q8 (1)
 */
static bool udsf_block_invalid_q8(int16_t *p, int ngap, int feaWinLen)
{
    if ((p == NULL) || (ngap >= (feaWinLen - 1)) || (ngap <= 0)) {
        return true;
    }

    bool pjsum = true;
    for (int i = 0; i <= ngap; i++) {
        /* 鍒ゆ柇骞呭害鏄惁灏忎簬 Q8 鏈€灏忓崟浣?1 (鍗?1/256) */
        pjsum &= (p[i] < EPS_Q8);
    }

    return pjsum;
}

/**
 * 鍔熻兘鎻忚堪: 鍘熷鐗瑰緛涓€兼护娉?(Q8瀹氱偣鐗?
 * 瀵?R/V/A 搴忓垪鎵ц婊戝姩绐楀彛涓€兼护娉?
 */
static int udsf_feature_filter_q8(RadarPoints *a, int len, int n0, RadarPoints *b)
{
    if ((a == NULL) || (b == NULL)) return ALG_NULL_PTR;
    
    int n = n0 / 2;
    if (len < n) n = len;

    /* 浣跨敤鏍堢┖闂村瓨鍌ㄦ墿灞曞悗鐨勪复鏃跺簭鍒?aa */
    /* RADAR_MAX_LEN (40) + 2*n0 (15) 绾?70 涓厓绱狅紝M0 鏍堢┖闂村彲鎵垮彈 */
    RadarPoints aa[RADAR_MAX_LEN + 32];
    
    /* 1. 鏋勫缓闀滃儚鎵╁睍鏁扮粍 (澶勭悊杈圭晫) */
    for (int i = 0; i < n; i++) aa[i] = a[n - 1 - i];
    for (int i = 0; i < len; i++) aa[n + i] = a[i];
    for (int i = 0; i < n; i++) aa[n + len + i] = a[len - 1 - i];

    /* 2. 婊戝姩绐楀彛璁＄畻涓綅鏁?*/
    int16_t win_r[FEAR_FLT_MAX_N0], win_v[FEAR_FLT_MAX_N0], win_a[FEAR_FLT_MAX_N0];

    for (int k = 0; k < len; k++) {
        for (int i = 0; i < n0; i++) {
            win_r[i] = aa[k + i].r;
            win_v[i] = aa[k + i].v;
            win_a[i] = aa[k + i].a;
        }
        /* 璋冪敤鎻掑叆鎺掑簭骞跺彇涓綅鍊?*/
        b[k].r = udsf_median_value_q8(win_r, n0);
        b[k].v = udsf_median_value_q8(win_v, n0);
        b[k].a = udsf_median_value_q8(win_a, n0);
        b[k].p = aa[n + k].p; /* SNR 涓嶅弬涓庢护娉紝鐩存帴閫忎紶 */
    }

    return RADAR_ALG_SUCCESS;
}

/**
 * 鍔熻兘鎻忚堪: 鎻愬彇 23 涓鐢熺壒寰?(Q8瀹氱偣鐗?
 * 娉ㄦ剰锛歴td 浣跨敤 udsf_fast_isqrt锛宻lope/prop 浣跨敤 32 浣嶉櫎娉?
 */
static int udsf_calc_feature_q8(const TimeSeries *ts, int16_t *feature)
{
    if (!ts || !feature) return ALG_NULL_PTR;

    /* 璺濈 (Range) 鐗瑰緛 [0-3] */
    feature[0] = udsf_median_value_q8(ts->r, ts->n);
    feature[1] = udsf_std_value_q8(ts->r, ts->n);
    feature[2] = udsf_range_value_q8(ts->r, ts->n);
    feature[3] = udsf_slope_feature_q8(ts->r, ts->n);

    /* 閫熷害 (Velocity) 鐗瑰緛 [4-7] */
    feature[4] = udsf_median_value_q8(ts->v, ts->n);
    feature[5] = udsf_std_value_q8(ts->v, ts->n);
    feature[6] = udsf_range_value_q8(ts->v, ts->n);
    feature[7] = udsf_slope_feature_q8(ts->v, ts->n);

    /* 瑙掑害 (Angle) 鐗瑰緛 [8-11] */
    feature[8] = udsf_median_value_q8(ts->a, ts->n);
    feature[9] = udsf_std_value_q8(ts->a, ts->n);
    feature[10] = udsf_range_value_q8(ts->a, ts->n);
    feature[11] = udsf_slope_feature_q8(ts->a, ts->n);

    /* SNR 鐗瑰緛 [12-15] */
    feature[12] = udsf_median_value_q8(ts->p, ts->n);
    feature[13] = udsf_std_value_q8(ts->p, ts->n);
    feature[14] = udsf_range_value_q8(ts->p, ts->n);
    feature[15] = udsf_slope_feature_q8(ts->p, ts->n);

    /* 鎵嬪娍鐗瑰畾璇嗗埆鐗瑰緛 [16-22] */
    /* Q8 鏍煎紡锛氭瘮渚?0.5f 瀛樺偍涓?128 */
    feature[16] = udsf_calc_sign_prop_q8(ts->v, ts->n);
    feature[17] = udsf_calc_sign_prop_q8(ts->a, ts->n);
    
    /* 鍙樺寲娆℃暟鍜屽嘲鍊兼暟鏄暣鏁帮紝浣嗗湪 SVM 涓篃闇€缁熶竴涓?Q8 浠ュ榻愭ā鍨?*/
    feature[18] = (int16_t)udsf_calc_dir_changes(ts->v, ts->n) << 8;
    feature[19] = (int16_t)udsf_calc_dir_changes(ts->a, ts->n) << 8;
    feature[20] = (int16_t)udsf_calc_peaks_num(ts->v, ts->n) << 8;
    feature[21] = (int16_t)udsf_calc_peaks_num(ts->a, ts->n) << 8;
    
    /* SNR 姣斾緥锛岄槇鍊间紶 GR_SNR_FEA_TH_Q8 */
    feature[22] = udsf_calc_snr_ratio_q8(ts->p, ts->n, GR_SNR_FEA_TH_Q8);

    return RADAR_ALG_SUCCESS;
}

/* --- SVM 鏍稿績瀹氱偣璁＄畻 --- */

/* 楂樻柉褰掍竴鍖栦笌 SVM 璇勫垎 (Q8鐗瑰緛 -> Q12鍐崇瓥) */
static int udsf_calc_svm_scores_q12(int16_t *features_q8, int32_t *decisions_q12) {
    int32_t feat_norm_q12[FEAR_EXP_NUM];

    /* 1. 褰掍竴鍖? (x - mu) * (1/sigma) */
    for (int i = 0; i < FEAR_EXP_NUM; i++) {
        int32_t diff = (int32_t)features_q8[i] - g_mu_q8[i];
        /* Q8 * Q12 = Q20 -> 鍙崇Щ 8 浣嶅緱鍒?Q12 */
        feat_norm_q12[i] = (diff * g_inv_sigma_q12[i]) >> 8;
    }

    /* 2. 鐭╅樀涔樻硶: dot(weights, feat) + bias */
    for (int d = 0; d < GR_CLASS_NUM; d++) {
        int64_t sum_q24 = 0;
        for (int f = 0; f < FEAR_EXP_NUM; f++) {
            /* Q12 * Q12 = Q24 */
            sum_q24 += (int64_t)feat_norm_q12[f] * g_svms_w_q12[d][f];
        }
        /* Q24 >> 12 = Q12 */
        decisions_q12[d] = (int32_t)(sum_q24 >> 12) + g_svms_b_q12[d];
    }
    return RADAR_ALG_SUCCESS;
}

/* 棰勬祴鍒嗙被 (鏍规嵁鏈€澶?score) */
static int udsf_predict_classify(const int32_t *scores_q12, int *classify) {
    const int classNames[] = {GR_HAND_RIGHT, GR_HAND_INVLD, GR_HAND_LEFT};
    int maxIdx = 0;
    int32_t maxVal = scores_q12[0];

    for (int i = 1; i < GR_CLASS_NUM; i++) {
        if (scores_q12[i] > maxVal) {
            maxVal = scores_q12[i];
            maxIdx = i;
        }
    }
    *classify = classNames[maxIdx];
    return RADAR_ALG_SUCCESS;
}

/* --- 鎵嬪娍鎻愬彇涓庡潡妫€娴嬮€昏緫 (Q8) --- */
static int udsf_point_get(const TargetPointType *points, int pointsNum, int16_t rThMax, int16_t rThMin, RadarPoints *ptOut) {
    if (pointsNum <= 0) {
        ptOut->r = ptOut->v = ptOut->a = ptOut->p = FEAR_INV_DATA_Q8;
        return RADAR_ALG_SUCCESS;
    }

    int index = -1;
    int16_t rangeMin = 32767;
    int16_t maxSnr = -32768;

    for (int i = 0; i < pointsNum; i++) {
        /* Q8 姣旇緝 */
        int16_t absV = points[i].velocity > 0 ? points[i].velocity : -points[i].velocity;
        int16_t absA = points[i].angle > 0 ? points[i].angle : -points[i].angle;

        if (points[i].range > rThMax || points[i].range < rThMin ||
            absV > FEAR_TAR_VTH_MAX_Q8 || absV < FEAR_TAR_VTH_MIN_Q8 ||
            absA > FEAR_TAR_ATH_Q8) {
            continue;
        }

        if (FEAR_SEL_TYPE == 0) {
            if (points[i].snr > maxSnr) {
                maxSnr = points[i].snr;
                index = i;
            }
        } else {
            if (points[i].range < rangeMin) {
                rangeMin = points[i].range;
                index = i;
            }
        }
    }

    if (index < 0) {
        ptOut->r = ptOut->v = ptOut->a = ptOut->p = FEAR_INV_DATA_Q8;
    } else {
        ptOut->p = points[index].snr;
        ptOut->r = points[index].range;
        ptOut->a = points[index].angle;
        ptOut->v = points[index].velocity;
    }
    return RADAR_ALG_SUCCESS;
}

static int32_t udsf_atan2_cordic_q15(int32_t y, int32_t x) 
{
    if (x == 0 && y == 0) return 0;
    
    int32_t angle = 0;
    if (x < 0) {
        x = -x;
        y = -y;
        angle = 32768; /* Q15 鏍煎紡涓嬬殑 PI */
    }
    
    int32_t cur_x, cur_y;
    
    /* 16 娆¤凯浠ｇ殑瑙掑害琛? atan(2^-i) / PI * 32768 */
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

/* 瑙掑害鏋佸樊闂ㄩ檺鍔ㄦ€佽绠?(瀹氱偣 CORDIC 鐗? */
static int16_t udsf_anglerange_thrcalc_q8(int16_t r_q8, int16_t grAngleRangeMin_q8) {
    if (r_q8 <= 0) return 0;
    /* 鐗╃悊: atan2(0.175, r) * 2 * 180/PI. 0.175m = 45 (Q8) */
    int32_t phase = udsf_atan2_cordic_q15(45, r_q8); 
    /* phase 32768 = 180搴? 璁＄畻 sita = phase * 360 / 32768 */
    int32_t sita_q8 = (phase * 360 * 256) >> 15;

    if (sita_q8 > GR_ATH_RANGE_MAX_Q8) sita_q8 = GR_ATH_RANGE_MAX_Q8;
    if (sita_q8 < grAngleRangeMin_q8) sita_q8 = grAngleRangeMin_q8;
    return (int16_t)sita_q8;
}

/**
 * 鍑芥暟鍚嶇О: udsf_radar_gesture_classify_base
 * 鍔熻兘鎻忚堪: 闆疯揪 SVM 鎵嬪娍鍒嗙被鍐呮牳锛堝畾鐐瑰寲鏃犵渷鐣ョ増锛?
 * 杈撳叆鍙傛暟: dcount   -- 鐩爣鏁?
 *           pts      -- 鐩爣鐐逛俊鎭?(鏉ヨ嚜娴嬭妯″潡鐨?TargetPointType)
 *           para     -- 绠楁硶鍙傛暟 (GrParams 缁撴瀯浣?
 *           ptFifo   -- 褰撳墠绾ц仈浣跨敤鐨勫巻鍙叉暟鎹?FIFO
 * 杈撳嚭鍙傛暟: classify -- 杈撳嚭缁撴灉(0:鏃犳墜鍔匡紝1:宸︼紝-1:鍙?
 * 杩斿洖璇存槑: 鐘舵€佺爜 (RADAR_ALG_SUCCESS 涓烘垚鍔?
 */
static int udsf_radar_gesture_classify_base(const int dcount, const TargetPointType *pts, const GrParams para, RadarPoints *ptFifo, int *classify)
{
    /* 1. 鍒濆鍖栦笌鍚堟硶鎬ф鏌?*/
    if ((pts == NULL) || (classify == NULL) || (ptFifo == NULL)) {
        return ALG_NULL_PTR;
    }

    *classify = GR_HAND_INVLD;
    if (dcount < 0) {
        return ALG_INVALID_LEN;
    }

    int retcode = RADAR_ALG_SUCCESS;
    int feaWinLen = para.feaWinLen;

    /* 2. 鎻愬彇褰撳墠甯ф渶浼樻墜鍔跨洰鏍囩偣 (Q8 鏍煎紡) */
    RadarPoints ptOut;
    retcode = udsf_point_get(pts, dcount, para.rThSelMax, para.rThSelMin, &ptOut);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 3. 鏇存柊鍘嗗彶鐗瑰緛 FIFO (鎵嬪姩绉讳綅锛孧0 鏁堢巼鏈€楂? */
    for (int i = feaWinLen - 1; i > 0; i--) {
        ptFifo[i] = ptFifo[i - 1];
    }
    ptFifo[0] = ptOut;

    /* 4. 鎻愬彇褰撳墠婊戝姩绐楀彛鐨?SNR 搴忓垪鐢ㄤ簬鍧楁娴?*/
    int16_t p_seq[RADAR_MAX_LEN] = {0};
    for (int i = 0; i < feaWinLen; i++) {
        p_seq[i] = ptFifo[i].p;
    }

    /* 5. 棰勭瓫閫夛細鍒ゆ柇鏁版嵁鏄惁鐢变簬杩囬暱鏃堕棿娌℃湁鏈夋晥鐐硅€屽け鏁?*/
    if (udsf_block_invalid_q8(p_seq, para.blockMinNgap, feaWinLen)) {
        return RADAR_ALG_SUCCESS;
    }

    /* 6. 鎵ц鏈夋晥鏁版嵁鍧楁娴?(getRvapBlock) */
    int startIdx = -1;
    int endIdx = -1;
    int vldFlag = 0;
    
    /* 鍐呴儴閫昏緫锛氬鎵剧涓€涓湁鏁堢偣骞跺垽瀹氬叾缁撴潫杈圭晫 */
    /* 瀵绘壘璧峰鐐?(鐢变簬鏄€掑簭FIFO锛?鏄渶鏂帮紝鏁呴渶浠庡ご鎵剧涓€涓湁鏁堢偣) */
    for (int n = 0; n < feaWinLen; n++) {
        if (p_seq[n] > EPS_Q8) {
            startIdx = n;
            break;
        }
    }

    if (startIdx == -1) {
        return RADAR_ALG_SUCCESS;
    }

    int vldCnt = 1;
    int invldCnt = 0;
    /* 閬嶅巻瀵绘壘鍧楃粨鏉熺偣 */
    for (int n = startIdx + 1; n < feaWinLen; n++) {
        if (p_seq[n] > EPS_Q8) {
            vldCnt++;
            /* 妫€鏌ヨ繛缁┖闅欙紝鍒ゆ柇鍧楁槸鍚︾粨鏉?*/
            if ((vldCnt >= para.vldPtNumMin) && (vldCnt <= para.feaWinLen)) {
                bool gapAllInvalid = true;
                for (int k = 1; k <= para.blockMinNgap; k++) {
                    if ((n + k) >= feaWinLen) continue;
                    if (p_seq[n + k] > EPS_Q8) {
                        gapAllInvalid = false;
                        break;
                    }
                }
                if (gapAllInvalid) {
                    endIdx = n;
                    vldFlag = 1;
                    break;
                }
            }
        } else {
            invldCnt++;
            if (invldCnt > para.blockInvldMax) {
                vldFlag = 0;
                break;
            }
        }
    }

    /* 濡傛灉娌℃娴嬪埌瀹屾暣鐨勬湁鏁堝潡锛屽垯閫€鍑?*/
    if (vldFlag == 0) {
        return RADAR_ALG_SUCCESS;
    }

    /* 7. 鎻愬彇鏈夋晥鍘熷鐗瑰緛骞惰繘琛屽€掔疆 (杩樺師鎵嬪娍鍙戠敓鐨勬椂闂撮『搴? */
    RadarPoints grFeaVld[RADAR_MAX_LEN] = {0};
    int nVld = 0;
    /* 鎻愬彇鏈夋晥鐐瑰苟缈昏浆椤哄簭 */
    for (int i = endIdx; i >= startIdx; i--) {
        if (ptFifo[i].p > EPS_Q8) {
            grFeaVld[nVld] = ptFifo[i];
            nVld++;
            if (nVld >= RADAR_MAX_LEN) break;
        }
    }

    if (nVld < para.vldPtNumMin) {
        return RADAR_ALG_SUCCESS;
    }

    /* 8. 鍘熷鐗瑰緛婊ゆ尝 (涓€兼护娉? */
    RadarPoints grFeaFilt[RADAR_MAX_LEN] = {0};
    retcode = udsf_feature_filter_q8(grFeaVld, nVld, para.blockFltN0, grFeaFilt);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 9. 缁撴瀯浣撹浆鏁扮粍 (鍑嗗杩涜鐗瑰緛缁熻璁＄畻) */
    TimeSeries ts = {0};
    ts.n = nVld;
    for (int i = 0; i < nVld; i++) {
        ts.r[i] = grFeaFilt[i].r;
        ts.v[i] = grFeaFilt[i].v;
        ts.a[i] = grFeaFilt[i].a;
        ts.p[i] = grFeaFilt[i].p;
    }

    /* 10. 鎻愬彇鐗瑰緛鍚戦噺 (23 涓墿灞曠壒寰侊紝Q8 鏍煎紡) */
    int16_t feature[FEAR_EXP_NUM] = {0};
    /* feature[0..15]: R,V,A,P 鐨勪腑浣嶆暟銆佹爣鍑嗗樊銆佹瀬宸€佹枩鐜?*/
    /* feature[16..22]: 绗﹀彿姣斾緥銆佹柟鍚戝彉鍖栨暟銆佸嘲鍊兼暟銆丼NR姣斾緥 */
    retcode = udsf_calc_feature_q8(&ts, feature);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 11. 鎻愬彇闂ㄩ檺鏍￠獙鐗瑰緛 */
    int16_t featureTh[8];
    featureTh[0] = udsf_min_value_q8(ts.r, ts.n);        /* R_min */
    featureTh[1] = udsf_min_value_q8(ts.a, ts.n);        /* A_min */
    featureTh[2] = udsf_max_value_q8(ts.a, ts.n);        /* A_max */
    featureTh[3] = featureTh[2] - featureTh[1];         /* A_range */
    featureTh[4] = (int16_t)udsf_calc_dir_changes(ts.v, ts.n);
    featureTh[5] = (int16_t)udsf_calc_dir_changes(ts.a, ts.n);
    featureTh[6] = udsf_median_value_q8(ts.p, ts.n);     /* SNR_median */
    featureTh[7] = udsf_absmax_value_q8(ts.v, ts.n);    /* V_absmax */

    /* 12. 鎵嬪娍璇嗗埆闂ㄧ閫昏緫 (Q8 姣旇緝) */
    /* 鍔ㄦ€佽绠楄搴﹂棬闄?(鑰冭檻璺濈琛板噺) */
    int16_t gr_athrange_th = udsf_anglerange_thrcalc_q8(featureTh[0], para.angleRangeTh);
    
    /* 璺濈闂ㄧ */
    if (featureTh[0] > para.rTh) {
        return RADAR_ALG_SUCCESS;
    }
    /* 瑙掑害鏋佸樊闂ㄧ */
    if (featureTh[3] < gr_athrange_th) {
        return RADAR_ALG_SUCCESS;
    }
    /* 閫熷害娲昏穬搴﹂棬绂?*/
    if (featureTh[4] < (int16_t)para.v0SelMin) {
        return RADAR_ALG_SUCCESS;
    }
    /* 淇″櫔姣斿己搴﹂棬绂?*/
    if (featureTh[6] > para.snrTh) {
        return RADAR_ALG_SUCCESS;
    }
    /* 缁濆閫熷害闂ㄧ (鑷冲皯闇€瑕?0.5m/s 鐨勭灛鏃堕€熷害) */
    if (featureTh[7] < 128) { /* 0.5 * 256 = 128 */
        return RADAR_ALG_SUCCESS;
    }

    /* 13. SVM 妯″瀷璁＄畻 (瀹氱偣鍖栨牳蹇? */
    /* 鐗瑰緛褰掍竴鍖? (feature - mu) / sigma */
    /* 璁＄畻鎵撳垎: dot(feature_norm, weights) + bias */
    int32_t decisions[GR_CLASS_NUM];
    retcode = udsf_calc_svm_scores_q12(feature, decisions);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 14. 鍒ゅ畾绫诲埆 (鍙栨渶澶ф墦鍒? */
    retcode = udsf_predict_classify(decisions, classify);
    if (retcode != RADAR_ALG_SUCCESS) {
        return retcode;
    }

    /* 15. 濡傛灉璇嗗埆鍑烘墜鍔匡紝寮哄埗娓呯┖ FIFO 缂撳瓨锛岄槻姝㈣繛甯﹁Е鍙?*/
    if (*classify != GR_HAND_INVLD) {
        RadarPoints ptEmpty = {FEAR_INV_DATA_Q8, FEAR_INV_DATA_Q8, FEAR_INV_DATA_Q8, FEAR_INV_DATA_Q8};
        for (int i = 0; i < feaWinLen; i++) {
            g_ptFifo_base[i] = ptEmpty;
            g_ptFifo_base2[i] = ptEmpty;
            g_ptFifo_base3[i] = ptEmpty;
        }
    }

    return retcode;
}

/**
 * 鍑芥暟鍚嶇О: radar_gesture_classify
 * 鍔熻兘鎻忚堪: 闆疯揪 SVM 鎵嬪娍鍒嗙被椤跺眰鍏ュ彛 (绾ц仈鍒ゅ喅閫昏緫)
 * 杈撳叆鍙傛暟: dcount   -- 鐩爣鏁?(CFAR/娴嬭杈撳嚭鐐规暟)
 *           pts      -- 鐩爣鐐逛俊鎭粨鏋勪綋鏁扮粍 (TargetPointType, Q8鏍煎紡)
 *           rTh      -- 鐢ㄦ埛鐣岄潰璁惧畾鐨勮窛绂婚槇鍊?(鍗曚綅: cm)
 * 杈撳嚭鍙傛暟: classify -- 杈撳嚭缁撴灉 (0:鏃犳墜鍔? 1:宸︽尌鎵? -1:鍙虫尌鎵?
 * 杩斿洖璇存槑: 鐘舵€佺爜 (RADAR_ALG_SUCCESS 涓烘垚鍔?
 */
int radar_gesture_classify(const int dcount, const TargetPointType *pts, const uint16_t rTh, int *classify)
{
    /* 1. 鎸囬拡鍚堟硶鎬ф鏌?*/
    if ((pts == NULL) || (classify == NULL)) {
        return ALG_NULL_PTR;
    }

    /* 鍒濆鍖栧垽鍐崇粨鏋滀负鏃犳晥鎵嬪娍 */
    *classify = GR_HAND_INVLD;
    
    /* 鐩爣鏁板紓甯告鏌?*/
    if (dcount < 0) {
        return ALG_INVALID_LEN;
    }

    /* 2. 鍏ュ彛鍙傛暟杞崲: 灏嗚緭鍏ョ殑鍘樼背(cm)杞崲涓?Q8 鏍煎紡鐨勭背(m) */
    /* 璁＄畻鍏紡: rTh_m = rTh_cm / 100.0f; Q8 杞崲: rTh_m * 256 */
    /* 鍗? (rTh * 256) / 100 */
    int16_t rThIn_q8 = (int16_t)((rTh * 256) / 100);

    /* 3. 杈圭晫淇濇姢锛氶檺鍒惰瘑鍒窛绂诲湪 [0.05m, 0.3m] 涔嬮棿 */
    /* GR_RTH_IN_MIN_Q8 = 13, GR_RTH_IN_MAX_Q8 = 77 */
    if (rThIn_q8 < GR_RTH_IN_MIN_Q8) {
        rThIn_q8 = GR_RTH_IN_MIN_Q8;
    }
    if (rThIn_q8 > GR_RTH_IN_MAX_Q8) {
        rThIn_q8 = GR_RTH_IN_MAX_Q8;
    }

    int retcode = RADAR_ALG_SUCCESS;

    /* ========================================================
       绾ц仈璇嗗埆閫昏緫 3: 鏈€灏忕獥鍙?(rTh + 0.05m)
       ======================================================== */
    int classifBase3 = GR_HAND_INVLD;
    GrParams paraBase3 = {
        .type          = 0,
        .rTh           = rThIn_q8 + GR_RTH_ADD_Q8,    /* 鍒ゅ喅璺濈闂ㄩ檺 */
        .angleRangeTh  = GR_ATH_RANGE_MIN_Q8,        /* 瑙掑害鏋佸樊闂ㄩ檺 */
        .snrTh         = GR_SNR_VLD_TH_Q8,           /* SNR闂ㄩ檺 */
        .vldPtNumMin   = BLOCK_NVLD_MIN,             /* 5涓偣 */
        .feaWinLen     = FEAR_WIN_LEN,               /* 30甯х獥鍙?*/
        .blockInvldMax = BLOCK_NINVLD_MAX,           /* 2涓棤鏁堢偣 */
        .blockMinNgap  = BLOCK_MIN_NGAP,            /* 2涓偣闂撮殧 */
        .blockFltN0    = BLOCK_FLT_N0,               /* 婊ゆ尝绐楀彛4 */
        .rThSelMin     = 0,                          /* 0m 璧峰鐐?*/
        .rThSelMax     = rThIn_q8 + 13,              /* 0.05m 绐楀彛澧為噺 (0.05*256=12.8) */
        .v0SelMin      = GR_V0_SEL_MIN,              /* 閫熷害娲昏穬闂ㄩ檺 */
    };
    
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase3, g_ptFifo_base3, &classifBase3);
    if (retcode != RADAR_ALG_SUCCESS) return retcode;
    
    if (classifBase3 != GR_HAND_INVLD) {
        *classify = classifBase3;
        return RADAR_ALG_SUCCESS;
    }

    /* ========================================================
       绾ц仈璇嗗埆閫昏緫 2: 涓瓑绐楀彛 (rTh + 0.125m)
       ======================================================== */
    int classifBase2 = GR_HAND_INVLD;
    GrParams paraBase2 = {
        .type          = 0,
        .rTh           = rThIn_q8 + GR_RTH_ADD_Q8,
        .angleRangeTh  = GR_ATH_RANGE_MIN_Q8,
        .snrTh         = GR_SNR_VLD_TH_Q8,
        .vldPtNumMin   = BLOCK_NVLD_MIN,
        .feaWinLen     = FEAR_WIN_LEN,
        .blockInvldMax = BLOCK_NINVLD_MAX,
        .blockMinNgap  = BLOCK_MIN_NGAP,
        .blockFltN0    = BLOCK_FLT_N0,
        .rThSelMin     = 0,
        .rThSelMax     = rThIn_q8 + 32,              /* 0.125m 绐楀彛澧為噺 (0.125*256=32) */
        .v0SelMin      = GR_V0_SEL_MIN,
    };
    
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase2, g_ptFifo_base2, &classifBase2);
    if (retcode != RADAR_ALG_SUCCESS) return retcode;
    
    if (classifBase2 != GR_HAND_INVLD) {
        *classify = classifBase2;
        return RADAR_ALG_SUCCESS;
    }

    /* ========================================================
       绾ц仈璇嗗埆閫昏緫 1: 鏈€澶х獥鍙?(rTh + 0.20m)
       ======================================================== */
    int classifBase1 = GR_HAND_INVLD;
    GrParams paraBase1 = {
        .type          = 0,
        .rTh           = rThIn_q8 + GR_RTH_ADD_Q8,
        .angleRangeTh  = GR_ATH_RANGE_MIN_Q8,
        .snrTh         = GR_SNR_VLD_TH_Q8,
        .vldPtNumMin   = BLOCK_NVLD_MIN,
        .feaWinLen     = FEAR_WIN_LEN,
        .blockInvldMax = BLOCK_NINVLD_MAX,
        .blockMinNgap  = BLOCK_MIN_NGAP,
        .blockFltN0    = BLOCK_FLT_N0,
        .rThSelMin     = 0,
        .rThSelMax     = rThIn_q8 + GR_RTH_FIFO_ADD_Q8, /* 0.20m 绐楀彛澧為噺 */
        .v0SelMin      = GR_V0_SEL_MIN,
    };
    
    retcode = udsf_radar_gesture_classify_base(dcount, pts, paraBase1, g_ptFifo_base, &classifBase1);
    if (retcode != RADAR_ALG_SUCCESS) return retcode;
    
    if (classifBase1 != GR_HAND_INVLD) {
        *classify = classifBase1;
        return RADAR_ALG_SUCCESS;
    }

    /* 鑻ヤ笁涓骇鑱斿潎鏈瘑鍒嚭鏈夋晥鎵嬪娍锛屽垯淇濇寔 GR_HAND_INVLD (0) */
    return RADAR_ALG_SUCCESS;
}

#endif

#endif



