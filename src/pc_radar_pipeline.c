#include "pc_radar_algo_float_v2.h"

#include "infohand_proto.h"
#include "radar_algo.h"
#include "radar_svm.h"
#include "pc_radar_debug.h"

typedef struct {
    uint32_t fhead;
    uint32_t findex;
    uint16_t cpack;
    uint16_t tpack;
    uint8_t dtype;
    uint8_t santx;
    uint16_t crc16;
    uint16_t points;
    uint16_t chrips;
    uint32_t dbytes;
} pc_infohand_header_t;

static int hex_value(int ch)
{
    if ((ch >= '0') && (ch <= '9')) {
        return ch - '0';
    }
    if ((ch >= 'a') && (ch <= 'f')) {
        return ch - 'a' + 10;
    }
    if ((ch >= 'A') && (ch <= 'F')) {
        return ch - 'A' + 10;
    }
    return -1;
}

static int load_file_bytes(const char *path, uint8_t **buffer, size_t *size)
{
    FILE *fp;
    long file_size;
    uint8_t *data;

    if ((path == NULL) || (buffer == NULL) || (size == NULL)) {
        return -1;
    }

    fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file: %s\n", path);
        return -1;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }

    file_size = ftell(fp);
    if (file_size < 0) {
        fclose(fp);
        return -1;
    }

    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }

    data = (uint8_t *)malloc((size_t)file_size);
    if (data == NULL) {
        fclose(fp);
        return -1;
    }

    if (fread(data, 1, (size_t)file_size, fp) != (size_t)file_size) {
        free(data);
        fclose(fp);
        return -1;
    }

    fclose(fp);
    *buffer = data;
    *size = (size_t)file_size;
    return 0;
}

static int decode_hex_text(const uint8_t *text, size_t text_size, uint8_t **out, size_t *out_size)
{
    uint8_t *bytes;
    size_t capacity = (text_size / 2U) + 1U;
    size_t count = 0;
    int high = -1;

    bytes = (uint8_t *)malloc(capacity);
    if (bytes == NULL) {
        return -1;
    }

    for (size_t i = 0; i < text_size; ++i) {
        const int hv = hex_value((int)text[i]);
        if (hv < 0) {
            continue;
        }
        if (high < 0) {
            high = hv;
        } else {
            bytes[count++] = (uint8_t)((high << 4) | hv);
            high = -1;
        }
    }

    if (count == 0) {
        free(bytes);
        return -1;
    }

    *out = bytes;
    *out_size = count;
    return 0;
}

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static void parse_infohand_header(const uint8_t *p, pc_infohand_header_t *h)
{
    h->fhead = read_le32(p + 0);
    h->findex = read_le32(p + 4);
    h->cpack = read_le16(p + 8);
    h->tpack = read_le16(p + 10);
    h->dtype = p[12];
    h->santx = p[13];
    h->crc16 = read_le16(p + 14);
    h->points = read_le16(p + 16);
    h->chrips = read_le16(p + 18);
    h->dbytes = read_le32(p + 20);
}

static int extract_2dfft_payload(const uint8_t *data, size_t size, uint8_t **payload, size_t *payload_size)
{
    const size_t header_size = 24U;
    const size_t tail_size = 4U;
    const size_t expected_payload = sizeof(radar_fftxd_type_t);

    for (size_t i = 0; i + header_size + tail_size <= size; ++i) {
        pc_infohand_header_t header;
        uint32_t tail;

        if (read_le32(data + i) != IFPROTO_FRAME_HEAD) {
            continue;
        }

        parse_infohand_header(data + i, &header);
        if (header.fhead != IFPROTO_FRAME_HEAD) {
            continue;
        }

        if ((size - i) < (header_size + (size_t)header.dbytes + tail_size)) {
            continue;
        }

        tail = read_le32(data + i + header_size + (size_t)header.dbytes);
        if (tail != IFPROTO_FRAME_TAIL) {
            continue;
        }

        if (header.dtype != 3U) {
            continue;
        }

        if ((header.santx != 2U) || (header.points != 20U) || (header.chrips != 64U)) {
            continue;
        }

        if ((size_t)header.dbytes != expected_payload) {
            continue;
        }

        *payload = (uint8_t *)malloc(expected_payload);
        if (*payload == NULL) {
            return -1;
        }

        memcpy(*payload, data + i + header_size, expected_payload);
        *payload_size = expected_payload;
        return 0;
    }

    return -1;
}

static int load_2dfft_frame(const char *path, radar_fftxd_type_t *frame)
{
    uint8_t *raw = NULL;
    uint8_t *decoded = NULL;
    uint8_t *payload = NULL;
    size_t raw_size = 0;
    size_t decoded_size = 0;
    size_t payload_size = 0;
    int status = -1;

    if (load_file_bytes(path, &raw, &raw_size) != 0) {
        return -1;
    }

    if ((raw_size == sizeof(*frame)) && (raw[0] != '5')) {
        memcpy(frame, raw, sizeof(*frame));
        status = 0;
        goto cleanup;
    }

    if (decode_hex_text(raw, raw_size, &decoded, &decoded_size) != 0) {
        goto cleanup;
    }

    if (extract_2dfft_payload(decoded, decoded_size, &payload, &payload_size) != 0) {
        goto cleanup;
    }

    if (payload_size != sizeof(*frame)) {
        goto cleanup;
    }

    memcpy(frame, payload, sizeof(*frame));
    status = 0;

cleanup:
    free(payload);
    free(decoded);
    free(raw);
    return status;
}

int pc_radar_run_from_2dfft_file(const char *path)
{
    radar_fftxd_type_t frame;
    DetectResultType *result = NULL;
    TargetPointType *points = NULL;
    uint32_t ts;
    uint32_t te;
    int dcount = 0;
    int retcode = 0;
    int classify = 0;
    const uint16_t sGesRangeThrCM = 15;
    const uint16_t sBrightScreenThrCM = 100;

    if (load_2dfft_frame(path, &frame) != 0) {
        fprintf(stderr, "Failed to parse 2DFFT input: %s\n", path);
        return 1;
    }

    ts = bsp_ticks();
    dcount = radar_execute_cfar(&frame, &result);
    te = bsp_ticks();
    // pr_info("radar_execute_cfar cast    : %2u ms", bsp_time_cast(te, ts));
    PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount);

    ts = bsp_ticks();
    dcount = radar_dbf_estimation(&frame, result, dcount, &points);
    te = bsp_ticks();
    // pr_info("radar_dbf_estimation cast  : %2u ms", bsp_time_cast(te, ts));
    PC_RADAR_DEBUG_TARGET_POINTS(points, dcount);

    if ((dcount > 0) && (points != NULL)) {
        radar_algo_target_hook(POINT_TYPE_CLUSTER, dcount, points, sBrightScreenThrCM);
    }

    ts = bsp_ticks();
    retcode = radar_gesture_classify(dcount, points, sGesRangeThrCM, &classify);
    te = bsp_ticks();
    pr_info("radar_gesture_classify cast: %2u ms, classify:%d", bsp_time_cast(te, ts), classify);
    PC_RADAR_DEBUG_GESTURE_RESULT(retcode, classify);

    if (retcode == 0) {
        radar_algo_gesture_hook(classify);
    }

    return 0;
}

int pc_radar_run_from_1dfft_file(const char *path)
{
    (void)path;
    fprintf(stderr, "1dfft mode is reserved and not implemented yet.\n");
    return 1;
}

int pc_radar_run_from_adc_file(const char *path)
{
    (void)path;
    fprintf(stderr, "adc mode is reserved and not implemented yet.\n");
    return 1;
}



