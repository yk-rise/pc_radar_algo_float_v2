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

typedef int (*pc_2dfft_frame_handler_t)(const radar_fftxd_type_t *frame, const pc_infohand_header_t *header, int frame_index, void *ctx);

static int for_each_2dfft_frame_from_buffer(const uint8_t *data, size_t size, pc_2dfft_frame_handler_t handler, void *ctx, int *frame_count)
{
    const size_t header_size = 24U;
    const size_t tail_size = 4U;
    const size_t expected_payload = sizeof(radar_fftxd_type_t);
    int count = 0;

    if ((data == NULL) || (handler == NULL)) {
        return -1;
    }

    for (size_t i = 0; i + header_size + tail_size <= size; ++i) {
        pc_infohand_header_t header;
        uint32_t tail;
        radar_fftxd_type_t frame;

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
            i += header_size + (size_t)header.dbytes + tail_size - 1U;
            continue;
        }

        if ((header.santx != 2U) || (header.points != 20U) || (header.chrips != 64U)) {
            i += header_size + (size_t)header.dbytes + tail_size - 1U;
            continue;
        }

        if ((size_t)header.dbytes != expected_payload) {
            i += header_size + (size_t)header.dbytes + tail_size - 1U;
            continue;
        }

        memcpy(frame, data + i + header_size, expected_payload);
        if (handler(&frame, &header, count, ctx) != 0) {
            return -1;
        }

        count++;
        i += header_size + (size_t)header.dbytes + tail_size - 1U;
    }

    if (frame_count != NULL) {
        *frame_count = count;
    }

    return count > 0 ? 0 : -1;
}

static int for_each_2dfft_frame_from_file(const char *path, pc_2dfft_frame_handler_t handler, void *ctx, int *frame_count)
{
    uint8_t *raw = NULL;
    uint8_t *decoded = NULL;
    size_t raw_size = 0;
    size_t decoded_size = 0;
    int status = -1;

    if (load_file_bytes(path, &raw, &raw_size) != 0) {
        return -1;
    }

    if ((raw_size == sizeof(radar_fftxd_type_t)) && (raw[0] != '5')) {
        radar_fftxd_type_t frame;
        memcpy(frame, raw, sizeof(frame));
        status = handler(&frame, NULL, 0, ctx);
        if ((status == 0) && (frame_count != NULL)) {
            *frame_count = 1;
        }
        free(raw);
        return status;
    }

    if (decode_hex_text(raw, raw_size, &decoded, &decoded_size) != 0) {
        free(raw);
        return -1;
    }

    status = for_each_2dfft_frame_from_buffer(decoded, decoded_size, handler, ctx, frame_count);

    free(decoded);
    free(raw);
    return status;
}

typedef struct {
    int processed_frames;
} pc_run_ctx_t;

static int run_single_2dfft_frame(const radar_fftxd_type_t *frame, const pc_infohand_header_t *header, int frame_index, void *ctx)
{
    DetectResultType *result = NULL;
    TargetPointType *points = NULL;
    uint32_t ts;
    uint32_t te;
    int dcount = 0;
    int retcode = 0;
    int classify = 0;
    const uint16_t sGesRangeThrCM = 15;
    const uint16_t sBrightScreenThrCM = 100;
    pc_run_ctx_t *run_ctx = (pc_run_ctx_t *)ctx;

    if (header != NULL) {
        pr_info("================ Frame %d ================", frame_index);
        pr_info("findex:%u dtype:%u santx:%u points:%u chirps:%u bytes:%u",
            header->findex, header->dtype, header->santx, header->points, header->chrips, header->dbytes);
    } else {
        pr_info("================ Frame %d ================", frame_index);
        pr_info("raw binary 2DFFT frame");
    }

    ts = bsp_ticks();
    dcount = radar_execute_cfar(frame, &result);
    te = bsp_ticks();
    pr_info("radar_execute_cfar cast    : %2u ms", bsp_time_cast(te, ts));
    PC_RADAR_DEBUG_DETECT_RESULTS(result, dcount);

    ts = bsp_ticks();
    dcount = radar_dbf_estimation(frame, result, dcount, &points);
    te = bsp_ticks();
    pr_info("radar_dbf_estimation cast  : %2u ms", bsp_time_cast(te, ts));
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

    if (run_ctx != NULL) {
        run_ctx->processed_frames++;
    }

    return 0;
}


int pc_radar_run_from_2dfft_frame(const radar_fftxd_type_t *frame)
{
    pc_run_ctx_t ctx = {0};

    if (frame == NULL) {
        fprintf(stderr, "2DFFT frame pointer is NULL.\n");
        return 1;
    }

    if (run_single_2dfft_frame(frame, NULL, 0, &ctx) != 0) {
        fprintf(stderr, "Failed to process 2DFFT frame.\n");
        return 1;
    }

    pr_info("processed 2DFFT frames: %d", ctx.processed_frames);
    return 0;
}

int pc_radar_run_from_2dfft_frames(const radar_fftxd_type_t *frames, int frame_count)
{
    pc_run_ctx_t ctx = {0};
    int i;

    if (frames == NULL) {
        fprintf(stderr, "2DFFT frames pointer is NULL.\n");
        return 1;
    }

    if (frame_count <= 0) {
        fprintf(stderr, "2DFFT frame count must be positive.\n");
        return 1;
    }

    for (i = 0; i < frame_count; ++i) {
        if (run_single_2dfft_frame(&frames[i], NULL, i, &ctx) != 0) {
            fprintf(stderr, "Failed to process 2DFFT frame index %d.\n", i);
            return 1;
        }
    }

    pr_info("processed 2DFFT frames: %d", ctx.processed_frames);
    return 0;
}
int pc_radar_run_from_2dfft_file(const char *path)
{
    int frame_count = 0;
    pc_run_ctx_t ctx = {0};

    if (for_each_2dfft_frame_from_file(path, run_single_2dfft_frame, &ctx, &frame_count) != 0) {
        fprintf(stderr, "Failed to parse any complete 2DFFT frame from: %s\n", path);
        return 1;
    }

    pr_info("processed 2DFFT frames: %d", ctx.processed_frames);
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





