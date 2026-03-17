#ifndef PC_INFOHAND_PROTO_INCLUDE
#define PC_INFOHAND_PROTO_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IFPROTO_FRAME_HEAD (0x55AA55BBU)
#define IFPROTO_FRAME_TAIL (0x55CC55DDU)

typedef enum {
    CMD_RAWADC = 0x00,
    CMD_1DFFT = 0x01,
    CMD_2DFFT = 0x02,
    CMD_CLUSTER = 0x03,
    CMD_TRACKER = 0x04,
    CMD_RMD = 0x05,
    CMD_DBSCAN = 0x06,
    CMD_GESTURE = 0x07
} CommandType;

#ifdef __cplusplus
}
#endif

#endif
