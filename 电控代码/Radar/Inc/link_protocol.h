#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_PROTOCOL_VERSION 6U
#define LINK_BROADCAST_NODE_ID 0U
#define LINK_DEFAULT_NODE_ID 1U
#define LINK_MAX_NODE_ID 254U
#define LINK_MAX_PAYLOAD 224U
#define LINK_MAX_RAW_FRAME (9U + LINK_MAX_PAYLOAD + 2U)
#define LINK_MAX_ENCODED_FRAME (LINK_MAX_RAW_FRAME + LINK_MAX_RAW_FRAME / 254U + 2U)

typedef enum {
    LINK_CMD_HELLO = 0x01,
    LINK_CMD_HEARTBEAT = 0x02,
    LINK_CMD_START = 0x03,
    LINK_CMD_STOP = 0x04,
    LINK_CMD_SELECT_ZONE = 0x05,
    LINK_CMD_SET_POSE = 0x06,
    LINK_CMD_SCAN_LOCKED = 0x07,
    LINK_CMD_SINGLE_MEASURE = 0x08,
    LINK_CMD_CALIBRATE = 0x09,
    LINK_CMD_CLEAR = 0x0A,
    LINK_CMD_RESET_CALIBRATION = 0x0B,
    LINK_CMD_SAVE_POSE = 0x11,
    LINK_CMD_GET_PERSISTED_STATE = 0x13,
    LINK_CMD_PLAN_PATH = 0x14,
    LINK_CMD_SET_NODE_ID = 0x17,
    LINK_CMD_SET_POINT_CLOUD_OUTPUT = 0x19,
    LINK_EVT_HELLO = 0x81,
    LINK_EVT_ACK = 0x82,
    LINK_EVT_STATUS = 0x83,
    LINK_EVT_SCAN_BEGIN = 0x84,
    LINK_EVT_SCAN_POINTS = 0x85,
    LINK_EVT_SCAN_END = 0x86,
    LINK_EVT_RESULT = 0x87,
    LINK_EVT_CALIBRATION = 0x88,
    LINK_EVT_STATS = 0x89,
    LINK_EVT_ERROR = 0x8A,
    LINK_EVT_OBSTACLE_GRID = 0x8D,
    LINK_EVT_PERSISTED_STATE = 0x8E,
    LINK_EVT_PATH_RESULT = 0x8F
} link_public_message_type_t;

typedef struct {
    uint8_t node_id;
    uint8_t type;
    uint16_t sequence;
    uint16_t payload_length;
    uint8_t payload[LINK_MAX_PAYLOAD];
} link_message_t;

typedef void (*link_message_callback_t)(const link_message_t *message, void *context);

typedef struct {
    uint8_t encoded[LINK_MAX_ENCODED_FRAME];
    uint16_t length;
    bool discarding;
    uint32_t good_frames;
    uint32_t bad_frames;
} link_decoder_t;

uint16_t link_crc16_ccitt(const uint8_t *data, size_t length);
size_t link_cobs_encode(const uint8_t *input, size_t length,
                        uint8_t *output, size_t capacity);
size_t link_cobs_decode(const uint8_t *input, size_t length,
                        uint8_t *output, size_t capacity);
size_t link_encode_message(uint8_t node_id, uint8_t type, uint16_t sequence,
                           const uint8_t *payload, uint16_t payload_length,
                           uint8_t *output, size_t capacity);
void link_decoder_init(link_decoder_t *decoder);
void link_decoder_reset_partial(link_decoder_t *decoder);
void link_decoder_feed(link_decoder_t *decoder, const uint8_t *data, size_t length,
                       link_message_callback_t callback, void *context);

#endif
