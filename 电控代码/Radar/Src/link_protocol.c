#include "link_protocol.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

uint16_t link_crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U ?
                  (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t link_cobs_encode(const uint8_t *input, size_t length,
                        uint8_t *output, size_t capacity)
{
    size_t read_index = 0U;
    size_t write_index = 1U;
    size_t code_index = 0U;
    uint8_t code = 1U;
    if (input == NULL || output == NULL || capacity == 0U) return 0U;
    while (read_index < length) {
        if (input[read_index] == 0U) {
            if (code_index >= capacity) return 0U;
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
            if (write_index > capacity) return 0U;
            ++read_index;
        } else {
            if (write_index >= capacity) return 0U;
            output[write_index++] = input[read_index++];
            ++code;
            if (code == 0xFFU) {
                output[code_index] = code;
                code = 1U;
                code_index = write_index++;
                if (write_index > capacity) return 0U;
            }
        }
    }
    if (code_index >= capacity) return 0U;
    output[code_index] = code;
    return write_index;
}

size_t link_cobs_decode(const uint8_t *input, size_t length,
                        uint8_t *output, size_t capacity)
{
    size_t read_index = 0U;
    size_t write_index = 0U;
    if (input == NULL || output == NULL) return 0U;
    while (read_index < length) {
        uint8_t code = input[read_index++];
        uint8_t copy;
        if (code == 0U || read_index + (size_t)code - 1U > length) return 0U;
        for (copy = 1U; copy < code; ++copy) {
            if (write_index >= capacity) return 0U;
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFFU && read_index < length) {
            if (write_index >= capacity) return 0U;
            output[write_index++] = 0U;
        }
    }
    return write_index;
}

size_t link_encode_message(uint8_t node_id, uint8_t type, uint16_t sequence,
                           const uint8_t *payload, uint16_t payload_length,
                           uint8_t *output, size_t capacity)
{
    uint8_t raw[LINK_MAX_RAW_FRAME];
    size_t raw_length;
    size_t encoded_length;
    uint16_t crc;
    if (node_id > LINK_MAX_NODE_ID || payload_length > LINK_MAX_PAYLOAD ||
        output == NULL || capacity < 2U ||
        (payload_length > 0U && payload == NULL)) return 0U;
    raw[0] = 'L';
    raw[1] = 'R';
    raw[2] = LINK_PROTOCOL_VERSION;
    raw[3] = node_id;
    raw[4] = type;
    write_u16(&raw[5], sequence);
    write_u16(&raw[7], payload_length);
    if (payload_length > 0U) memcpy(&raw[9], payload, payload_length);
    raw_length = 9U + payload_length;
    crc = link_crc16_ccitt(raw, raw_length);
    write_u16(&raw[raw_length], crc);
    raw_length += 2U;
    encoded_length = link_cobs_encode(raw, raw_length, output, capacity - 1U);
    if (encoded_length == 0U) return 0U;
    output[encoded_length++] = 0U;
    return encoded_length;
}

void link_decoder_init(link_decoder_t *decoder)
{
    if (decoder != NULL) memset(decoder, 0, sizeof(*decoder));
}

void link_decoder_reset_partial(link_decoder_t *decoder)
{
    if (decoder != NULL) {
        decoder->length = 0U;
        decoder->discarding = false;
    }
}

static bool decode_message(const uint8_t *encoded, uint16_t encoded_length,
                           link_message_t *message)
{
    uint8_t raw[LINK_MAX_RAW_FRAME];
    size_t raw_length = link_cobs_decode(encoded, encoded_length, raw, sizeof(raw));
    uint16_t payload_length;
    uint16_t expected_crc;
    if (raw_length < 11U || raw[0] != 'L' || raw[1] != 'R' ||
        raw[2] != LINK_PROTOCOL_VERSION || raw[3] > LINK_MAX_NODE_ID) return false;
    payload_length = read_u16(&raw[7]);
    if (payload_length > LINK_MAX_PAYLOAD || raw_length != 11U + payload_length)
        return false;
    expected_crc = read_u16(&raw[9U + payload_length]);
    if (link_crc16_ccitt(raw, 9U + payload_length) != expected_crc) return false;
    message->node_id = raw[3];
    message->type = raw[4];
    message->sequence = read_u16(&raw[5]);
    message->payload_length = payload_length;
    if (payload_length > 0U) memcpy(message->payload, &raw[9], payload_length);
    return true;
}

void link_decoder_feed(link_decoder_t *decoder, const uint8_t *data, size_t length,
                       link_message_callback_t callback, void *context)
{
    size_t index;
    if (decoder == NULL || (length > 0U && data == NULL)) return;
    for (index = 0U; index < length; ++index) {
        if (data[index] == 0U) {
            if (decoder->discarding) {
                decoder->discarding = false;
            } else if (decoder->length > 0U) {
                link_message_t message;
                if (decode_message(decoder->encoded, decoder->length, &message)) {
                    ++decoder->good_frames;
                    if (callback != NULL) callback(&message, context);
                } else {
                    ++decoder->bad_frames;
                }
            }
            decoder->length = 0U;
        } else if (decoder->discarding) {
            continue;
        } else if (decoder->length < sizeof(decoder->encoded)) {
            decoder->encoded[decoder->length++] = data[index];
        } else {
            ++decoder->bad_frames;
            decoder->length = 0U;
            decoder->discarding = true;
        }
    }
}
