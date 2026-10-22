/*
 * tlm_proto.c - framing, CRC and streaming parser. No hardware dependencies,
 * so it also compiles on the PC for tests (see test/proto_selftest.c).
 */
#include "tlm_proto.h"
#include <string.h>

enum { S_SYNC0, S_SYNC1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 };

uint16_t tlm_crc16(const uint8_t *data, size_t n, uint16_t crc)
{
    while (n--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t tlm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out)
{
    out[0] = TLM_SYNC0;
    out[1] = TLM_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len)
        memcpy(&out[4], payload, len);
    uint16_t crc = tlm_crc16(&out[2], (size_t)len + 2u, 0xFFFFu);
    out[4 + len] = (uint8_t)(crc & 0xFF);
    out[5 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + TLM_OVERHEAD;
}

void tlm_parser_init(tlm_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = S_SYNC0;
}

int tlm_parser_feed(tlm_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC0:
        if (b == TLM_SYNC0) p->state = S_SYNC1;
        break;
    case S_SYNC1:
        if (b == TLM_SYNC1)      p->state = S_TYPE;
        else if (b != TLM_SYNC0) p->state = S_SYNC0;   /* 0xA5 0xA5 0x5A still syncs */
        break;
    case S_TYPE:
        p->type = b;
        p->state = S_LEN;
        break;
    case S_LEN:
        if (b > TLM_MAX_PAYLOAD) { p->state = S_SYNC0; break; }
        p->len = b;
        p->idx = 0;
        p->state = b ? S_PAYLOAD : S_CRC0;
        break;
    case S_PAYLOAD:
        p->buf[p->idx++] = b;
        if (p->idx >= p->len) p->state = S_CRC0;
        break;
    case S_CRC0:
        p->crc_rx = b;
        p->state = S_CRC1;
        break;
    case S_CRC1: {
        p->crc_rx |= (uint16_t)b << 8;
        p->state = S_SYNC0;
        uint8_t hdr[2] = { p->type, p->len };
        uint16_t crc = tlm_crc16(hdr, 2, 0xFFFFu);
        crc = tlm_crc16(p->buf, p->len, crc);
        if (crc == p->crc_rx) { p->frames_ok++; return 1; }
        p->crc_errors++;
        break;
    }
    default:
        p->state = S_SYNC0;
    }
    return 0;
}
