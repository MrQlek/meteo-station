#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "lang_utils.h"
#include "results.h"

typedef struct {
    uint8_t * const storage;
    uint16_t head;
    uint16_t tail;
    const uint16_t maxlen;
} ring_buffer_t;

#define NELEMS(a) ((int)(sizeof(a)/sizeof((a)[0])))
#define RING_BUFFER(_storage) (ring_buffer_t) { \
    .storage = _storage, \
    .head = 0, \
    .tail = 0, \
    .maxlen = NELEMS(_storage) \
}

extern result_t ring_buffer_push(MUT ring_buffer_t * const buffer, uint8_t data);
extern byte_result_t ring_buffer_pop(MUT ring_buffer_t * const buffer);
extern uint16_t ring_buffer_left_space(const ring_buffer_t * const buffer);

#endif // !__RING_BUFFER_H__