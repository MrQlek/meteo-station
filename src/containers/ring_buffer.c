#include "asserts.h"

#include "ring_buffer.h"

result_t ring_buffer_push(MUT ring_buffer_t * const buffer, uint8_t data) {
    assert_pointer(buffer);

    uint16_t next = buffer->head + 1;

    if(next >= buffer->maxlen) {
        next = 0;
    }

    if(next == buffer->tail) {
        return RESULT_GENERIC_ERROR;
    }

    buffer->storage[buffer->head] = data;
    buffer->head = next;

    return RESULT_OK;
}

byte_result_t ring_buffer_pop(MUT ring_buffer_t * const buffer) {
    assert_pointer(buffer);

    if(buffer->head == buffer->tail) {
        return BYTE_RESULT(0, RESULT_GENERIC_ERROR); 
    }

    uint16_t next = buffer->tail + 1;
    if(next >= buffer->maxlen) {
        next = 0;
    }

    uint8_t data = buffer->storage[buffer->tail];
    buffer->tail = next;
    return BYTE_RESULT(data, RESULT_OK);
}

uint16_t ring_buffer_left_space(const ring_buffer_t * const buffer) {
    if(buffer->tail > buffer->head) {
        return buffer->tail - buffer->head;
    }

    //TODO check if this is correct
    return (uint16_t)(
        ((uint16_t)(buffer->maxlen - buffer->head) + buffer->tail) - 1);
}