/**
 * @file    ring_buffer.c
 * @brief   Byte ring buffer implementation.
 */
#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size)
{
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool RingBuffer_Push(RingBuffer_t *rb, uint8_t byte)
{
    if (RingBuffer_IsFull(rb)) {
        return false;
    }
    rb->buffer[rb->head] = byte;
    rb->head = (uint16_t)((rb->head + 1U) % rb->size);
    rb->count = (uint16_t)(rb->count + 1U);
    return true;
}

bool RingBuffer_PushSegment(RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    /* 放不下整段丢弃,避免解析器看到半帧导致误解析 */
    if (len > (uint16_t)(rb->size - rb->count)) {
        return false;
    }
    for (i = 0U; i < len; i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (uint16_t)((rb->head + 1U) % rb->size);
    }
    rb->count = (uint16_t)(rb->count + len);
    return true;
}

bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *byte)
{
    if (RingBuffer_IsEmpty(rb)) {
        return false;
    }
    *byte = rb->buffer[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % rb->size);
    rb->count = (uint16_t)(rb->count - 1U);
    return true;
}

uint16_t RingBuffer_Available(const RingBuffer_t *rb)
{
    return rb->count;
}

bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return (rb->count == 0U);
}

bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    return (rb->count >= rb->size);
}

void RingBuffer_Flush(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}
