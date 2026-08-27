/**
 * @file    ring_buffer.h
 * @brief   Byte ring buffer, useful for UART/referee protocols.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;   /* write index */
    volatile uint16_t tail;   /* read index  */
    volatile uint16_t count;
} RingBuffer_t;

void     RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
bool     RingBuffer_Push(RingBuffer_t *rb, uint8_t byte);
/* 分批推入一段数据;放得下才整段写入并返回 true,放不下整段丢弃返回 false(保帧完整) */
bool     RingBuffer_PushSegment(RingBuffer_t *rb, const uint8_t *data, uint16_t len);
bool     RingBuffer_Pop(RingBuffer_t *rb, uint8_t *byte);
uint16_t RingBuffer_Available(const RingBuffer_t *rb);
bool     RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool     RingBuffer_IsFull(const RingBuffer_t *rb);
void     RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
