/**
 * @file    mathFunc.c
 * @brief   Small math/encode-decode helpers ported from R2 chassis.
 */
#include "mathFunc.h"
#include <string.h>

void ChangeDataByte(uint8_t *p1, uint8_t *p2)
{
    uint8_t t = *p1;
    *p1 = *p2;
    *p2 = t;
}

int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int32_t res = (int32_t)(((uint32_t)buffer[*index] << 24) |
                            ((uint32_t)buffer[*index + 1] << 16) |
                            ((uint32_t)buffer[*index + 2] << 8) |
                            ((uint32_t)buffer[*index + 3]));
    *index += 4;
    return res;
}

int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int16_t res = (int16_t)(((uint32_t)buffer[*index] << 8) |
                            ((uint32_t)buffer[*index + 1]));
    *index += 2;
    return res;
}

float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s32_from_buffer(buffer, index) / scale;
}

float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s16_from_buffer(buffer, index) / scale;
}

void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index)
{
    uint32_t u = (uint32_t)source;
    buffer[(*index)++] = (uint8_t)(u >> 24U);
    buffer[(*index)++] = (uint8_t)(u >> 16U);
    buffer[(*index)++] = (uint8_t)(u >> 8U);
    buffer[(*index)++] = (uint8_t)u;
}

void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index)
{
    uint16_t u = (uint16_t)source;
    buffer[(*index)++] = (uint8_t)u;
    buffer[(*index)++] = (uint8_t)(u >> 8U);   /* little-endian as the original project */
}

double cvtFloat2Double(float n1, float n2)
{
    struct { float n1; float n2; } s;
    double result;
    s.n1 = n1;
    s.n2 = n2;
    memcpy(&result, &s, sizeof(result));
    return result;
}

float uint2float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

u16 float2uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (u16)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Lerp(float start, float end, float t)
{
    if (t > 1.0f) t = 1.0f;
    else if (t < 0.0f) t = 0.0f;
    return (end - start) * t + start;
}

void Rotate(float *x, float *y, float x0, float y0, float a)
{
    float x1 = *x;
    float y1 = *y;
    float rad = a * PI / 180.0f;
    *x = (x1 - x0) * cosf(rad) - (y1 - y0) * sinf(rad) + x0;
    *y = (y1 - y0) * cosf(rad) + (x1 - x0) * sinf(rad) + y0;
}

s16 MSG_Byte2Int16(uint8_t *buff, uint8_t i)
{
    return (s16)((uint16_t)buff[i + 1] << 8 | (uint16_t)buff[i]);
}

int MSG_Byte2Int32(uint8_t *buff, uint8_t i)
{
    return (int)((uint32_t)buff[i + 3] << 24 |
                 (uint32_t)buff[i + 2] << 16 |
                 (uint32_t)buff[i + 1] << 8 |
                 (uint32_t)buff[i]);
}

void MSG_Int162Byte(s16 data, uint8_t *buff, uint8_t i)
{
    uint16_t u = (uint16_t)data;
    buff[i] = (uint8_t)(u & 0xffU);
    buff[i + 1] = (uint8_t)(u >> 8U);
}

void MSG_Int322Byte(int data, uint8_t *buff, uint8_t i)
{
    uint32_t u = (uint32_t)data;
    buff[i] = (uint8_t)(u & 0xffU);
    buff[i + 1] = (uint8_t)((u >> 8U) & 0xffU);
    buff[i + 2] = (uint8_t)((u >> 16U) & 0xffU);
    buff[i + 3] = (uint8_t)(u >> 24U);
}
