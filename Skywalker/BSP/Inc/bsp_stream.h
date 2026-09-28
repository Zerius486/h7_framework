#ifndef BSP_STREAM_H
#define BSP_STREAM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
} StreamBuffer;

void StreamBufferInit(StreamBuffer *stream, uint8_t *buffer,
                      uint16_t capacity);
void StreamBufferAppend(StreamBuffer *stream, const uint8_t *data,
                        uint16_t length);
void StreamBufferConsume(StreamBuffer *stream, uint16_t length);

#endif // BSP_STREAM_H
