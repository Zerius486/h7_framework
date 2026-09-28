#include "bsp_stream.h"

#include "string.h"

/**
 * @brief 初始化流式缓存
 * @param stream 流式缓存对象指针
 * @param buffer 缓存数据指针
 * @param capacity 缓存容量
 */
void StreamBufferInit(StreamBuffer *stream, uint8_t *buffer,
                      uint16_t capacity)
{
  if (stream == NULL)
  {
    return;
  }

  stream->buffer = buffer;
  stream->capacity = capacity;
  stream->length = 0U;
}

/**
 * @brief 向流式缓存追加数据
 * @param stream 流式缓存对象指针
 * @param data 输入数据指针
 * @param length 输入数据长度
 */
void StreamBufferAppend(StreamBuffer *stream, const uint8_t *data,
                        uint16_t length)
{
  if (stream == NULL || stream->buffer == NULL || stream->capacity == 0U ||
      data == NULL || length == 0U)
  {
    return;
  }

  if ((uint32_t)stream->length + length > stream->capacity)
  {
    if (length >= stream->capacity)
    {
      memcpy(stream->buffer, &data[length - stream->capacity],
             stream->capacity);
      stream->length = stream->capacity;
      return;
    }

    uint16_t keep_length = (uint16_t)(stream->capacity - length);
    if (stream->length < keep_length)
    {
      keep_length = stream->length;
    }
    memmove(stream->buffer, &stream->buffer[stream->length - keep_length],
            keep_length);
    memcpy(&stream->buffer[keep_length], data, length);
    stream->length = (uint16_t)(keep_length + length);
    return;
  }

  memcpy(&stream->buffer[stream->length], data, length);
  stream->length = (uint16_t)(stream->length + length);
}

/**
 * @brief 消费流式缓存中的数据
 * @param stream 流式缓存对象指针
 * @param length 消费数据长度
 */
void StreamBufferConsume(StreamBuffer *stream, uint16_t length)
{
  if (stream == NULL || stream->buffer == NULL || length == 0U)
  {
    return;
  }

  if (length >= stream->length)
  {
    stream->length = 0U;
    return;
  }

  memmove(stream->buffer, &stream->buffer[length],
          (uint16_t)(stream->length - length));
  stream->length = (uint16_t)(stream->length - length);
}
