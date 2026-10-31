#ifndef BSP_DEBUG_LOG_H
#define BSP_DEBUG_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define DEBUG_LOG_VERSION 1U
#define DEBUG_LOG_MAX_PAYLOAD 96U
#define DEBUG_LOG_MAX_FRAME 104U
#define DEBUG_LOG_EVENT_CAPACITY 8U

typedef struct
{
  uint32_t time_ms;
  uint16_t sequence;
  uint8_t stage;
  uint8_t flags;
  uint8_t limited;
  uint8_t reserved;
  float reference;
  float position;
  float omega;
  float actual_current;
  float requested_current;
  float executed_current;
  float model_inertia;
  float model_fitness;
} DebugLogSample;

typedef struct
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t length;
  uint16_t sequence;
} DebugLogHeader;

_Static_assert(sizeof(DebugLogHeader) == 6U,
               "DebugLogHeader wire size must stay 6 bytes");

_Static_assert(sizeof(DebugLogSample) == 44U,
               "DebugLogSample wire size must stay 44 bytes");

// 调试记录通道配置
typedef struct
{
  UART_HandleTypeDef *huart;
  bool enabled;
  bool binary_enabled;
  bool text_enabled;
  uint16_t sample_period;
  uint32_t baudrate;
} DebugLogConfig;

// 待发送的英文事件
typedef struct
{
  uint8_t length;
  uint8_t text[DEBUG_LOG_MAX_PAYLOAD];
} DebugLogEvent;

// 调试日志运行状态
typedef struct
{
  DebugLogConfig config;
  uint16_t sequence;
  uint32_t dropped_frames;
  uint32_t dropped_text_frames;
  uint32_t sample_count;
  uint32_t text_count;
  uint32_t start_time_ms;
  uint32_t stop_time_ms;
  bool initialized;
  bool busy;
  bool recording;
  bool stopping;
  bool stop_sent;
  uint8_t event_head;
  uint8_t event_count;
  DebugLogEvent events[DEBUG_LOG_EVENT_CAPACITY];
  uint8_t tx_frame[DEBUG_LOG_MAX_FRAME];
} DebugLogObject;

bool DebugLogInit(DebugLogObject *log, const DebugLogConfig *config);
void DebugLogStart(DebugLogObject *log, uint32_t time_ms);
void DebugLogStartNamed(DebugLogObject *log, uint32_t time_ms,
                        const char *name);
void DebugLogStop(DebugLogObject *log, uint32_t time_ms);
void DebugLogProcess(DebugLogObject *log);
void DebugLogSampleWrite(DebugLogObject *log, const DebugLogSample *sample);
void DebugLogInfo(DebugLogObject *log, uint32_t time_ms, const char *message);
void DebugLogError(DebugLogObject *log, uint32_t time_ms, const char *message);

#endif // BSP_DEBUG_LOG_H
