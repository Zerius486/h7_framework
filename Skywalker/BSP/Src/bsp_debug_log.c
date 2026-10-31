#include "bsp_debug_log.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "bsp_uart.h"

enum
{
  kDebugLogMagic = 0xA5,
  kDebugLogTypeSample = 1,
  kDebugLogTypeText = 2,
};

/**
 * @brief 计算调试帧校验值
 * @param data 帧数据指针
 * @param length 帧长度
 * @return 校验值
 */
static uint16_t DebugLogCrc(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1) ^ 0xA001U)
                             : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

/**
 * @brief 提交调试二进制帧，不修改序号及统计
 * @param log 日志对象指针
 * @param type 帧类型
 * @param payload 负载指针
 * @param length 负载长度
 * @param sequence 帧序号
 * @return 是否成功提交
 */
static bool DebugLogFrame(DebugLogObject *log, uint8_t type,
                          const uint8_t *payload, uint8_t length,
                          uint16_t sequence)
{
  uint8_t *frame = log->tx_frame;
  DebugLogHeader header = {.magic = kDebugLogMagic,
                           .version = DEBUG_LOG_VERSION,
                           .type = type,
                           .length = length,
                           .sequence = sequence};
  memcpy(frame, &header, sizeof(header));
  memcpy(&frame[sizeof(header)], payload, length);
  uint16_t crc = DebugLogCrc(frame, sizeof(header) + length);
  frame[sizeof(header) + length] = (uint8_t)(crc & 0xFFU);
  frame[sizeof(header) + length + 1U] = (uint8_t)(crc >> 8);
  log->busy = true;
  bool sent = UartTransmitTry(log->config.huart, frame,
                              (uint16_t)(sizeof(header) + length + 2U));
  log->busy = false;
  return sent;
}

/**
 * @brief 在UART空闲时发送一个事件或结束摘要
 * @param log 日志对象指针，须由同一任务周期调用
 */
void DebugLogProcess(DebugLogObject *log)
{
  if (log == NULL || !log->initialized || !log->config.enabled ||
      !log->recording || log->busy || !UartTransmitReady(log->config.huart))
  {
    return;
  }
  if (log->event_count > 0U)
  {
    DebugLogEvent *event = &log->events[log->event_head];
    if (DebugLogFrame(log, kDebugLogTypeText, event->text, event->length,
                       log->sequence))
    {
      log->sequence++;
      log->text_count++;
      log->event_head = (log->event_head + 1U) % DEBUG_LOG_EVENT_CAPACITY;
      log->event_count--;
    }
    return;
  }
  if (!log->stopping)
  {
    return;
  }
  if (!log->config.text_enabled || log->stop_sent)
  {
    log->recording = false;
    log->stopping = false;
    return;
  }
  char text[DEBUG_LOG_MAX_PAYLOAD];
  int length = snprintf(text, sizeof(text),
                        "[%lu] INFO RUN_STOP DROPPED=%lu TEXT_DROPPED=%lu\r\n",
                        (unsigned long)log->stop_time_ms,
                        (unsigned long)log->dropped_frames,
                        (unsigned long)log->dropped_text_frames);
  if (length > 0 && length < (int)sizeof(text) &&
      DebugLogFrame(log, kDebugLogTypeText, (const uint8_t *)text,
                      (uint8_t)length, log->sequence))
  {
    log->sequence++;
    log->text_count++;
    log->stop_sent = true;
  }
}

/**
 * @brief 缓存英文事件，满时淘汰最早事件并计数
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 * @param level 英文日志级别
 * @param message 英文消息
 */
static void DebugLogText(DebugLogObject *log, uint32_t time_ms,
                         const char *level, const char *message)
{
  if (log == NULL || message == NULL || !log->initialized ||
      !log->config.enabled || !log->recording || log->stopping ||
      !log->config.text_enabled || log->busy)
  {
    return;
  }
  DebugLogProcess(log);
  char text[DEBUG_LOG_MAX_PAYLOAD];
  int length = snprintf(text, sizeof(text), "[%lu] %s %s\r\n",
                        (unsigned long)time_ms, level, message);
  if (length <= 0 || length >= (int)sizeof(text))
  {
    log->sequence++;
    log->dropped_frames++;
    log->dropped_text_frames++;
    return;
  }
  if (log->event_count == DEBUG_LOG_EVENT_CAPACITY)
  {
    log->event_head = (log->event_head + 1U) % DEBUG_LOG_EVENT_CAPACITY;
    log->event_count--;
    log->sequence++;
    log->dropped_frames++;
    log->dropped_text_frames++;
  }
  uint8_t index =
      (log->event_head + log->event_count) % DEBUG_LOG_EVENT_CAPACITY;
  log->events[index].length = (uint8_t)length;
  memcpy(log->events[index].text, text, (size_t)length);
  log->event_count++;
  DebugLogProcess(log);
}

/**
 * @brief 初始化调试记录
 * @param log 日志对象指针
 * @param config 调试配置指针
 * @return 是否成功
 */
bool DebugLogInit(DebugLogObject *log, const DebugLogConfig *config)
{
  if (log == NULL || config == NULL || config->sample_period == 0U ||
      (config->enabled && config->huart == NULL) ||
      (config->enabled && !config->binary_enabled && !config->text_enabled))
  {
    return false;
  }
  memset(log, 0, sizeof(*log));
  log->config = *config;
  if (log->config.enabled &&
      log->config.huart->Init.BaudRate != log->config.baudrate)
  {
    return false;
  }
  log->initialized = true;
  return true;
}

/**
 * @brief 开始一次调试记录
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 */
void DebugLogStart(DebugLogObject *log, uint32_t time_ms)
{
  DebugLogStartNamed(log, time_ms, "MANUAL");
}

/**
 * @brief 使用英文运行名称开始一次调试记录
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 * @param name 英文运行名称
 */
void DebugLogStartNamed(DebugLogObject *log, uint32_t time_ms,
                        const char *name)
{
  if (log != NULL && log->initialized)
  {
    if (!log->config.enabled || log->recording)
    {
      return;
    }
    log->start_time_ms = time_ms;
    log->sample_count = 0U;
    log->text_count = 0U;
    log->dropped_frames = 0U;
    log->dropped_text_frames = 0U;
    log->sequence = 0U;
    log->busy = false;
    log->recording = true;
    log->stopping = false;
    log->stop_sent = false;
    log->event_head = 0U;
    log->event_count = 0U;
    char message[DEBUG_LOG_MAX_PAYLOAD];
    (void)snprintf(message, sizeof(message), "RUN_START MODE=%s",
                   name != NULL ? name : "UNKNOWN");
    DebugLogInfo(log, time_ms, message);
  }
}

/**
 * @brief 停止接收采样，由周期处理发送剩余事件及摘要
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 */
void DebugLogStop(DebugLogObject *log, uint32_t time_ms)
{
  if (log != NULL && log->initialized && log->config.enabled &&
      log->recording && !log->stopping)
  {
    log->stop_time_ms = time_ms;
    log->stopping = true;
    DebugLogProcess(log);
  }
}

/**
 * @brief 记录一个控制采样点
 * @param log 日志对象指针
 * @param sample 采样数据指针
 */
void DebugLogSampleWrite(DebugLogObject *log, const DebugLogSample *sample)
{
  DebugLogProcess(log);
  if (log == NULL || sample == NULL || !log->initialized ||
      !log->config.enabled || !log->recording || log->stopping)
  {
    return;
  }
  bool selected = (log->sample_count++ % log->config.sample_period) == 0U;
  if (!selected || !log->config.binary_enabled)
  {
    return;
  }
  DebugLogSample copy = *sample;
  copy.sequence = log->sequence++;
  if (log->busy || log->event_count > 0U ||
      !UartTransmitReady(log->config.huart) ||
      !DebugLogFrame(log, kDebugLogTypeSample, (const uint8_t *)&copy,
                       sizeof(copy), copy.sequence))
  {
    log->dropped_frames++;
  }
}

/**
 * @brief 记录英文信息日志
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 * @param message 英文消息
 */
void DebugLogInfo(DebugLogObject *log, uint32_t time_ms, const char *message)
{
  DebugLogText(log, time_ms, "INFO", message);
}

/**
 * @brief 记录英文错误日志
 * @param log 日志对象指针
 * @param time_ms 当前时间（ms）
 * @param message 英文消息
 */
void DebugLogError(DebugLogObject *log, uint32_t time_ms, const char *message)
{
  DebugLogText(log, time_ms, "ERROR", message);
}
