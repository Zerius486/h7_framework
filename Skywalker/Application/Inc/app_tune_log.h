#ifndef APP_TUNE_LOG_H
#define APP_TUNE_LOG_H

#include <stdint.h>

#include "alg_autotune.h"
#include "alg_motor_tune.h"
#include "bsp_debug_log.h"

void AppTuneLogInit(const DebugLogConfig *config);
void AppTuneLogProcess(void);
void AppTuneLogAutotune(const AutotuneObject *tune, uint32_t time_ms,
                        float position, float omega, float actual_current,
                        float requested_current, float executed_current);
void AppTuneLogMotorTune(const MotorTuneObject *tune, uint32_t time_ms,
                         float position, float omega, float actual_current);

#endif // APP_TUNE_LOG_H
