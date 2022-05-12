#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx_hal.h"

__attribute__((noreturn)) void bootloaderResetTo();
void bootloaderJumpIfNeeded(RTC_HandleTypeDef* hrtc);
#ifdef __cplusplus
}
#endif
