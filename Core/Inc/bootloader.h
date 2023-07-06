#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx_hal.h"

typedef enum {
	// first entry actually unused here
	kBootloaderMagicNone = 0,
	kBootloaderMagicUserBootloader = 0xBE7A8007, // BOOT
	kBootloaderMagicUserApplication = 0xBE7AE4EC, // EXEC
	kBootloaderMagicSystemBootloader = 0xBE7A5457, // SYST
} BootloaderResetDest_t;

__attribute__((noreturn)) void bootloaderResetTo(BootloaderResetDest_t dest);
void bootloaderJumpIfNeeded(RTC_HandleTypeDef* hrtc);
uint32_t bootloaderShouldJump(RTC_HandleTypeDef* hrtc);
void bootloaderSetVector(void);
void bootloaderJump(RTC_HandleTypeDef* hrtc, BootloaderResetDest_t to);
int bootloaderIs();
int bootloaderIsPartOf(BootloaderResetDest_t dest);

#ifdef __cplusplus
}
#endif
