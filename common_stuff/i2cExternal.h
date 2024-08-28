#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

int i2cMidiInit();
int sysexSendI2c(const uint8_t* payload, size_t len);
void i2cProcessIncomingFromMainThread();

#ifdef __cplusplus
} // extern "C"
#endif
