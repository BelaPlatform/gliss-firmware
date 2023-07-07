#include "stringId.h"
#ifdef BOOTLOADER_ONLY
char stringId[256] = "GLISS-BOOTLOADER";
#else
char stringId[256] __attribute__((section(".stringIdSec"), used)) = "Gliss";
#endif // BOOTLOADER_ONLY
