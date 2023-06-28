#include "stringId.h"

#ifdef BOOTLOADER_ONLY
char other_section[] __attribute__((section(".applicationSec"), used)) =
#include "hexdumped_application.h"
#else // BOOTLOADER_ONLY
char other_section[] __attribute__((section(".bootloaderSec"), used)) =
#include "hexdumped_bootloader.h"
#endif // BOOTLOADER_ONLY

