#include "verificationBlock.h"
#ifdef CFG_BOOTLOADER
#define ATTR
#else
#define ATTR __attribute__((section(".stringIdSec"), used))
#endif

#if __has_include("gitHashes.h")
#include "gitHashes.h"
#else
#define GIT_HASHES ""
#endif // has-include

struct VerificationBlock kVerificationBlock ATTR = {
#ifdef CFG_FLASHER
	.stringId = "Gliss-flasher-v1.2",
#else
	.stringId = "Gliss-v1.2",
#endif // CFG_FLASHER
	.gitHashes = GIT_HASHES,
	.reservedBytes = "",
	.tag = { 0xbe, 0x7a, 0x67, 0x15}, //BELAGLIS
};
_Static_assert(sizeof(struct VerificationBlock) == 2048 - 512); // one full flash sector minus the 512 byte offset
