#pragma once
#include <stdint.h>
struct VerificationBlock {
	// start is relative to beginning of flash sector
	// start: 0, size: 512 reserved for isr_vector
	// start: 512, size: 128 stringId
	char stringId[128];
	// start: 640, size: 256 git hashes
	char gitHashes[256];
	char reservedBytes[1148];
	uint8_t tag[4];
};
extern struct VerificationBlock kVerificationBlock;
