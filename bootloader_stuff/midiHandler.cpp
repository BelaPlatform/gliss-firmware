#ifdef USE_HAL_DRIVER
#define GLISS
#endif
#ifdef GLISS
#include "usbd_midi_if.h"
#include "bootloader.h"
#include "storage.h"
#include "midiHandler.h"
#include "stringId.h"
#else
static const char stringId[] = "test-stringId";
#endif // GLISS
#include <algorithm>
#include "sysex.h"

#ifdef GLISS
int verifyFlashRangeIsWritable(const char* targetStart, const char* targetStop)
{
	int err = 0;
	BootloaderResetDest_t weAre = bootloaderIs() ? kBootloaderMagicUserBootloader : kBootloaderMagicUserApplication;
	const size_t kSz = storageGetSectorSize();
	for(char const* target = targetStart; target < targetStop; target += kSz)
	{
		// verify we are trying to erase valid portions of memory
		if(bootloaderIsPartOf(target, weAre) || bootloaderIsPartOf(target + kSz, weAre))
		{
			printf("cannot erase ourselves");
			err = 3;
			break;
		}
		int sector = storageGetSectorFromAddress(target);
		if(sector < 0)
		{
			printf("storageGetSectorFromAddress returned %d\n\r", sector);
			err = -sector; // 1 or 2
			break;
		}
	}
	return err;
}


static uint8_t sysexIdx = 0;
static std::array<uint8_t,266 + kExtraBytes> sysexIn;

int sysexSend(const uint8_t* payload, size_t len)
{
	uint8_t msg[4];
	size_t i = 0;
	size_t total = len + kExtraBytes;
	for(size_t n = 0; n < total; ++n)
	{
		uint8_t byte;
		if(n < kOffset)
		{
			if(0 == n)
				byte = kSox;
			else
				byte = kOurSysex[n - 1];
		}
		else if(n < kOffset + len)
			byte = payload[n - kOffset];
		else
			byte = kEox;
		printf("%d\n\r", byte);
		msg[1 + i] = byte;
		bool done = (n + 1 == total);
		bool msgRd = (i == 2) || done;
		if(msgRd)
		{
			if(done)
			{
				msg[0] = 0x5 + i;
				for(size_t k = 2 + i; k < sizeof(msg); ++k)
					msg[k] = 0;
			} else
				msg[0] = 0x4;
			i = 0;
			printf("%#02x %d %d %d\n\r", msg[0], msg[1], msg[2], msg[3]);
			sendMidiMessage(msg, sizeof(msg));
		} else {
			++i;
		}
	}
	return 0;
}

uint16_t midiInputCallback(uint8_t *msg, uint16_t length)
{
	for(unsigned int n = 0; n < length; ++n)
		printf("%02x ", msg[n]);
	if(length)
		printf("\n\r");
	size_t count = 0;
	bool containsEox = 0;
	switch(msg[0])
	{
	case 0x4:
		count = 3;
		break;
	case 0x5:
		count = 1;
		containsEox = 1;
		break;
	case 0x6:
		count = 2;
		containsEox = 1;
		break;
	case 0x7:
		count = 3;
		containsEox = 1;
		break;
	}
	if(!count)
		return 1;
	for(size_t n = 0; n < count; ++n)
	{
		sysexIn[sysexIdx++] = msg[1 + n];
		if(sysexIdx == sysexIn.size())
		{
			if(n == count - 1)
				break;
			else {
				printf("too much sysex\n\r");
				sysexIdx = 0;
				return 1;
			}
		}
	}
	if(!containsEox)
		return 0;
	const size_t size = sysexIdx;
	const uint8_t* data = sysexIn.data();
	sysexIdx = 0; // ready for next message
	if(!sysexIsValid(data, size))
	{
		printf("message is not valid sysex\n\r");
		return 1;
	}
	if(!sysexForUs(data, size))
	{
		printf("Not our sysex\n\r");
		return 1;
	}
	deviceProcessSysex(data + kOffset, size - kExtraBytes);
	return 0;
}
void midiInit()
{
	setHdlAll(midiInputCallback);
}
#endif // GLISS

void sendAck(const uint8_t* buf, size_t len, uint8_t ack)
{
	std::array<uint8_t,kNumAckedBytes + 2> data {};
	size_t i = 0;
	for(size_t n = 0; n < kAck.size(); ++n)
		data[i++] = kAck[n];
	for(size_t n = 0; n < kNumAckedBytes && n < len; ++n)
		data[i++] = buf[n];
	assert(len >= kNumAckedBytes);
	data[i++] = ack;
	sysexSend(data.data(), data.size());
}
// we receive the payload of a valid sysex message that's addressed to us
void deviceProcessSysex(const uint8_t* buf, size_t len)
{
	printf("payload [%zu]:", len);
	for(size_t n = 0; n < len; ++n)
	{
		printf("%d ", buf[n]);
	}
	printf("\n\r");
	if(sysexMsgMatches(buf, len, kJumpToBootloader, 1))
	{
		uint8_t byte = buf[sizeof(kJumpToBootloader)];
		printf("JUMP TO BOOTLOADER %d\n\r", byte);
		int ret = 0;
#ifdef GLISS
		BootloaderResetDest_t dest;
		switch(byte)
		{
		default:
		case 0:
			dest = kBootloaderMagicNone;
			break;
		case 1:
			dest = kBootloaderMagicUserBootloader;
			break;
		case 2:
			dest = kBootloaderMagicUserApplication;
			break;
		case 3:
			dest = kBootloaderMagicSystemBootloader;
			break;
		}
		if(kBootloaderMagicNone == dest)
			printf("Invalid jumping destination\n\r");
		else {
			bootloaderResetTo(dest);
		}
#endif // GLISS
		sendAck(buf, len, ret);
	}
	if(sysexMsgMatches(buf, len, kIdentifyQuery, 0))
	{
		printf("IDENTIFY\n\r");
		sendAck(buf, len, 0);
		uint8_t data[sizeof(stringId) + kIdentifyReply.size()];
		size_t i = 0;
		for(size_t n = 0; n < kIdentifyReply.size(); ++n)
			data[i++] = kIdentifyReply[n];
#ifdef GLISS
		data[i++] = bootloaderIs();
#else // GLISS
		data[i++] = 1;
#endif // GLISS
		size_t strl = strlen(stringId) + 1; //copy also the terminating NULL byte
		memcpy(&data[i], stringId, strl);
		sysexSend(data, i + strl);
	}
	if(sysexMsgMatches(buf, len, kFlashErase, 9))
	{
		const uint8_t* c = buf + kFlashErase.size();
		bool force = *c;
		c++;
		const char* const targetStart = (char*)(long unsigned)midiToUint28(c);
		c += 4;
		const char* const targetStop = (char*)(long unsigned)midiToUint28(c);
		c += 4;
		printf("FLASH %s ERASE %p to %p\n\r", force ? "force" : "", targetStart, targetStop);
		uint8_t err = 0;
#ifdef GLISS
		int writable = verifyFlashRangeIsWritable(targetStart, targetStop);
		if(!writable && !force)
			err = writable;
		if(!err)
		{
			// do erase flash
			for(const char* target = targetStart; target < targetStop; target += storageGetSectorSize())
			{
				unsigned int sector = storageGetSectorFromAddress(target); //already validated above
				err = storageErase(sector);
				if(err)
				{
					printf("storageErase(%u) returned %d\n\r", sector, err);
					break;
				}
			}
		}
#endif // GLISS
		sendAck(buf, len, err);
	}
	static uint16_t pastSeq = 0;
	if(sysexMsgMatches(buf, len, kFlashWrite, 9))
	{
		const uint8_t* c = buf + kFlashWrite.size();
		bool force = *c;
		c++;
		const char* const targetStart = (char*)(long unsigned)midiToUint28(c);
		c += 4;
		const char* const targetStop = (char*)(long unsigned)midiToUint28(c);
		c += 4;
		printf("FLASH %s WRITE %p to %p\n\r", force ? "force" : "", targetStart, targetStop);
		uint8_t err = 0;
#ifdef GLISS
		int writable = verifyFlashRangeIsWritable(targetStart, targetStop);
		if(!writable && !force)
			err = writable;
#endif // GLISS
		if(!err)
			pastSeq = 0;
		sendAck(buf, len, err);
	}
	if(sysexMsgMatches(buf, len, kFlashWritePayload, 2 + kMidiBytesPerPayloadUnit))
	{
		const uint8_t* c = buf + kFlashWrite.size();
		size_t seq = midiToUint14(c);
		c += 2;
		int err = 0;
		if(seq != pastSeq + 1)
		{
			err = 1;
		}
		pastSeq = seq;
#if 0
		if(!err)
		{
			uint8_t flashData[192];
			const uint8_t* src = data + 4;
			uint8_t* dst = flashData;
			size_t sLen = len - 4; // 4 bytes used for the target above
			size_t dLen = std::min(sizeof(flashData), (3 * sLen) / 4);
			printf("%zu incoming bytes -> %zu flash bytes\n\r", sLen, dLen);
			sLen = std::min(len, 4 * sizeof(flashData) / 3);
			size_t d = 0;
			size_t s = 0;
			for(; d < dLen - 2 && s < sLen - 3; )
			{
				dst[d] = (src[s] & 0x3f); // s0, 6 bits
				s++;
				dst[d] |= (src[s] & 0x03) << 6; // s1, 2 lower bits
				d++;

				dst[d] = (src[s] & 0x3c) >> 2; // s1, 4 top bits
				s++;
				dst[d] |= (src[s] & 0x0f) << 4; // s2, 4 lower bits
				d++;

				dst[d] = (src[s] & 0x30) >> 4; // s2, 2 upper bits
				s++;
				dst[d] |= (src[s] & 0x3f) << 2; // s3, 6 bits
				s++;
				d++;
			}
			printf("Received %zu bytes, turned into %zu bytes\n\r", s, d);
			if(target < 0x08050000)
			{
				printf("not writing 0x%lx: we are still testing\n\r", (unsigned long)target);
				err = 3;
			}
#ifdef GLISS
			if(storageWriteStatic(target, flashData, d))
				err = 4;
#endif // GLISS
		}
#endif
		sendAck(buf, len, err);
	}
	if(sysexMsgMatches(buf, len, kMemoryRead, 6))
	{
		const uint8_t* data = buf + sizeof(kMemoryRead);
		uint32_t target = midiToUint28(data) + 0x08000000;
		size_t len = midiToUint14(data + 2);
		printf("TODO: send flash read of %zu bytes at 0x%010lx\n\r", len, (unsigned long)target);
	}
//	// program change channel 1, program 2, (i.e.: 0x0 and 0x1 respectively)
//#include "bootloader.h"
//	if(0x0c == msg[0] && 0xc0 == msg[1] && 0x01 == msg[2])
//	{
//		np.clear();
//		np.setPixelColor(kNumLeds - 1, 0, 255, 0);
//		// show() may fail if another buffer is being sent right now.
//		// TODO: wait for it but ensure the timer thread has a higher preemption priority than this one
//		np.show();
//		printf("Jumping to bootloader\n\r");
//		bootloaderResetTo();
//	}
}

