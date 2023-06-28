#include "usbd_midi_if.h"
#include "bootloader.h"
#include "stringId.h"
#include "storage.h"
#define sysexMatches(comp, args) (sizeof(comp) + args == len && 0 == memcmp(buf, comp, sizeof(comp)))

constexpr uint8_t kOurSysex[] = { 0x0, 0x21, 0x3e, 0x00};
static constexpr size_t kOffset = 1 + sizeof(kOurSysex);
static constexpr size_t kExtraBytes = kOffset + 1;
constexpr uint8_t kEox = 247;
constexpr uint8_t kSox = 240;
static uint8_t sysexIn[kExtraBytes + 256 + 10];
static uint8_t sysexIdx = 0;

static void sendSysex(uint8_t* payload, size_t len)
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
}

static uint16_t midiInputCallback(uint8_t *msg, uint16_t length)
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
		if(sysexIdx == sizeof(sysexIn))
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
	size_t end = sysexIdx;
	sysexIdx = 0; // for the next time
	if(sysexIn[0] != kSox)
	{
		printf("Invalid first byte\n\r");
		return 1;
	}
	if(containsEox && sysexIn[end - 1] != kEox)
	{
		printf("Coudln't find EOX but it was expected\n\r");
		return 1;
	}
	for(size_t n = 0; n < sizeof(kOurSysex) && n + 1 < end; ++n)
	{
		if(sysexIn[n + 1] != kOurSysex[n])
		{
			printf("Not our sysex\n\r");
			return 0;
		}
	}
	const size_t len = end - kExtraBytes;
	const uint8_t* buf = sysexIn + kOffset;

	printf("payload [%d]:", end - kExtraBytes);
	for(size_t n = 0; n < len; ++n)
	{
		printf("%d ", buf[n]);
	}
	printf("\n\r");
	constexpr uint8_t kJumpToBootloader[] = {63, 126};
	if(sysexMatches(kJumpToBootloader, 1))
	{
		uint8_t byte = buf[sizeof(kJumpToBootloader)];
		printf("JUMP TO BOOTLOADER %d\n\r", byte);
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
			uint8_t data[] = {127, 126, byte};
			sendSysex(data, sizeof(data));
			bootloaderResetTo(dest);
		}
	}
	constexpr uint8_t kIdentify[] = {0, 0};
	if(sysexMatches(kIdentify, 0))
	{
		printf("IDENTIFY\n\r");
		uint8_t data[2 + (strlen(stringId) + 1)];
		data[0] = 64;
		data[1] = bootloaderIs();
		memcpy(data + 1, stringId, strlen(stringId) + 1); //copy also closing NULL byte
		sendSysex(data, sizeof(data));
		// sendMidiMessage(msg, length);
	}
	constexpr uint8_t kFlashErase[] = {63, 125};
	if(sysexMatches(kFlashErase, 2))
	{
		buf += sizeof(kFlashErase);
		uint8_t target = buf[0] + (buf[1] << 7);
		printf("FLASH ERASE %#x\n\r", target);
		uint8_t err = 0;
		if(bootloaderIs())
		{
			if(target < 32) {
				printf("cannot flash ourselves");
				err = 1;
			}
		} else {
			if(target >= 32 && target < 128)
			{
				printf("cannot flash ourselves");
				err = 2;
			}
		}
		if(target < 192)
		{
			printf("Still testing, can't erase %u\n\r", target);
			err = 3;
		}
		if(!err)
		{
			if(storageErase(target))
				err = 4;
		}
		uint8_t data[] = {127, 125, err, target};
		sendSysex(data, sizeof(data));
	}
	constexpr uint8_t kFlashWrite[] = {63, 124};
	if(sysexMatches(kFlashWrite, 256 + 4))
	{
		const uint8_t* data = buf + sizeof(kFlashWrite);
		printf("FLASH WRITE %d %d %d %d \n\r", data[0], data[1], data[2], data[3]);
//		uint32_t target = buf[0] + (buf[1] << 7) + (buf[2] << 14) + (buf[3] << 21));
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
	return 0;
}

extern "C" void midiInit()
{
//	setHdlCtlChange(midiCtlCallback);
	setHdlAll(midiInputCallback);
}
