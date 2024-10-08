#ifdef USE_HAL_DRIVER
#define GLISS
#endif

#include <stdint.h>

#ifdef GLISS
#include "usbd_midi_if.h"
#include "bootloader.h"
#include "storage.h"
#include "midiHandler.h"
#include "verificationBlock.h"
#include "../TrillRackApplication/trill-neopixel/GlissProtocol.h"
#define P(ptr) ((void*)(ptr))
#else
#include <vector>
const struct VerificationBlock {
	char stringId[128] = "test-stringId";
} kVerificationBlock;
static std::vector<uint8_t> storage(512 * 1024);
#include <unistd.h>
#define P(ptr) ((void*)(unsigned long)(ptr))
#endif // GLISS
#include <algorithm>
#include "i2cExternal.h"
#include "sysex.h"

static bool verbose = false;
#ifdef GLISS
int verifyFlashRangeIsWritable(uint32_t targetStart, uint32_t targetStop, bool careAboutBoundaries)
{
	int err = 0;
	BootloaderResetDest_t weAre = bootloaderIsFlasher() ? kBootloaderMagicUserFlasher : kBootloaderMagicUserApplication;
	const size_t kSz = storageGetSectorSize();
	int sector = 0;
	for(uint32_t target = targetStart; target < targetStop; target += kSz)
	{
		// verify we are trying to erase valid portions of memory
		if(bootloaderIsPartOf(target, weAre) || bootloaderIsPartOf(target + kSz, weAre))
		{
			printf("cannot erase ourselves\n\r");
			err = 3;
			break;
		}
		sector = storageGetSectorFromAddress(target, careAboutBoundaries);
		if(sector < 0)
			break;
	}
	if(sector > 0)
		sector = storageGetSectorFromAddress(targetStop, careAboutBoundaries);
	if(sector < 0)
	{
		printf("storageGetSectorFromAddress returned %d\n\r", sector);
		err = -sector; // 1 or 2
	}
	return err;
}

static int sysexSendUsb(const uint8_t* payload, size_t len)
{
	uint8_t msg[4];
	size_t i = 0;
	size_t total = len + kExtraBytes;
	if(verbose) {
		printf("sending %u { ", len);
		for(size_t n = 0; n < len; ++n)
			printf("%u ", payload[n]);
		printf("}\n\r");
	}
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
		// printf("%d ", byte);
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
//			printf("%#02x %d %d %d\n\r", msg[0], msg[1], msg[2], msg[3]);
			sendMidiMessage(msg, sizeof(msg));
		} else {
			++i;
		}
	}
	return 0;
}

int sysexSend(SysexPeripheral sp, const uint8_t* payload, size_t len)
{
	if(kSysexUsb == sp)
		return sysexSendUsb(payload, len);
	if(kSysexI2c == sp)
		return sysexSendI2c(payload, len);
	return 0;
}

enum Color {
	kDark,
	kGreen,
	kRed,
};

#include <main.h>
static void setColor(Color color){
#ifdef CFG_FLASHER
	GPIO_PinState swa;
	GPIO_PinState swb;
	switch(color)
	{
		default:
		case kDark:
			swa = swb = GPIO_PIN_RESET;
			break;
		case kGreen:
			swa = GPIO_PIN_SET;
			swb = GPIO_PIN_RESET;
			break;
		case kRed:
			swa = GPIO_PIN_RESET;
			swb = GPIO_PIN_SET;
			break;
	}
	// red
	HAL_GPIO_WritePin(GPIOB, SW_LED_A_Pin, swa);
	HAL_GPIO_WritePin(GPIOB, SW_LED_B_Pin, swb);
#endif // CFG_FLASHER
}
class Colorer {
public:
	Colorer(Color color ) { setColor(color); }
	~Colorer() { setColor(kRed); };
};

uint16_t usbMidiInputCallback(uint8_t *msg, uint16_t length)
{
	static uint8_t sysexIdx = 0;
	static std::array<uint8_t,266 + kExtraBytes> sysexIn;
//	for(unsigned int n = 0; n < length; ++n)
//		printf("%02x ", msg[n]);
//	if(length)
//		printf("\n\r");
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
	midiHandleIncoming(kSysexUsb, data + kOffset, size - kExtraBytes);
	return 0;
}
void midiInit()
{
	setHdlAll(usbMidiInputCallback);
}
#endif // GLISS

class MidiHandler {
private:
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
SysexPeripheral sp;

public:
MidiHandler(SysexPeripheral sp) : sp(sp) {}

void sysexSend(const uint8_t* data, size_t size)
{
	::sysexSend(sp, data, size);
}
template <typename T>
static int sysexSend(const T& content) {
	return sysexSend(content.data(), content.size());
}
// we receive the payload of a valid sysex message that's addressed to us
void handleSysex(const uint8_t* buf, size_t len)
{
#ifdef GLISS
	Colorer colorer(kGreen);
#endif // GLISS
	if(verbose)
	{
		printf("payload [%u]:", (unsigned)len);
		for(size_t n = 0; n < len; ++n)
		{
			printf("%d ", buf[n]);
		}
		printf("\n\r");
	}
	if(sysexMsgMatches(buf, len, kTestQuery, 1, false))
	{
		size_t sz = kTestReply.size();
		size_t payloadSz = len - kTestQuery.size();
		uint8_t reply[sz + payloadSz];
		memcpy(reply, kTestReply.data(), sz);
		memcpy(reply + sz, buf + kTestQuery.size(), payloadSz);
		sysexSend(reply, sizeof(reply));
	}
	if(sysexMsgMatches(buf, len, kJumpToBootloader, 1))
	{
		uint8_t byte = buf[sizeof(kJumpToBootloader)];
		printf("JUMP TO BOOTLOADER %d with APP_Rx_ptr_out: %lu\n\r", byte, APP_Rx_ptr_out);
		int ret = 0;
#ifdef GLISS
		BootloaderResetDest_t dest;
		switch(byte)
		{
		default:
		case kPartitionIdNone:
			dest = kBootloaderMagicNone;
			ret = 1;
			break;
		case kPartitionIdFlasher:
			dest = kBootloaderMagicUserFlasher;
			break;
		case kPartitionIdApplication:
			dest = kBootloaderMagicUserApplication;
			break;
		case kPartitionIdSystemBootloader:
			dest = kBootloaderMagicSystemBootloader;
			break;
		}
		if(ret)
			printf("Invalid jumping destination\n\r");
		printf("Jumping to dest %#x, ret: %d\n\r", dest, ret);
#endif // GLISS
		sendAck(buf, len, ret);
#ifdef GLISS
		// ensure the host has time to read the ack before we reboot
		void USBD_MIDI_SendPacket();
		// Two issues at play:
		// 1. More than one calls to USB_MIDI_SendPacket()
		// may be required for the whole content of the circular buffer to be
		// made available to the host. This is because only sections of contiguous
		// memory from the circular buffer can be passed to the low-level HAL USB functions
		// So, if the circular buffer is currnetly wrapped around, two subsequent calls
		// are needed. Possibly (I didn't investigate) a wait is necessary between the two
		// 2. We need to make sure that the host has enough time to read the data before
		// the device reboots. We have witnessed Windows machines where a 50 ms wait was
		// not enough and the host was missing the response.
		//
		// So, we wait about 500ms, calling USB_MIDI_SendPacket() several times for good measure.
		for(size_t n = 0; n < 50; ++n)
		{
			USBD_MIDI_SendPacket();
			HAL_Delay(10);
		}
		if(!ret)
			bootloaderResetTo(dest);
#endif // GLISS
	}
	if(sysexMsgMatches(buf, len, kIdentifyQuery, 0))
	{
		printf("IDENTIFY\n\r");
		const char* stringId = kVerificationBlock.stringId;
		sendAck(buf, len, 0);
		size_t strl = strlen(stringId) + 1; //copy also the terminating NULL byte
		uint8_t data[strl + 1 + kIdentifyReply.size()];
		size_t i = 0;
		for(size_t n = 0; n < kIdentifyReply.size(); ++n)
			data[i++] = kIdentifyReply[n];
#ifdef GLISS
		data[i++] = bootloaderIsFlasher() ? kPartitionIdFlasher : kPartitionIdApplication;
#else // GLISS
		data[i++] = 1;
#endif // GLISS
		for(size_t n = 0; n < strl; ++n)
			data[i++] = stringId[n] & 0x7f;
		sysexSend(data, sizeof(data));
	}
	if(sysexMsgMatches(buf, len, kFlashErase, 9))
	{
		const uint8_t* c = buf + kFlashErase.size();
		bool force = *c;
		c++;
		uint32_t targetStart = midiToUint28(c);
		c += 4;
		uint32_t targetStop = midiToUint28(c);
		c += 4;
		printf("FLASH %s ERASE %p to %p\n\r", force ? "force" : "", P(targetStart), P(targetStop));
		uint8_t err = 0;
#ifdef GLISS
		int writable = verifyFlashRangeIsWritable(targetStart, targetStop, true);
		if(writable && !force)
			err = writable;
		if(!err)
		{
			// do erase flash
			for(uint32_t target = targetStart; target < targetStop; target += storageGetSectorSize())
			{
				unsigned int sector = storageGetSectorFromAddress(target, true); //already validated above
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
	if(sysexMsgMatches(buf, len, kFlashWrite, 9))
	{
		const uint8_t* c = buf + kFlashWrite.size();
		bool force = *c;
		c++;
		writeTargetStart = midiToUint28(c);
		c += 4;
		writeTargetStop = midiToUint28(c);
		c += 4;
		printf("FLASH %s WRITE %p to %p\n\r", force ? "force" : "", P(writeTargetStart), P(writeTargetStop));
		uint8_t err = 0;
#ifdef GLISS
		int writable = verifyFlashRangeIsWritable(writeTargetStart, writeTargetStop, false);
		if(!writable && !force)
			err = writable;
#endif // GLISS
		if(!err)
		{
			writeIdx = 0;
			writePastSeq = -1;
			writeIsGood = true;
		} else {
			writeIsGood = false;
		}
		sendAck(buf, len, err);
	}
	if(sysexMsgMatches(buf, len, kFlashWritePayload, 2 + kMidiBytesPerPayloadUnit))
	{
		if(!writeIsGood)
			printf("Uninitialised or invalidated write session\n\r");
		const uint8_t* c = buf + kFlashWritePayload.size();
		uint8_t seq = *c;
		c++;
		int err = !writeIsGood;
		if(seq != ((writePastSeq + 1) & 0x7f))
		{
			err = 2;
			writeIsGood = false;
		} else
			writePastSeq = seq;
		uint8_t fullBytes = *c;
		c++;
		size_t midiBytes = len - kFlashWritePayload.size() - 2;
		if(fullBytes > midiToPayloadBytes(midiBytes))
			err = 3;
		if(fullBytes % sizeof(uint64_t))
		{
			printf("payload size %d is not compatible with flash word size\n\r", fullBytes);
			err = 4;
		}
		if(writeIdx + fullBytes + writeTargetStart > writeTargetStop)
		{
			printf("Trying to write to write from %p to %p which is beyond %p\n\r",
				P(writeTargetStart + writeIdx), P(writeTargetStart + writeIdx + fullBytes), P(writeTargetStop));
			err = 5;
			writeIsGood = false;
		}
		if(!err)
		{
			auto payload = midiToPayload(c);
#ifdef GLISS
			static_assert(payload.size() % sizeof(uint64_t) == 0, "payload is not compatible with flash word size");
			if(storageWriteStatic(writeTargetStart + writeIdx, payload.data(), fullBytes))
				err = 4;
#else
			std::copy(payload.begin(), payload.begin() + fullBytes, storage.begin() + (writeTargetStart - 0x08000000) + writeIdx);
#endif
			writeIdx += fullBytes;
		}
		sendAck(buf, len, err);
	}
	if(sysexMsgMatches(buf, len, kMemoryReadQuery, 7))
	{
		const uint8_t* c = buf + kMemoryReadQuery.size();
		bool force = *c;
		c++;
		const uint8_t* memoryStart = (uint8_t*)(long unsigned)midiToUint28(c);
		c += 4;
		uint16_t memSz = midiToUint14(c);
		c += 2;
		if(verbose)
			printf("Memory read: %u bytes starting at %p\n\r", memSz, memoryStart);
		int err = 0;
#ifdef GLISS
		// TODO: find valid flash and RAM ranges in the address space
#else
		// need to access some fake memory or we are in trouble
		memoryStart = storage.data() + (memoryStart - (uint8_t*)0x08000000);
		if(memSz + memoryStart > storage.data() + storage.size())
			err = 1;
#endif
		ssize_t midiBytes = payloadToMidiBytes(memSz);
		if(midiBytes < 0) // TODO: allow to read smaller number of bytes
			err = 2;
		if(force)
			err = 0;
		sendAck(buf, len, err);
#ifndef GLISS
		usleep(1000); // it would occasionally fail without this on macos with virtual midi ports ...
#endif
		if(!err)
		{
			size_t hdSz = kMemoryReadReply.size();
			uint8_t data[hdSz + midiBytes];
			for(size_t n = 0; n < hdSz; ++n)
				data[n] = kMemoryReadReply[n];
			for(size_t w = 0, r = 0; w < size_t(midiBytes) && r < memSz;
				w += kMidiBytesPerPayloadUnit,
				r += kFullBytesPerPayloadUnit)
			{
				auto midi = payloadToMidi(memoryStart + r, kFullBytesPerPayloadUnit);
				memcpy(data + hdSz + w, midi.data(), midi.size());
			}
			sysexSend(data, sizeof(data));
		}
	}
#ifdef GLISS
#ifndef CFG_FLASHER
	if(sysexMsgMatches(buf, len, kProtocol, 1, false))
	{
		size_t consumed = kProtocol.size();
		gp_incoming(sp, buf + consumed,len - consumed);
	}
#endif // CFG_FLASHER
#endif // GLISS
}
private:
uint8_t writePastSeq = 0;
size_t writeIdx = 0;
uint32_t writeTargetStart = 0;
uint32_t writeTargetStop = 0;
bool writeIsGood = false;
}; // class MidiHandler

void midiHandleIncoming(SysexPeripheral sp, const uint8_t* buf, size_t len)
{
	static std::array<MidiHandler,kSysexNumSp> midiHandlers = {{kSysexUsb, kSysexI2c}};
	midiHandlers[sp].handleSysex(buf, len);
}
