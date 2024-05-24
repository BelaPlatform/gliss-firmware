#include <stdint.h>
#include <array>

constexpr uint8_t kEox = 247;
constexpr uint8_t kSox = 240;
constexpr std::array<uint8_t,4> kOurSysex = { 0x0, 0x21, 0x3e, 0x00}; // last is fw rev
static constexpr size_t kOffset = 1 + kOurSysex.size();;
static constexpr size_t kTrailingBytes = 1;
static constexpr size_t kExtraBytes = kOffset + kTrailingBytes;
enum {
	kPartitionIdNone = 0,
	kPartitionIdFlasher = 1,
	kPartitionIdApplication = 2,
	kPartitionIdSystemBootloader = 3,
};

constexpr uint8_t kByteSystem = 5;
constexpr uint8_t kByteProtocol = 6;
constexpr uint8_t kByteAck = 100;
constexpr uint8_t kByteBootloader = 127;
constexpr uint8_t kNumAckedBytes = 2;
constexpr std::array<uint8_t,2> kIdentifyQuery = {kByteSystem, 6};
constexpr std::array<uint8_t,2> kIdentifyReply = {kByteSystem, 7};
constexpr std::array<uint8_t,1> kAck = {kByteAck};
constexpr std::array<uint8_t,2> kJumpToBootloader = {kByteBootloader, 127};
constexpr std::array<uint8_t,2> kFlashErase = {kByteBootloader, 126};
constexpr std::array<uint8_t,2> kFlashWrite = {kByteBootloader, 125};
constexpr std::array<uint8_t,2> kFlashWritePayload = {kByteBootloader, 124};
constexpr std::array<uint8_t,2> kMemoryReadQuery = {kByteBootloader, 123};
constexpr std::array<uint8_t,2> kMemoryReadReply = {kByteBootloader, 122};
constexpr std::array<uint8_t,2> kTestQuery = {kByteBootloader, 121};
constexpr std::array<uint8_t,2> kTestReply = {kByteBootloader, 120};
constexpr std::array<uint8_t,1> kProtocol = {kByteProtocol};
constexpr size_t kFullBytesPerPayloadUnit = 56;
constexpr size_t kMidiBytesPerPayloadUnit = kFullBytesPerPayloadUnit / 7 * 8;

static inline bool sysexIsValid(const uint8_t* sysex, size_t sysexSize)
{
	if(sysexSize < 2)
		return false;
	if(kSox != sysex[0])
		return false;
	if(kEox != sysex[sysexSize - 1])
		return false;
	return true;
}
static inline bool sysexForUs(const uint8_t* sysex, size_t sysexSize)
{
	if(sysexSize < kExtraBytes)
		return false;
	for(size_t n = 0; n < kOurSysex.size(); ++n)
		if(sysex[1 + n] != kOurSysex[n])
			return false;
	return true;
}

template <typename T>
bool sysexMsgMatches(const uint8_t* buf, size_t len, const T& comp, size_t bytes, bool exact = true)
{
	if(comp.size() <= len)
	{
		if(memcmp(buf, comp.data(), comp.size()))
			return false;
		size_t actualBytes = len - comp.size();
		if(exact)
			return bytes == actualBytes;
		else
			return bytes <= actualBytes;
	}
	return false;
}
template <typename T,typename U>
bool sysexMsgMatches(const U& buf, const T& comp, size_t bytes, bool exact = true)
{
	return sysexMsgMatches(buf.data(), buf.size(), comp, bytes, exact);
}

static inline std::array<uint8_t,2> uint14ToMidi(uint16_t val)
{
	return { uint8_t(val & 0x7f), uint8_t((val >> 7) & 0x7f) };
}

static inline std::array<uint8_t,4> uint28ToMidi(uint32_t val)
{
	auto bottom = uint14ToMidi(val);
	auto top = uint14ToMidi(val >> 14);
	std::array<uint8_t,4> ret;
	for(size_t n = 0; n < bottom.size(); ++n)
	{
		ret[n] = bottom[n];
		ret[n + bottom.size()] = top[n];
	}
	return ret;
}

static inline ssize_t shuffleBits(uint8_t* out, const size_t outSize, const uint8_t outMaxBits, const uint8_t* in, const size_t inSize, const uint8_t inMaxBits)
{
	const uint8_t* const outBak = out;
	const uint8_t* const inEnd = in + inSize;
	const uint8_t* const outEnd = out + outSize;
	const uint8_t outMask = (1 << outMaxBits) - 1;
	uint8_t inBits = inMaxBits;
	uint8_t outBits = 0;
	uint8_t inByte;
	uint8_t outByte = 0;
	ssize_t ret = 0;
	while(1)
	{
		if(outMaxBits == outBits)
		{
			// printf(">>> outByte: %02x,\n\r", outByte);
			*out++ = outByte;
			outBits = 0;
			outByte = 0;
			if(out >= outEnd)
				break;
		}
		if(inMaxBits == inBits)
		{
			if(in >= inEnd)
			{
				// if we run out of input bytes, pad with zeros
				inByte = 0;
			} else {
				inByte = *in++;
			}
			inBits = 0;
		}
		uint8_t copyBits = std::min(inMaxBits - inBits, outMaxBits - outBits);
		uint8_t inMask = ((1 << copyBits) - 1) << inBits;
		//printf("copyBits: %d, inBits: %d, inMask: %02x, inByte: %02x === ", copyBits, inBits, inMask, inByte);
		//printf("outBits: %d, outByte: %02x\n\r", outBits, outByte);
		outByte |= outMask & (((inByte & inMask) >> inBits) << outBits);
		outBits += copyBits;
		inBits += copyBits;
	}
	if(ret)
		return ret;
	return out - outBak;
}

static inline std::array<uint8_t,kFullBytesPerPayloadUnit> midiToPayload(const uint8_t* in)
{
	std::array<uint8_t,kFullBytesPerPayloadUnit> out;
	ssize_t ret = shuffleBits(out.data(), out.size(), 8, in, kMidiBytesPerPayloadUnit, 7);
	if(ret < 0)
	{
		fprintf(stderr, "ERROR: ran out of input bytes\n\r");
	}
	if(ret != out.size())
	{
		fprintf(stderr, "ERROR: expected %u bytes, got %u bytes\n\r", unsigned(ret), unsigned(out.size()));
	}
	return out;
}

static inline std::array<uint8_t,kMidiBytesPerPayloadUnit> payloadToMidi(const uint8_t* in, size_t maxFullBytes)
{
	std::array<uint8_t,kMidiBytesPerPayloadUnit> out;
	ssize_t ret = shuffleBits(out.data(), out.size(), 7, in, std::min(maxFullBytes, kFullBytesPerPayloadUnit), 8);
	if(ret < 0)
	{
		fprintf(stderr, "ERROR: ran out of input bytes\n\r");
	}
	if(ret != out.size())
	{
		fprintf(stderr, "ERROR: expected %u bytes, got %u bytes\n\r", unsigned(ret), unsigned(out.size()));
	}
	return out;
}

static inline ssize_t payloadToMidiBytes(size_t payloadBytes)
{
	if(payloadBytes % kFullBytesPerPayloadUnit)
		return -1;
	return payloadBytes / kFullBytesPerPayloadUnit * kMidiBytesPerPayloadUnit;
}

static inline ssize_t midiToPayloadBytes(size_t midiBytes)
{
	if(midiBytes % kMidiBytesPerPayloadUnit)
		return -1;
	return midiBytes / kMidiBytesPerPayloadUnit * kFullBytesPerPayloadUnit;
}

static inline uint16_t midiToUint14(const uint8_t* data)
{
	return (data[0] & 0x7f) | ((data[1] & 0x7f) << 7);
}

static inline uint32_t midiToUint28(const uint8_t* data)
{
	return midiToUint14(data) | (midiToUint14(data + 2) << 14);
}

void deviceProcessSysex(const uint8_t* sysexIn, size_t sysexSize);
int sysexSend(const uint8_t* payload, size_t len);
template <typename T>
static int sysexSend(const T& content) {
	return sysexSend(content.data(), content.size());
}
