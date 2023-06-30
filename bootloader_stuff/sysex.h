#include <stdint.h>
#include <array>

constexpr uint8_t kEox = 247;
constexpr uint8_t kSox = 240;
constexpr std::array<uint8_t,4> kOurSysex = { 0x0, 0x21, 0x3e, 0x00}; // last is fw rev
static constexpr size_t kOffset = 1 + kOurSysex.size();;
static constexpr size_t kTrailingBytes = 1;
static constexpr size_t kExtraBytes = kOffset + kTrailingBytes;

constexpr uint8_t kByteSystem = 5;
constexpr uint8_t kByteAck = 100;
constexpr uint8_t kByteBootloader = 127;
constexpr uint8_t kNumAckedBytes = 2;
constexpr std::array<uint8_t,2> kIdentifyQuery = {kByteSystem, 6};
constexpr std::array<uint8_t,2> kIdentifyReply = {kByteSystem, 7};
constexpr std::array<uint8_t,1> kAck = {kByteAck};
constexpr std::array<uint8_t,2> kJumpToBootloader = {kByteBootloader, 127};
constexpr std::array<uint8_t,2> kFlashErase = {kByteBootloader, 126};
constexpr std::array<uint8_t,2> kFlashWrite = {kByteBootloader, 125};
constexpr std::array<uint8_t,2> kFlashRead = {kByteBootloader, 124};

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
	/*c
	printf("RET: ");
	for(auto b : ret)
		printf("%d ", b);
	printf("\n");
	*/
	return ret;
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
