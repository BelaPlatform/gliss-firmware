#include "storage.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint32_t baseAddress;
	uint8_t data[kStorageSlotSize];
	uint8_t synced;
} Storage;

static Storage storage = {0};

static void* getSectorStart(uint32_t sector)
{
	return (void*)(kFlashBase + sector * kStorageSectorSize);
}

static size_t countNonErasedBytes(uint8_t* p, size_t size)
{
	size_t count = 0;
	for(size_t n = 0; n < size; ++n)
	{
		if(0xff != p[n])
			++count;
	}
	return count;
}

void storageInit(uint32_t sector, uint32_t slot)
{
	storage.baseAddress = (uint32_t)getSectorStart(sector) + slot * kStorageSlotSize;
	storage.synced = 0;
}

void storageRead()
{
	memcpy(&storage.data, ((const char*)storage.baseAddress), kStorageSlotSize);
	storage.synced = 1;
}

static void printError(const char* id, uint32_t error, uint32_t loc)
{
	printf("%s storage error %ld at location %lx\r\n", id, error, loc);
}

int storageErase(uint32_t sector)
{
	// On the STM32G4, writing 0xff is not the same as the word being erased
	// This means that we cannot infer a page is empty by simply looking at it
	// Therefore, we need to unconditionally erase pages
#if 0
	if(0 == countNonErasedBytes(getSectorStart(sector), kStorageSectorSize))
	{
#ifndef CFG_FLASHER
		printf("No need to erase sector %lu: already erased\n\r", sector);
#endif // CFG_FLASHER
		return 0;
	}
#endif
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t errorLoc;
	eraseInit.Banks = FLASH_BANK_1;
#ifdef FLASH_HAS_SECTORS
	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Sector = sector;
	eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#else
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Page = sector;
	eraseInit.NbPages = 1;
#endif // FLASH_HAS_SECTORS
	HAL_StatusTypeDef ret = HAL_FLASH_Unlock();
	if(ret)
		printf("HAL_FLASH_Unlock: %d\n\r", ret);
	ret = HAL_FLASHEx_Erase(&eraseInit, &errorLoc);
	HAL_FLASH_Lock();
	if(HAL_OK != ret) {
		printError("erase", HAL_FLASH_GetError(), errorLoc);
		return -1;
	}
	size_t count = countNonErasedBytes(getSectorStart(sector), kStorageSectorSize);
	if(count) {
		printf("Error erasing flash sector %lu: %zu bytes still non-zero\n\r", sector, count);
		return 1;
	}
#ifndef CFG_FLASHER
	printf("Successfully erased flash sector %lu\n\r", sector);
#endif // CFG_FLASHER
	return 0;
}

int storageGetSectorFromAddress(uint32_t addr, uint8_t checkForBoundaries)
{
	uint32_t ad = (uint32_t)addr;
	if(addr < kFlashBase)
		return -1; // outside flash
	uint32_t offset = (ad - kFlashBase);
	if(checkForBoundaries)
	{
		if(offset % storageGetSectorSize())
				return -2; // unaligned
	}
	int sector = offset / storageGetSectorSize();
	if(sector > 256) // this is device-specific
		return -1; // outside flash
	return sector;
}

size_t storageGetSectorSize()
{
	return kStorageSectorSize;
}

int storageWriteStatic(uint32_t address, uint8_t* data, size_t len)
{
#ifndef CFG_FLASHER
	uint32_t startAddress = address;
#endif // CFG_FLASHER
	const uint8_t* ptr = (uint8_t*)address;
	size_t count = countNonErasedBytes((void*)address, len);
	if(count)
	{
		printf("Cannot write to flash because the destination has %u of %u non-erased bytes starting at %p\n\r", count, len, ptr);
		return -3;
	}
	int ret = HAL_FLASH_Unlock();
	if(ret)
		printf("HAL_FLASH_Unlock: %d\n\r", ret);
#ifdef FLASH_HAS_SECTORS
	const size_t kFlashWordSize = FLASH_NB_32BITWORD_IN_FLASHWORD * sizeof(uint32_t);
	const int kFlashProgramType = FLASH_TYPEPROGRAM_FLASHWORD;
#else
	const size_t kFlashWordSize = sizeof(uint64_t);
	const int kFlashProgramType = FLASH_TYPEPROGRAM_DOUBLEWORD;
#endif
	for(unsigned int idx = 0; idx < len; idx += kFlashWordSize)
	{
		// writes one flash word at a time
		HAL_StatusTypeDef ret = HAL_FLASH_Program(kFlashProgramType, address,
#ifdef FLASH_HAS_SECTORS
#ifndef STM32H7xx_HAL_FLASH_H
#error HAL_FLASH_Program(): check meaning of third argument
#endif
				// "conveniently", the third argument to HAL_FLASH_Program()
				// is a pointer on the H7 ...
				(uint32_t)(data + idx)
#else
#ifndef STM32G4xx_HAL_FLASH_H
#error HAL_FLASH_Program(): check meaning of third argument
#endif
				// ... and it's actual data on the G4
				((uint64_t*)(data + idx))[0]
#endif
		);
		if(HAL_OK != ret) {
			printError("write", HAL_FLASH_GetError(), address);
			printf("at idx %u\n\r", idx);
			return -2;
		}
		address += kFlashWordSize ;
	}
	HAL_FLASH_Lock();
#ifndef CFG_FLASHER
	printf("Successfully written flash starting at %#lx (%#lx bytes written)\n\r", startAddress, address - startAddress);
#endif // CFG_FLASHER
	return 0;
}

int storageWrite()
{
	uint32_t address = storage.baseAddress;
	int ret = storageWriteStatic(address, storage.data, kStorageSlotSize);
	if(0 == ret)
		storage.synced = 1;
	return ret;
}

static StorageWord_t* storageGetPtr(uint32_t index)
{
	return ((StorageWord_t*)storage.data) + index;
}

uint8_t storageIsSynced()
{
	return storage.synced;
}

StorageWord_t storageGet(uint32_t index)
{
	return storageGetPtr(index)[0];
}

void storageSet(uint32_t index, StorageWord_t value)
{
	if(index < storageGetLength()) {
		StorageWord_t* ptr = storageGetPtr(index);
		if(ptr[0] != value) {
			ptr[0] = value;
			storageWasSet();
		}
	}
}

StorageWord_t* storageGetData()
{
	return (StorageWord_t*)storage.data;
}

uint32_t storageGetLength()
{
	return kStorageSlotSize / sizeof(StorageWord_t);
}

void storageWasSet()
{
	storage.synced = 0;
}

static void storagePrint()
{
	StorageWord_t* data = storageGetData();
	size_t size = storageGetLength();
	for(size_t n = 0; n < size; ++n)
	{
		printf("%02x ", data[n]);
		if((n % 64) == 63)
			printf("\n\r");
	}
}

int storageTest(size_t sector, size_t slot, int verbose)
{
	StorageWord_t* data = storageGetData();
	size_t size = storageGetLength();
	storageInit(sector, slot);
	storageRead();
	if(verbose)
	{
		printf("read\n\r");
		storagePrint();
		printf("erase\n\r ");
	}
	int ret = storageErase(sector);
	if(ret)
		return ret;
	storageRead();
	for(size_t n = 0; n < size; ++n)
	{
		const uint8_t exp = 0xff;
		if(data[n] != exp)
		{
			if(verbose)
			{
				printf("storage error: should be %#02x at %d, is %#02x\n\r", exp, n, data[n]);
				storagePrint();
			}
			return 1;
		}
	}
	if(verbose)
	{
		printf("read\n\r");
		storagePrint();
	}
	for(size_t n = 0; n < size; ++n)
		data[n] = n;
	storageWasSet();
	storageWrite();
	if(verbose)
	{
		printf("set\n\r");
		storagePrint();
	}
	storageRead();
	for(size_t n = 0; n < size; ++n)
	{
		const uint8_t exp = (n & 255);
		if(exp != data[n])
		{
			if(verbose)
			{
				printf("storage error: should be %#02x at %d, is %#02x\n\r", exp, n, data[n]);
				storagePrint();
			}
			return 1;
		}
	}
	if(verbose)
	{
		printf("read\n\r");
		storagePrint();
	}
	return 0;
}
