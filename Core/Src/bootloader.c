#include "bootloader.h"
#include "../../common_stuff/verificationBlock.h"
#define BOOTLOADER_BKP_REG RTC_BKP_DR0

uint32_t const BL_SYSTEM_BOOTLOADER_START = 0x1FFF0000; // for STM32G4, got this from AN2602 or 2.6.1 of the TRM
uint32_t const BL_USER_FLASHER_START = 0x08002000;
uint32_t const BL_USER_APPLICATION_START = 0x08010000;
uint32_t const BL_USER_OFFSET = 0x800; // the first sector is used for sectionGood(): code starts 2k laer
uint32_t const BL_USER_SETTINGS_START = 0x08060000;
uint32_t const BL_VERIFICATION_TAG_OFFSET = 2044; // 4 last bytes of sector


extern void main();
int bootloaderIsFlasher()
{
    if(BL_USER_APPLICATION_START < BL_USER_FLASHER_START && (uint32_t)main > BL_USER_FLASHER_START)
        return 1;
    else if (BL_USER_FLASHER_START < BL_USER_APPLICATION_START && (uint32_t)main < BL_USER_APPLICATION_START)
        return 1;
    return 0;
}

int bootloaderIsPartOf(uint32_t ptr, BootloaderResetDest_t dest)
{
	switch(dest)
	{
	case kBootloaderMagicUserFlasher:
		return ptr >= BL_USER_FLASHER_START && ptr < BL_USER_APPLICATION_START;
	case kBootloaderMagicUserApplication:
		return ptr >= BL_USER_APPLICATION_START && ptr < BL_USER_SETTINGS_START;
	default:
		return 0;
	}
}

void bootloaderSetVector()
{
    // set the interrupt vector address.
    // should be called as soon as possible from main()

    // The vector address is set to 0x08000000 by SystemInit, but
    // we have to change it when starting the code at a
    // different location. Unfortunately it cannot be set
    // via Makefile or HAL (yet).
    // See e.g.: https://github.com/STMicroelectronics/STM32CubeL0/pull/10 and related issues
    uint32_t addr;
    if(bootloaderIsFlasher())
        addr = (uint32_t)BL_USER_FLASHER_START;
    else
        addr = (uint32_t)BL_USER_APPLICATION_START;
    // printf("main is %p. Vector is %p. Setting it to %p\n\r", main, (void*)SCB->VTOR, (void*)addr);
    // a similar solution is mentioned in https://stackoverflow.com/a/28689746/2958741
    SCB->VTOR = addr;
}

static void prepareJumpOrReset()
{
	// from https://st.force.com/community/s/article/STM32H7-bootloader-jump-from-application
	/* Disable all interrupts */
	__disable_irq();

	/* Disable Systick timer */
	SysTick->CTRL = 0;

	/* Set the clock to the default state */
	HAL_RCC_DeInit();

	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for(unsigned int i = 0; i < 5; ++i)
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}
	/* Re-enable all interrupts */
	__enable_irq();
}

static void writeToBackupRegister(RTC_HandleTypeDef* hrtc, uint32_t value)
{
	// Write Back Up Register Data
	HAL_PWR_EnableBkUpAccess();
	// Writes a data in a RTC Backup data Register 1
	HAL_RTCEx_BKUPWrite(hrtc, BOOTLOADER_BKP_REG, value);
	HAL_PWR_DisableBkUpAccess();
}

__attribute__((noreturn)) void bootloaderResetTo(BootloaderResetDest_t dest)
{
	prepareJumpOrReset();
	extern RTC_HandleTypeDef hrtc;
	writeToBackupRegister(&hrtc, dest);
	HAL_NVIC_SystemReset();
	while(1) // compensating for HAL_NVIC_SystemReset() not being marked as noreturn
		;
}

__attribute__((noreturn)) void bootloaderJumpToAddr(uint32_t BootAddr)
{
	// more from https://st.force.com/community/s/article/STM32H7-bootloader-jump-from-application
	prepareJumpOrReset();

	/* Set up the jump to bootloader address + 4 */
	__attribute__((noreturn)) void (*SysMemBootJump)(void);
	 SysMemBootJump = (void*)(*(uint32_t*)(BootAddr + 4)); // offset + 4 contains the address of the Reset_Handler

	/* Set the main stack pointer to the bootloader stack */
	__set_MSP(*(uint32_t *)BootAddr);

	/* Call the function to jump to bootloader location */
	SysMemBootJump();
}

static int sectionGood(BootloaderResetDest_t dest)
{
	uint32_t target;
	switch(dest)
	{
	case kBootloaderMagicUserApplication:
		target = BL_USER_APPLICATION_START;
		break;
	case kBootloaderMagicUserFlasher:
		target = BL_USER_FLASHER_START;
		break;
	default:
		return 0;
	}
	// start: 0, size: 512 reserved for isr_vector
	// start: 512, size: 128 stringId
	// start: 640, size: 256 git hashes
	// start: 896, size: ...  reserved
	// start: 2044, size: 4 verification tag
	for(size_t n = 0; n < sizeof(kVerificationBlock.tag); ++n)
		if((kVerificationBlock.tag[n] != ((uint8_t*)target + BL_VERIFICATION_TAG_OFFSET)[n]))
			return 0;
	return 1;
}

uint32_t  bootloaderGetDest(RTC_HandleTypeDef* hrtc) {
	uint32_t value = HAL_RTCEx_BKUPRead(hrtc, BOOTLOADER_BKP_REG);
	// return where we should jump to.
	// If a value is specified, jump there
	switch(value)
	{
	default:
		break;
	case kBootloaderMagicUserFlasher:
	case kBootloaderMagicUserApplication:
	case kBootloaderMagicSystemBootloader:
		return value;
	}
	// otherwise do some heuristics to make sure we jump to a valid location
	if(sectionGood(kBootloaderMagicUserApplication))
		return kBootloaderMagicUserApplication;
	if(sectionGood(kBootloaderMagicUserFlasher))
		return kBootloaderMagicUserFlasher;
	return kBootloaderMagicSystemBootloader;
}

void bootloaderJump(RTC_HandleTypeDef* hrtc, BootloaderResetDest_t to)
{
	// restore content to default so we don't get stuck here forever
	writeToBackupRegister(hrtc, kBootloaderMagicNone);
	// ensure elsewhere that the host is forced to
	// re-enumerate the peripheral so it can detect it now
	// is a DFU device.
	uint32_t dest = 0;
	switch(to)
	{
	case kBootloaderMagicUserApplication:
		dest = BL_USER_APPLICATION_START;
		break;
	case kBootloaderMagicUserFlasher:
		dest = BL_USER_FLASHER_START;
		break;
	case kBootloaderMagicSystemBootloader:
		dest = BL_SYSTEM_BOOTLOADER_START;
		break;
	case kBootloaderMagicNone:
		dest = 0;
		break;
	}
	if(dest)
		bootloaderJumpToAddr(dest);
}

// this must be called before any DMA/Timer/IRQ configuration takes place
void bootloaderJumpIfNeeded(RTC_HandleTypeDef* hrtc) {
	// do jump to bootloader or runtime if the backup register indicates
	// that we are not on the right one.
	uint32_t to = bootloaderGetDest(hrtc);
	if(to)
		bootloaderJump(hrtc, to);
	// otherwise keep going as we are
}
