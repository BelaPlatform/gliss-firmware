#include "bootloader.h"

#define BOOTLOADER_BKP_REG RTC_BKP_DR0

enum {
	SYSTEM_BOOTLOADER_START = 0x1FFF0000, // for STM32G4, got this from AN2602 or 2.6.1 of the TRM
	USER_BOOTLOADER_START = 0x08000000,
	USER_APPLICATION_START = 0x08010000,
};

extern void main();
int bootloaderIs()
{
    if(USER_APPLICATION_START < USER_BOOTLOADER_START && (uint32_t)main > USER_BOOTLOADER_START)
        return 1;
    else if (USER_BOOTLOADER_START < USER_APPLICATION_START && (uint32_t)main < USER_APPLICATION_START)
        return 1;
    return 0;
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
    if(bootloaderIs())
        addr = USER_BOOTLOADER_START;
    else
        addr = USER_APPLICATION_START;
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

uint32_t  bootloaderShouldJump(RTC_HandleTypeDef* hrtc) {
	uint32_t value = HAL_RTCEx_BKUPRead(hrtc, BOOTLOADER_BKP_REG);
	switch(value)
	{
	case kBootloaderMagicUserBootloader:
	case kBootloaderMagicUserApplication:
	case kBootloaderMagicSystemBootloader:
		return value;
	}
	return 0;
}

void bootloaderJump(RTC_HandleTypeDef* hrtc, BootloaderResetDest_t to)
{
	// restore content so we don't get stuck here forever
	writeToBackupRegister(hrtc, kBootloaderMagicUserApplication);
	// ensure elsewhere that the host is forced to
	// re-enumerate the peripheral so it can detect it now
	// is a DFU device.
	uint32_t dest = 0;
	switch(to)
	{
	case kBootloaderMagicUserApplication:
		dest = USER_APPLICATION_START;
		break;
	case kBootloaderMagicUserBootloader:
		dest = USER_BOOTLOADER_START;
		break;
	case kBootloaderMagicSystemBootloader:
		dest = SYSTEM_BOOTLOADER_START;
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
	uint32_t to = bootloaderShouldJump(hrtc);
	if(to)
		bootloaderJump(hrtc, to);
	// otherwise keep going as we are
}
