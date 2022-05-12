#include "bootloader.h"

#define BOOTLOADER_BKP_REG RTC_BKP_DR0
enum {
	// first entry actually unused here
	kBootloaderMagicRuntime = 0xBE7AE4EC, // EXEC
	kBootloaderMagicSystemBootloader = 0xBE7A5457, // SYST
};

enum {
	SYSTEM_BOOTLOADER_START = 0x1FFF0000, // for STM32G4, got this from AN2602 or 2.6.1 of the TRM
	RUNTIME_START = 0x08000000,
};

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

__attribute__((noreturn)) void bootloaderResetTo()
{
	uint32_t value = kBootloaderMagicSystemBootloader;
	prepareJumpOrReset();
	// Write Back Up Register Data
	HAL_PWR_EnableBkUpAccess();
	// Writes a data in a RTC Backup data Register 1
	extern RTC_HandleTypeDef hrtc;
	HAL_RTCEx_BKUPWrite(&hrtc, BOOTLOADER_BKP_REG, value);
	HAL_PWR_DisableBkUpAccess();
	HAL_NVIC_SystemReset();
	while(1) // compensating for HAL_NVIC_SystemReset() not being marked as noreturn
		;
}

__attribute__((noreturn)) void bootloaderJumpTo(uint32_t BootAddr)
{
	// more from https://st.force.com/community/s/article/STM32H7-bootloader-jump-from-application
	prepareJumpOrReset();

	/* Set up the jump to bootloader address + 4 */
	__attribute__((noreturn)) void (*SysMemBootJump)(void);
	SysMemBootJump = (void*) /* cast to avoid compiler warning*/ (void (*)(void)) (*((uint32_t *) ((BootAddr + 4))));

	/* Set the main stack pointer to the bootloader stack */
	__set_MSP(*(uint32_t *)BootAddr);

	/* Call the function to jump to bootloader location */
	SysMemBootJump();
}

// this must be called before any DMA/Timer/IRQ configuration takes place
void bootloaderJumpIfNeeded(RTC_HandleTypeDef* hrtc) {
	// do jump to bootloader or runtime if the backup register indicates
	// that we are not on the right one.
	uint32_t value = HAL_RTCEx_BKUPRead(hrtc, BOOTLOADER_BKP_REG);
	if(kBootloaderMagicSystemBootloader == value || 1)
	{
		// restore content so we don't get stuck here forever
		HAL_RTCEx_BKUPWrite(hrtc, BOOTLOADER_BKP_REG, kBootloaderMagicRuntime);
		HAL_PWR_DisableBkUpAccess();
		// ensure elsewhere that the host is forced to
		// re-enumerate the peripheral so it can detect it now
		// is a DFU device.
		bootloaderJumpTo(SYSTEM_BOOTLOADER_START);
	}
	// otherwise keep going as we are
}
