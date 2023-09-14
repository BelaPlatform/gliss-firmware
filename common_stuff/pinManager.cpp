#include "main.h"
#include "pinManager.h"

// initialisations below copied from HAL_I2C_MspInit(), without any of the clocks stuff

void i2cPinsMode(enum I2cPinsMode mode)
{
	{
		// set function for I2C2 pins
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		/**I2C2 GPIO Configuration
		PA8     ------> I2C2_SDA
		PA9     ------> I2C2_SCL
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		uint32_t pinMode;
		uint32_t alternate;
		switch(mode)
		{
		default:
		case kI2cPinsModeExternal:
			pinMode = GPIO_MODE_INPUT;
			alternate = 0;
			break;
		case kI2cPinsModeI2c:
			pinMode =  GPIO_MODE_AF_OD;
			alternate = GPIO_AF4_I2C2;
			break;
		case kI2cPinsModeProgramming:
			pinMode = GPIO_MODE_OUTPUT_PP;
			alternate = 0;
			break;
		}
		GPIO_InitStruct.Mode = pinMode;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = alternate;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	{
		// set SCL pullup strong high, or high-Z
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		bool enablePullup = mode != kI2cPinsModeProgramming;
		GPIO_InitStruct.Pin = PSOC_PULLUP_SDA_Pin;
		GPIO_InitStruct.Mode = enablePullup ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = 0;
		HAL_GPIO_Init(PSOC_PULLUP_SDA_GPIO_Port, &GPIO_InitStruct);
		if(enablePullup) // if the pullup is disabled, the GPIO's value won't matter
			HAL_GPIO_WritePin(PSOC_PULLUP_SDA_GPIO_Port, PSOC_PULLUP_SDA_Pin, GPIO_PIN_SET);
	}
}

void psocPower(uint8_t power)
{
	// powering the PSoC via inverting FET
	HAL_GPIO_WritePin(TRILL_3V3_GPIO_Port, TRILL_3V3_Pin, power ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
