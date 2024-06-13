#include "main.h"
#include <stdio.h>
#include "../TrillRackApplication/trill-neopixel/GlissProtocol.h"
#include <TrillRackApplication_bsp.h>

// see https://controllerstech.com/stm32-as-i2c-slave-part-3/

constexpr size_t kRxSize = 100;
static uint8_t rxData[kRxSize];
static uint8_t rxCount = 0;

static void processData()
{
//	printf("Received %d bytes: ", rxCount);
//	for(size_t n = 0; n < rxCount; ++n)
//		printf("%d ", rxData[n]);
//	printf("\n\r");
	gp_incoming(kGpI2c, rxData, rxCount);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rxCount = 0;
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		// receive using sequential function.
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxData + rxCount, 1, I2C_FIRST_FRAME);
	} else { // if the master requests the data from the slave
		// dummy response, which is enough for now to have i2cdetect return success
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, rxData, 1, I2C_FIRST_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rxCount++;
	if(rxCount < kRxSize)
	{
		if(kRxSize - 1 == rxCount)
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxData+rxCount, 1, I2C_LAST_FRAME);
		else
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxData+rxCount, 1, I2C_NEXT_FRAME);
	}
	if(kRxSize == rxCount)
	{
		processData();
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(&externalHi2c == hi2c)
	{
		if(HAL_I2C_ERROR_AF == HAL_I2C_GetError(hi2c))
		{
			// the host terminated the transmission sending a NACK
			// before the maximum number of bytes
			processData();
		}
	} else {
		  // we cannot test this because we find no way of triggering it.
		  // Turning off the PSoC, for instance, doesn't trigger this.
		  // We'll leave it on in case we find something that triggers it at some point.
		  fprintf(stderr, "HAL_I2C_ErrorCallback. TODO: handle me\n\r");
	}
}
