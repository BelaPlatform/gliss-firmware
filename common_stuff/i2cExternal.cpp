#include "main.h"
#include <stdio.h>
#include "../TrillRackApplication/trill-neopixel/GlissProtocol.h"
#include <TrillRackApplication_bsp.h>
#include <string.h>

constexpr size_t kRxSize = 100;
static uint8_t rxData[kRxSize];
static uint8_t txData[kRxSize];
static uint8_t rxCount = 0;
static uint8_t txCount = 0;
static uint8_t dir;

static void processData(I2C_HandleTypeDef* hi2c)
{
	if(!rxCount)
		return;
//	printf("Received %d bytes: ", rxCount);
//	for(size_t n = 0; n < rxCount; ++n)
//		printf("%d ", rxData[n]);
//	printf("\n\r");
	// placeholder while we craft a response
	memset(txData, 0x55, sizeof(txData));
	txData[0] = 0x12;
	txData[1] = 0x34;
	uint8_t checksum = 0;
	for(ssize_t n = 0; n < rxCount - 1; ++n)
	{
		checksum += rxData[n];
	}
	if(checksum != rxData[rxCount - 1])
	{
		printf("co\n\r");
		return;
	}
#if !defined(CFG_DEBUG) && !defined(CFG_FLASHER)
	if(rxCount > 0)
	{
		--rxCount; // remove checksum
		if(6 == rxData[0])
		{
			gp_incoming(kGpI2c, rxData + 1, rxCount - 1);
		}
	}
#endif
}

// Slave DMA I2C is complicated.
// We start by calling HAL_I2C_EnableListen_IT() at initialisation time to listen
// for the address match event.
// This ensures HAL_I2C_AddrCallback() gets called when an address match event takes place.
// There we call HAL_I2C_DisableListen_IT() and call the HAL_I2C_Slave_{Transmit,Receive}_DMA()
// appropriately so that the DMA handles the transfer of the data for the remaining bytes in this
// transaction. We also keep track of the direction of the ongoing transfer.
// When the transfer is completed _successfully_, the HAL will either:
// - call HAL_I2C_{Listen,Receive}CpltCallback() when/once the number of transferred bytes matches
// the number we programmed when starting the DMA, or
// - call HAL_I2C_ErrorCallback() (yes, I know) with i2c error flag HAL_I2C_ERROR_AF set, if the
// master ends the transmission earlier
// In either case we call the appropriate handle{Rx,Tx}Complete(), which will also call
// HAL_I2C_EnableListen_IT() again to go back to the initial state.
// Well, almost. As a matter of fact, the same HAL ISR that calls either the error callback
// or the transfer complete callback will shortly thereafter also disable listening and call
// HAL_I2C_ListenCpltCallback(). In HAL_I2C_ListenCpltCallback() we therefore call HAL_I2C_EnableListen_IT()
// once again, to go back to the initial state - for real this time.
// Fun fact: if you don't call HAL_I2C_EnableListen_IT() from the transfer complete/"error" callback, you won't
// get the HAL_I2C_ListenCpltCallback() call, so we are force to call HAL_I2C_EnableListen_IT() twice in a row.
// "I set a flag so you can immediately unset it and tell me you unset it so I can set it again". Makes sense.
//
/// To get the number of received bytes we rely on the DMA's own counter, because hi2c's one will be 0.

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	int ret = HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	dir = TransferDirection;
	HAL_I2C_DisableListen_IT(hi2c);
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		HAL_I2C_Slave_Receive_DMA(hi2c, rxData, sizeof(rxData));
	} else {
		// if the master requests the data from the slave
		HAL_I2C_Slave_Transmit_DMA(hi2c, txData, sizeof(txData));
	}
}

static void handleRxComplete(I2C_HandleTypeDef *hi2c)
{
	rxCount = sizeof(rxData) - __HAL_DMA_GET_COUNTER(hi2c->hdmarx);
	processData(hi2c);
	int ret = HAL_I2C_EnableListen_IT(hi2c);
}

static void handleTxComplete(I2C_HandleTypeDef *hi2c)
{
	txCount = sizeof(rxData) - __HAL_DMA_GET_COUNTER(hi2c->hdmatx);;
	int ret = HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	handleRxComplete(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	handleTxComplete(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(&externalHi2c == hi2c)
	{
		if(HAL_I2C_ERROR_AF == HAL_I2C_GetError(hi2c))
		{
			// the host terminated the transfer early sending a NACK
			// before the maximum number of bytes
			if(I2C_DIRECTION_TRANSMIT == dir) // if the host has finished transmitting
			{
				handleRxComplete(hi2c);
			} else {
				handleTxComplete(hi2c);
			}
		} else {
			printf("I2C ERROR: %ld\n\r", HAL_I2C_GetError(hi2c));
		}
	} else {
		  // we cannot test this because we find no way of triggering it.
		  // Turning off the PSoC, for instance, doesn't trigger this.
		  // We'll leave it on in case we find something that triggers it at some point.
		  fprintf(stderr, "HAL_I2C_ErrorCallback. TODO: handle me\n\r");
	}
}
