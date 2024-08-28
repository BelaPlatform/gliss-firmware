#include "main.h"
#include <stdio.h>
#include "../TrillRackApplication/trill-neopixel/GlissProtocol.h"
#include <TrillRackApplication_bsp.h>
#include <string.h>
#include "sysex.h"
extern "C" {
#include "z_ringbuffer.h"
}
#include "i2cExternal.h"

//#define VERBOSE
//#define SIMULATE_ERRORS

static enum I2cError {
	kErrorNo = 0,
	kOutgoingMaxMsgLength = 253, // higher values would interfere with the error reporting
	kErrorFull = 254,
	kErrorChecksum = 255,
} errorCode;

static uint8_t rxData[257];
static uint8_t txData[kOutgoingMaxMsgLength + 1];
static uint8_t rxCount = 0;
static uint8_t txCount = 0;
static uint8_t dir;
static ring_buffer i2cRxQ;
static ring_buffer i2cTxQ;


int i2cMidiInit()
{
	static char rxRbBuf[256];
	static char txRbBuf[256];
	rb_init_preallocated(&i2cRxQ, rxRbBuf, sizeof(rxRbBuf));
	rb_init_preallocated(&i2cTxQ, txRbBuf, sizeof(txRbBuf));
	if (HAL_I2C_EnableListen_IT(&externalHi2c) != HAL_OK)
		printf("error enabling external I2C\n\r");
	return 0;
}

static int computeChecksum(uint8_t* data, size_t count)
{
	uint8_t checksum = 0;
	for(ssize_t n = 0; n < rxCount - 1; ++n)
		checksum += data[n];
	return checksum;
}

static void i2cMidiDataRx(I2C_HandleTypeDef* hi2c)
{
	if(!rxCount)
		return;
//	for(size_t n = 0; n < rxCount; ++n)
//		printf("%d ", rxData[n]);
//	printf("\n\r");
	// placeholder while we craft a response
	txData[0] = 0;
	uint8_t checksum = computeChecksum(rxData, rxCount);
#ifdef SIMULATE_ERRORS
	static int cs = 0;
	if(cs++ >= 6)
		cs = 0;
	if(4 == cs)
		checksum += 1; // cause an error
#endif // SIMULATE_ERRORS
	if(checksum != rxData[rxCount - 1])
	{
		printf("chk\n\r"); // bad checksum
		errorCode = kErrorChecksum;
		return;
	}
	--rxCount; // remove checksum
	int ret = rb_available_to_write(&i2cRxQ);
#ifdef SIMULATE_ERRORS
	if(10 == cs)
		ret--;
#endif // SIMULATE_ERRORS
	if(ret < rxCount)
	{
		printf("full\n\r"); // not enough space
		errorCode = kErrorFull;
		return;
	}
	rb_write_to_buffer(&i2cRxQ, 2, &rxCount, sizeof(rxCount), (const char*)rxData, rxCount);
}

int sysexSendI2c(const uint8_t* payload, size_t len)
{
	int ret = rb_available_to_write(&i2cTxQ);
	if(int(len) > ret || len > kOutgoingMaxMsgLength)
	{
		printf("Can't fit this message\n\r");
		return -1;
	}
	char charLen = len;
#ifdef VERBOSE
	printf("Writ %d\n\r", len);
#endif // VERBOSE
	ret = rb_write_to_buffer(&i2cTxQ, 2, &charLen, sizeof(charLen), (const char*)payload, len);
	if(ret)
	{
		printf("Error writing message\n\r");
		return -1;
	}
	return 0;
}

void i2cProcessIncomingFromMainThread()
{
	size_t available = rb_available_to_read(&i2cRxQ);
	if(available > 0)
	{
		uint8_t count;
		int ret = rb_read_from_buffer(&i2cRxQ, (char*)&count, sizeof(count));
		if(ret < 0)
		{
			printf("error bytes not ready when getting them\n\r");
			return;
		}
		--available;
		if(available < count)
		{
			printf("Not enough data in the buffer\n\r");
			return;
		}
		uint8_t data[count];
		ret = rb_read_from_buffer(&i2cRxQ, (char*)data, sizeof(data));
		if(ret < 0)
		{
			printf("error %d bytes not ready when getting them: %d\n\r", sizeof(data), ret);
			return;
		}
		midiHandleIncoming(kSysexI2c, data, count);
	}
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
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
#ifdef VERBOSE
	printf("adr");
#endif // VERBOSE
	dir = TransferDirection;
	HAL_I2C_DisableListen_IT(hi2c);
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		HAL_I2C_Slave_Receive_DMA(hi2c, rxData, sizeof(rxData));
	} else {
		static uint8_t count;
		static bool waitingForCount = true;
		// if the master requests data from the slave
		int available = rb_available_to_read(&i2cTxQ);
		size_t len = 0;
		if(kErrorNo != errorCode)
		{
			txData[0] = errorCode;
			errorCode = kErrorNo;
			len = 1;
		} else if(available <= 0)
		{
			txData[0] = 0;
			len = 1;
		} else {
			if(waitingForCount)
			{
				// read count into count
				int ret = rb_read_from_buffer(&i2cTxQ, (char*)&count, sizeof(count));
				if(ret)
				{
					printf("reading from ISR: %d\n\r", ret);
					return;
				}
				len = 1;
				txData[0] = count;
#ifdef VERBOSE
				printf("S %d (av: %d)\n\r", count, available);
#endif // VERBOSE
				if(0 == count)
				{
					printf("WTF\n\r");
				}
				waitingForCount = false;
			} else {
				if(count > int(sizeof(txData) - 1))
				{
					printf("reading from ISR2: %d\n\r", count);
					return;
				}
				// read message body into txData
				int ret = rb_read_from_buffer(&i2cTxQ, (char*)txData, count);
				if(ret)
				{
					printf("reading from ISR3: %d\n\r", ret);
					return;
				}
				len = count;
#ifdef VERBOSE
				printf("S [%d]: ", count);
				for(size_t n = 0; n < count && n < 6; ++n)
					printf("%02x ", txData[n]);
				printf("\n\r");
#endif // VERBOSE
				waitingForCount = true;
			}
		}
		txData[len] = computeChecksum(txData, len);
		HAL_I2C_Slave_Transmit_DMA(hi2c, txData, len + 1); // include checksum
	}
}

static void handleRxComplete(I2C_HandleTypeDef *hi2c)
{
	rxCount = sizeof(rxData) - __HAL_DMA_GET_COUNTER(hi2c->hdmarx);
	i2cMidiDataRx(hi2c);
	HAL_I2C_EnableListen_IT(hi2c);
}

static void handleTxComplete(I2C_HandleTypeDef *hi2c)
{
	txCount = sizeof(rxData) - __HAL_DMA_GET_COUNTER(hi2c->hdmatx);;
	HAL_I2C_EnableListen_IT(hi2c);
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
			// e.g.: BERR
			printf("I2C ERROR: %ld\n\r", HAL_I2C_GetError(hi2c));
			HAL_I2C_EnableListen_IT(hi2c);
		}
	} else {
		  // we cannot test this because we find no way of triggering it.
		  // Turning off the PSoC, for instance, doesn't trigger this.
		  // We'll leave it on in case we find something that triggers it at some point.
		  fprintf(stderr, "HAL_I2C_ErrorCallback. TODO: handle me\n\r");
	}
}
