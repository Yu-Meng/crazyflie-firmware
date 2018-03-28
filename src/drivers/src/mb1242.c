/*
 * mb1242.c
 *
 *  Created on: Mar 7, 2018
 *      Author: Yu Meng
 */
#include "FreeRTOS.h"
#include "task.h"
#include "mb1242.h"
#include "debug.h"
#include <inttypes.h>
// Optimal value for getting "real-time" range values is 80 ms (see datasheet).
// Be sure to call mb1242Read() more than (1000 / RANGING_CYCLE_MS) times
// per second to avoid dropping of range values. The faster you call mb1242Read()
// than RANGING_CYCLE_MS, the more returned range values will be cached (old) ones.
#define RANGING_CYCLE_MS	150

static bool isInit;
static I2C_Dev *I2Cx;
static uint8_t devAddr;
static uint8_t sonarBuffer[2];

void mb1242Init(I2C_Dev *i2cPort)
{
	if (isInit)
		return;

	I2Cx = i2cPort;
	devAddr = MB1242_I2C_ADDRESS;
	//i2cdevInit(I2Cx);
	DEBUG_PRINT("[mb1242]: sclpin %" PRIu32 " sdapin %" PRIu32 " clkspeed %" PRIu32 " \n", I2Cx->def->gpioSCLPin,I2Cx->def->gpioSDAPin, I2Cx->def->i2cClockSpeed);
	DEBUG_PRINT("[mb1242]:mb1242 init: %d \n ",devAddr);

//	mb1242SetAddress(0b1111000);
//	DEBUG_PRINT("[mb1242]:mb1242 init2: %d \n ",devAddr);
	isInit = true;
}

bool mb1242Test(void)
{
	if (!isInit)
		return false;

	return mb1242TestConnection();
}

bool mb1242TestConnection(void)
{
//	return (mb1242GetDeviceInfo() > 0);
//	uint8_t add = 0x00;
//	for (add = 0x00; add <= 0xFF ; add ++){
//		i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, add);
//		vTaskDelay(M2T(200));
//		uint8_t sonarB[2];
//		i2cdevRead(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 2, sonarB);
//		if (sonarB[0] > 0 || sonarB[1] > 0){
//			DEBUG_PRINT("got data: [0]%d, [1]%d, index: %d", sonarB[0], sonarB[1], add);
//		}else{
//			if(add == 0xFF){
//				DEBUG_PRINT("game over");
//			}
//		}
//	}


	return true;
}

uint16_t mb1242GetDeviceInfo(void)
{
	// Read the device's info bytes (available until first range reading is requested)
	i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, MB1242_TAKE_RANGE_READING_CMD);
	uint32_t lastRequest = xTaskGetTickCount();
	uint32_t now =  xTaskGetTickCount();
	while(now - lastRequest < 50){
		now = xTaskGetTickCount();
	}

	i2cdevRead(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 2, sonarBuffer);
	DEBUG_PRINT("[mb1242]:mb1242GetDeviceInfo: [0] %d,  [1]%d \n",sonarBuffer[0], sonarBuffer[1]);
	return (((uint16_t)sonarBuffer[0]) << 8) | sonarBuffer[1];
}

void mb1242RequestRangeReading(void)
{
	// Write command 0x51 to request a range reading.
	bool req = i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 0x51);
	DEBUG_PRINT("[mb1242]:mb1242RequestRangeReading -----%d------ \n", req);
}

uint16_t mb1242GetRangeReading(void)
{
	// Get high and low byte of range reading.
	uint8_t sonarB[2];
	bool read = i2cdevRead(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 2, sonarB);
	DEBUG_PRINT("[mb1242]:read: ===%d===  sonarB[0]: %d  sonarB[1]: %d \n", read, sonarB[0], sonarB[1]);
	return (((uint16_t)sonarB[0]) << 8) | sonarB[1];
}

// Coordinates the sequence of requesting and reading range values from the sensor.
uint16_t mb1242Read(void)
{
	uint32_t now = xTaskGetTickCount();
	static uint32_t lastRequest = 0;
	static uint16_t rangeCache = 0;

	DEBUG_PRINT("[mb1242]:lastRequest: %" PRIu32 "  rangeCache: %" PRIu16 " now: %" PRIu32 "  \n",lastRequest, rangeCache,now);

	// Check if range command was sent to the sensor at least [RANGING_CYCLE_MS] ms ago.
	if (lastRequest != 0 && (now - lastRequest) >= RANGING_CYCLE_MS)
	{
		// If so, the measured range can be retrieved.
		lastRequest = 0;
		rangeCache = mb1242GetRangeReading();
		return rangeCache;
	}
	else
	{
		if (lastRequest == 0)
		{
			// If not, send a range command to the sensor.
			mb1242RequestRangeReading();
			lastRequest = now;
		}
		// Return cached (old) range value if sensor hasn't finished measuring yet
		// or if the range command was sent recently. Thus, the returned range value
		// will never be 0.
		return rangeCache;
	}
}

void mb1242SetAddress(uint8_t newAddress)
{
	uint8_t writeData[3];
	writeData[0] = MB1242_ADDR_UNLOCK_1_CMD;
	writeData[1] = MB1242_ADDR_UNLOCK_2_CMD;
	writeData[2] = newAddress;

	// TODO: Check new address for validity.
	// Already in use by on-bard hardware: 0x50, 0x0C, 0x1E, 0x5D, 0x68
	// Invalid address values stated in the datasheet: 0x00, 0x50, 0xA4, 0xAA

	if (i2cdevWrite(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 3, writeData))
		devAddr = newAddress;
}

