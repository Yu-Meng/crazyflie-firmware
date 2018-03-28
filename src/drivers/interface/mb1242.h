/*
 * mb1242.h
 *
 *  Created on: Mar 7, 2018
 *      Author: Yu Meng
 */

#ifndef MB1242_H
#define MB1242_H

#include <stdbool.h>
#include "i2cdev.h"

#define MB1242_I2C_ADDRESS				0b1110000
#define MB1242_TAKE_RANGE_READING_CMD	0x51
#define MB1242_ADDR_UNLOCK_1_CMD		0xAA
#define MB1242_ADDR_UNLOCK_2_CMD		0xA5
#define MB1242_ADDR_WRITE 0XE0
#define MB1242_ADDR_READ 0XE1

void mb1242Init(I2C_Dev *i2cPort);
bool mb1242Test(void);
bool mb1242TestConnection(void);
uint16_t mb1242GetDeviceInfo(void);
void mb1242RequestRangeReading(void);
uint16_t mb1242GetRangeReading(void);
uint16_t mb1242Read(void);
void mb1242SetAddress(uint8_t newAddress);

#endif // MB1242_H
