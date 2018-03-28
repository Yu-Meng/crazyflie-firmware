/*
 * gyus42uart.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Yu Meng
 */

#ifndef GYUS42UART_H_
#define GYUS42UART_H_

#include <stdbool.h>
#include "uart1.h"



#define UART_RATE_DEF  9600
#define UART_RATE_HIGH  115200
#define GET_SINGLEDATA_CMD 0x01
#define GET_CONTINUOUS_CMD 0x02
#define GET_DATA_TIMEOUT_MS 45
#define GYUS42_TASK_FREQ 100

void gyus42Init(uint32_t baudrate);
bool gyus42Test(void);
void gyus42Task(void* arg);
uint16_t get_range_last(void);
uint16_t get_range_data(void);
uint16_t get_singal_data(void);
uint16_t get_continueours_data(void);
void send_command_range(uint8_t command);
void send_command_i2caddr(void);
void set_uart_rate(uint32_t baudrate);
void send_command_savestate(void);
#endif /* GYUS42UART_H_ */
