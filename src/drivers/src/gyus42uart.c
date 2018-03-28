/*
 * gyus42uart.c
 *
 *  Created on: Mar 9, 2018
 *      Author: Yu Meng
 */

#include "FreeRTOS.h"
#include "task.h"
#include "system.h"

#include "gyus42uart.h"
#include <inttypes.h>
#include "debug.h"

static bool isInit;
static char Re_buf[11];
static uint16_t counter=0;
static char sign=0;
static uint16_t distance=0;
static uint32_t uartrate;
static uint16_t range_last_ut = 0;

void gyus42Init(uint32_t baudrate){
	if (isInit)
		return;
	uartrate = baudrate;
	uart1Init(uartrate);

	set_uart_rate(UART_RATE_HIGH);
	xTaskCreate(gyus42Task, GYUS42_TASK_NAME, GYUS42_TASK_STACKSIZE, NULL, GYUS42_TASK_PRI, NULL);

	isInit = true;

}

bool gyus42Test(void)
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

void gyus42Task(void* arg){
	  systemWaitStart();
	  TickType_t xLastWakeTime;
	  xLastWakeTime = xTaskGetTickCount();
//	  send_command_range(GET_CONTINUOUS_CMD);
	  uint16_t range_counts = 0;

	  while(1){

//		  range_last_u = get_continueours_data();
		  send_command_range(GET_SINGLEDATA_CMD);//查询输出
		  if (range_counts == 0){
			  send_command_savestate();
		  }
		  vTaskDelayUntil(&xLastWakeTime, M2T(GYUS42_TASK_FREQ));
		  xLastWakeTime = xTaskGetTickCount();
		  range_last_ut = get_range_data();

//		  DEBUG_PRINT("[gyus42]: drange_last_u[%d] ___%d___ \n",range_counts, range_last_u);
		  range_counts ++;

	  }
}

uint16_t get_range_last(void){
    return range_last_ut;
}

uint16_t get_range_data(void){
	uint16_t sum=0;
	uint16_t i, distance_tmp = 0;
	uint32_t start = xTaskGetTickCount();
	uint32_t now = xTaskGetTickCount();
	uint16_t index = 0;
		  while (now - start < GET_DATA_TIMEOUT_MS) {
			index++;
			now = xTaskGetTickCount();
			uint8_t ch;
			uart1GetDataWithTimout(&ch);
		    Re_buf[counter] = ch;
//		    DEBUG_PRINT("now: %"PRIu32"  \n",now);
//		    DEBUG_PRINT("index: %d ch[%d] %d \n",index, counter, Re_buf[counter]);
		    //Serial.println(Re_buf[counter++],HEX);
		    if(counter==0&&Re_buf[0]!=0x5A)
		    		continue;      // 检查帧头
//		    DEBUG_PRINT("ch[%d] %d \n", counter, Re_buf[counter]);
		    counter++;

		    if(counter==7)                //接收到数据
		    {
		       counter=0;                 //重新赋值，准备下一帧数据的接收
		       sign=1;
		       break;
		    }
		  }
		  if(sign)
		  {
		     sign=0;
		     for(i=0;i<6;i++)
		    	 sum += (uint8_t)Re_buf[i];

		     char sum8 = sum&0XFF;
//		     DEBUG_PRINT("[gyus42]:sum %d, sum8: %d",sum, sum8);
		     if(sum8 == Re_buf[i] )        //检查帧头，帧尾
		     {
//		    	 DEBUG_PRINT("[gyus42]:getbuf[4]: %d buf[5]: %d  ",(uint8_t)Re_buf[4],(uint8_t)Re_buf[5]);
		    	 distance_tmp = ((uint8_t)Re_buf[4]<<8)|(uint8_t)Re_buf[5];
//		    	 DEBUG_PRINT("[gyus42]:getdist: %d  ",distance_tmp);

		   }
		  }
		  if(distance_tmp > 0){
			  if(distance_tmp < 25 ){
				  distance_tmp = 25;
			  }
			  if(distance_tmp > 700){
				  distance_tmp = distance / 10;
			  }
			  distance_tmp = distance_tmp * 10;
			  distance = distance_tmp;

		  }
return distance;
}



uint16_t get_singal_data(void){


	send_command_range(GET_SINGLEDATA_CMD);//查询输出
	vTaskDelay(M2T(GET_DATA_TIMEOUT_MS));

	  return get_range_data();
}

uint16_t get_continueours_data(void){

	uint32_t now = xTaskGetTickCount();
	static uint32_t lastRequest = 0;
	static uint16_t rangeCache = 0;

//	DEBUG_PRINT("[mb1242]:lastRequest: %" PRIu32 "  rangeCache: %" PRIu16 " now: %" PRIu32 "  \n",lastRequest, rangeCache,now);

	// Check if range command was sent to the sensor at least [RANGING_CYCLE_MS] ms ago.
	if (lastRequest != 0 && (now - lastRequest) >= GET_DATA_TIMEOUT_MS)
	{
		// If so, the measured range can be retrieved.
		lastRequest = 0;
		rangeCache = get_range_data();
		return rangeCache;
	}
	else
	{
		if (lastRequest == 0)
		{
			// If not, send a range command to the sensor.
//			get_range_data();
			lastRequest = now;
		}
		// Return cached (old) range value if sensor hasn't finished measuring yet
		// or if the range command was sent recently. Thus, the returned range value
		// will never be 0.
		return rangeCache;
	}
}


void send_command_range(uint8_t command)
{
  uint8_t send_datas[4]={0xa5,0x56,0,0};
  send_datas[2]=command;
  send_datas[3]=send_datas[0]+send_datas[1]+send_datas[2];
  uart1SendData(4, send_datas);
	send_datas[0] = 0xa5;
	send_datas[1] = 0x5a;
	send_datas[2] = 0x01;
	send_datas[3] = 0X00;

	uart1SendData(4, send_datas);
}

void send_command_savestate(void){
	  uint8_t send_datas[4]={0xa5,0x5a,0x01,0X00};

//		send_datas[0] = 0xa5;
//		send_datas[1] = 0x5a;
//		send_datas[2] = 0x01;
//		send_datas[3] = 0X00;

		uart1SendData(4, send_datas);
}
void send_command_i2caddr(void)
{
  uint8_t send_datas[4]={0xa5,0x55,0x15,0};
  send_datas[3]=send_datas[0]+send_datas[1]+send_datas[2];
  uart1SendData(4, send_datas);

	send_datas[0] = 0xa5;
	send_datas[1] = 0x5a;
	send_datas[2] = 0x01;
	send_datas[3] = 0X00;

	uart1SendData(4, send_datas);
}

void set_uart_rate(uint32_t baudrate){
	uint8_t send_datas[4];
	if (baudrate == 9600){
		send_datas[0] = 0xa5;
		send_datas[1] = 0x58;
		send_datas[2] = 0xae;
		send_datas[3] = 0Xab;
	}
	if(baudrate == 115200){
		send_datas[0] = 0xa5;
		send_datas[1] = 0x58;
		send_datas[2] = 0xaf;
		send_datas[3] = 0Xac;
	}
	 uart1SendData(4, send_datas);

	send_datas[0] = 0xa5;
	send_datas[1] = 0x5a;
	send_datas[2] = 0x01;
	send_datas[3] = 0X00;

	uart1SendData(4, send_datas);


}

