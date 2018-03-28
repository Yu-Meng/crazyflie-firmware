/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "i2cdev.h"
#include "zranger.h"
#include "vl53l0x.h"

#include "stabilizer_types.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "arm_math.h"

#include "mb1242.h"
#include "gyus42uart.h"

#include "stabilizer.h"

//// Measurement noise model
//static float expPointA = 1.0f;
//static float expStdA = 0.0025f; // STD at elevation expPointA [m]
//static float expPointB = 1.3f;
//static float expStdB = 0.2f;    // STD at elevation expPointB [m]
//static float expCoeff;

// Measurement noise model
static float expPointA = 1.0f;
static float expStdA = 0.0025f; // STD at elevation expPointA [m]
static float expPointB = 2.0f;
static float expStdB = 0.01f;    // STD at elevation expPointB [m]
static float expCoeff;


// Measurement noise model for gy-us42
static float expPointA_u = 1.0f;
static float expStdA_u = 0.01f; // STD at elevation expPointA [m]
static float expPointB_u = 5.0f;
static float expStdB_u = 0.05f;    // STD at elevation expPointB [m]
static float expCoeff_u;


//#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]

#define RANGE_OUTLIER_LIMIT 6500 // the measured range is in [mm]

static uint16_t range_last = 0;
static uint16_t range_last_u = 0;
//static uint16_t range_last_l = 0;
static float accZ = 0;

static bool isInit;

static VL53L0xDev dev;

void zRangerInit(DeckInfo* info)
{

  if (isInit)
    return;


  vl53l0xInit(&dev, I2C1_DEV, true);

  //Change to MB1242 Init
//  mb1242Init(I2C1_DEV);
//  gyus42UartInit(UART_RATE_HIGH);
//  set_uart_rate(UART_RATE_HIGH);

  xTaskCreate(zRangerTask, ZRANGER_TASK_NAME, ZRANGER_TASK_STACKSIZE, NULL, ZRANGER_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  expCoeff_u = logf(expStdB_u / expStdA_u) / (expPointB_u - expPointA_u);

  isInit = true;
}

bool zRangerTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus  = vl53l0xTestConnection(&dev);
  // Change to mb1242 test
//  testStatus = mb1242TestConnection();
//  testStatus = vl53l0xTestConnection(&dev) || gyus42uartTest();
//  DEBUG_PRINT("[mb1242]:testinfo: %d \n",testStatus);

  return testStatus;
}

void zRangerTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;


  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodPreRange, 18);
  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodFinalRange, 14);
//    vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodPreRange, 12);
//    vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodFinalRange, 8);
  vl53l0xStartContinuous(&dev, 0);

//  send_command_range(GET_CONTINUOUS_CMD);

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    range_last = vl53l0xReadRangeContinuousMillimeters(&dev);

    // Set range_last with mb1242Read()

//    range_last = mb1242Read();

    //read from US-42
//    range_last_u = get_continueours_data();
    range_last_u = get_range_last();
    accZ = getAccZ();
//
//    if (range_last <= 500){
//    	range_last = range_last_l;
//
//    }
//
//    if ( range_last > 500 && range_last <= 800){
//    	if(range_last_l < 1200){
//    		if(range_last_u < 1200 && range_last_u >400){
//    			range_last = (range_last_u + range_last_l) / 2;
//    		}else{
//    			range_last = range_last_l;
//    		}
//    	}else{
//    		if(range_last_u < 1200){
//    			range_last = range_last_u;
//    		}
//    	}
//    }
//    if (range_last >  800){
//
//    	range_last = range_last_u;
//    }

//    DEBUG_PRINT("[zranger]:range_last_l %d , range_last_u %d and range_last %d\n",range_last_l, range_last_u,range_last);
//    DEBUG_PRINT("l %d , u %d and a %d\n",range_last_l, range_last_u,range_last);
//    DEBUG_PRINT("U %d , L %d, acc %f \n", range_last_u,range_last, (double)accZ);

    // check if range is feasible and push into the kalman filter
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (getStateEstimator() == kalmanEstimator &&
        range_last < RANGE_OUTLIER_LIMIT) {
      // Form measurement
      tofMeasurement_t tofData;
      tofData.timestamp = xTaskGetTickCount();
      tofData.distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
//      double dis = tofData.distance;
//      DEBUG_PRINT("[zranger]: tofData.distancet %.2f \n", dis);
//      tofData.distance = (float)range_last * 0.01f; // Scale from [cm] to [m]
      tofData.stdDev = expStdA * (1.0f  + expf( expCoeff * ( tofData.distance - expPointA)));
      estimatorKalmanEnqueueTOF(&tofData);
    }

//    DEBUG_PRINT("[zranger]: dev.measurement_timing_budget_ms ___%d___ \n", dev.measurement_timing_budget_ms);
    vTaskDelayUntil(&xLastWakeTime, M2T(dev.measurement_timing_budget_ms));
//    uint16_t measurement_timing_budget_ms = 60;
//    vTaskDelayUntil(&xLastWakeTime, M2T(10));
  }
}

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT) {
      zrange->distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
//      zrange->distance = (float)range_last * 0.01f; // Scale from [cm] to [m]
      zrange->timestamp = tick;
      updated = true;
    }
  }
  return updated;
}

static const DeckDriver zranger_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcZRanger",
  .usedGpio = 0x0C,

  .init = zRangerInit,
  .test = zRangerTest,
};

DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, zrange, &range_last)
LOG_GROUP_STOP(range)
