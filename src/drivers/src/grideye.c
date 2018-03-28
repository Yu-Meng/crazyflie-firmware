/*
 * grideye.c
 *
 *  Created on: Jan 1, 2018
 *      Author: Yu Meng
 */

#define DEBUG_MODULE "GRE"

#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "i2cdev.h"
#include "grideye.h"

#include "stabilizer_types.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "arm_math.h"

#define CORRIDOR_HEIGHT       2650

#define RAW_DATA_FRAME_NUM    1
#define BG_FRAME_NUM          100
#define PEAK_THREHOLD         1.5f
//
//#define LEFT_UP      7
//#define RIGHT_UP     63
//#define LEFT_DOWN    0
//#define RIGHT_DOWN   56

#define LEFT_UP      63
#define RIGHT_UP     56
#define LEFT_DOWN    7
#define RIGHT_DOWN   0

#define MAXRANGE 80

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

//static uint16_t io_timeout = 0;
//static bool did_timeout;
//static uint16_t timeout_start_ms;

static uint8_t buffer[31];

uint16_t range_last = 0;
uint16_t range_last_down = 0;
uint16_t range_last_front_l = 0;
uint16_t range_last_front_r = 0;

static uint32_t yellowGroupH = 0;
static uint32_t yellowGroupL = 0;
static uint32_t orangeGroupH = 0;
static uint32_t orangeGroupL = 0;
static uint32_t redGroupH = 0;
static uint32_t redGroupL = 0;

static uint8_t  rawDataFrameIndex = 0;
static float    rawData[64*RAW_DATA_FRAME_NUM] = {0};
static uint8_t  bgFrameIndex = 0;
static float    bgData[64*BG_FRAME_NUM] = {0};

static bool isCalibrate = false;
static bool checkCalibrate = false;
static float avgCal[64] = {0};
static float squareAvg[64] = {0};
static float stdCal[64] = {0};
static float zscoreData[64*RAW_DATA_FRAME_NUM] = {0};
static float zscore[64] = {0.0f};
static float x_weight_coordinate = 0.0f;
static float y_weight_coordinate = 0.0f;
static float xh_weight_coordinate = 0.0f;
static float yh_weight_coordinate = 0.0f;
// static int16_t z_total_heat = 0;
// static int16_t zscore_int16_t[64] = {0};
static int16_t z_max = 0;
static int16_t z_min = 0;
static bool zscoreWeight[64] = {false};

static float zscore_threshold_high = 10.0f;
static float zscore_threshold_low = 5.0f;
static float zscore_threshold_hot = 35.0f;
static float zscore_threshold_recal = 10.0f;
static bool zcal = true;


void gridEyeInit(void){
	DEBUG_PRINT("Grideye initing.\n");
	if(isInit)
		return;
	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = GRIDEYE_DEFAULT_ADDRESS;
	//reset flag
	i2cdevWriteByte(I2Cx, devAddr, 0x01, 0x30);
	//reset setting
	i2cdevWriteByte(I2Cx, devAddr, 0x01, 0x3f);

	xTaskCreate(gridEyeTask, GRIDEYE_TASK_NAME, GRIDEYE_TASK_STACKSIZE, NULL, GRIDEYE_TASK_PRI, NULL);

	isInit = true;
	DEBUG_PRINT("Grideye init [OK].\n");
}

bool gridEyeTest(void){

	  bool testStatus;
	  buffer[0] = 0x78;
	  buffer[1] = 0x88;
	  buffer[30] = 0X98;
	  if (!isInit)
	    return false;
	  DEBUG_PRINT("Grideye Testing.\n");

	  //i2cdevReadBits(I2Cx, devAddr, 0x07, 5, 16,buffer);

	  i2cdevRead(I2Cx, devAddr, 0x00,5,buffer );
//	  for(int i = 0; i < 30; i++){
//		  DEBUG_PRINT("bf[%d]:  %d.\n", i, buffer[i]);
//	  }
	  testStatus  = buffer[4] == 0x00 ;


	  DEBUG_PRINT("Grideye test interuption: %d .\n", buffer[4]);
	  return testStatus;
}

void gridEyeTask(void* arg){
	  systemWaitStart();
	  TickType_t xLastWakeTime;
//	  int addresses[] = {0x80, 0x10, 0x04}; // 0x80 down, 0x10 front, 0x04 top
//	  for (int i=0; i<3; i++){
//	    i2cdevWriteByte(I2Cx, TCAADDR, addresses[i], addresses[i]);
//	    vl53l0xSetVcselPulsePeriod(VcselPeriodPreRange, 18);
//	    vl53l0xSetVcselPulsePeriod(VcselPeriodFinalRange, 14);
//	    vl53l0xStartContinuous(0);
//	  }

	  while (1) {
	    xLastWakeTime = xTaskGetTickCount();


	    uint8_t data[128];
	    //i2cdevWriteByte(I2Cx, TCAADDR, 0x08, 0x08);
	    i2cdevRead(I2Cx, GRIDEYE_DEFAULT_ADDRESS, PIXEL0 | GRIDEYE_ADDR_AUTO_INC, 128, data);

	    if (!isCalibrate) {
	        for(int i=(64*bgFrameIndex);i<(64*(bgFrameIndex+1));i++){
	            int map_index = i - (64*bgFrameIndex);
	            bgData[i] = ((data[map_index*2+1] << 8) | data[map_index*2]) * 0.25f;
	        }

	        if (bgFrameIndex == BG_FRAME_NUM-1) {
	          calibration();
	          if (check_p2p()) {
	            isCalibrate = true;
	          } else {
	            bgFrameIndex = 0;
	          }
	        }
	        bgFrameIndex = (bgFrameIndex+1)%BG_FRAME_NUM;

	    } else if (!checkCalibrate) {
	        for(int i=0;i<64;i++){
	            rawData[i] = ((data[i*2+1] << 8) | data[i*2]) * 0.25f;
	        }
	        zscoreCalculation();

	        if (z_max > 800 || z_min < -800) { // ensure that the initial zscore is between 8 and -8.
	            isCalibrate = false;
	        } else {
	            checkCalibrate = true;
	        }

	    } else {
	        rawDataFrameIndex %= RAW_DATA_FRAME_NUM;
	        for(int i=(64*rawDataFrameIndex);i<(64*(rawDataFrameIndex+1));i++){
	            int map_index = i - (64*rawDataFrameIndex);
	            rawData[i] = ((data[map_index*2+1] << 8) | data[map_index*2]) * 0.25f;
	        }
	        zscoreCalculation();

	        uint8_t color[64]={0};
	        uint8_t rColor[64]={0};
	        int largestSubset[64] = {-1};
	        int maxSubsetLen = 0;
	        get_largest_subset(largestSubset, &maxSubsetLen);

	        reset_log_variable(color);

	        if (maxSubsetLen > 1) {
	            if (z_max > zscore_threshold_hot) {
	                get_filtered_xy(largestSubset, maxSubsetLen);
	            } else if (z_max > zscore_threshold_low) {
	                get_xy(largestSubset, maxSubsetLen);
	                // get_total_heat(largestSubset, maxSubsetLen);
	            }
	            rotateXY();
	            labelPixel(largestSubset, maxSubsetLen, color);
	            rotateColor(color, rColor);
	            findGroup(rColor);
	        } else {
	            clear_zscore_weight();
	            if (zcal && z_max < zscore_threshold_recal) {
	                copy_to_bgData();
	                fast_calibration();
	                bgFrameIndex = (bgFrameIndex+1)%BG_FRAME_NUM;
	            }
	        }

	        rawDataFrameIndex += 1;
	    }


	    vTaskDelayUntil(&xLastWakeTime, M2T(GRIDEYE_TASK_FREQ));
	  }

}



bool check_p2p() {
  for(int i=0; i<64; i++) {
    for(int j=0; j<BG_FRAME_NUM; j++) {
      if (abs(avgCal[i] - bgData[i+j*64]) > PEAK_THREHOLD) return false;
    }
  }
  return true;
}

void copy_to_bgData() {
  for(int i=0; i<64; i++) {
    bgData[i+64*bgFrameIndex] = rawData[i+64*rawDataFrameIndex];
  }
}

void fast_calibration() {
  int latest_p = 64*bgFrameIndex;
  int earliest_p = 64*(bgFrameIndex+1);
  if (bgFrameIndex == BG_FRAME_NUM-1) {
    earliest_p = 0;
  }
  for(int i=0; i<64; i++) {
    avgCal[i] = avgCal[i] - bgData[i+earliest_p]/BG_FRAME_NUM + bgData[i+latest_p]/BG_FRAME_NUM;
    squareAvg[i] = squareAvg[i] - powf(bgData[i+earliest_p], 2)/BG_FRAME_NUM + powf(bgData[i+latest_p], 2)/BG_FRAME_NUM;
  }
  for(int i=0; i<64; i++) {
    stdCal[i] = sqrtf(squareAvg[i]-powf(avgCal[i], 2));
  }
}

void calibration() {
  for(int i=0; i<64; i++) {
    float tempSum = 0;
    for(int j=0; j<BG_FRAME_NUM; j++) {
      tempSum += bgData[i+j*64];
    }
    avgCal[i] = tempSum / BG_FRAME_NUM;
  }

  for(int i=0; i<64; i++) {
    // float tempSum = 0;
    float tempSquareSum = 0;
    for(int j=0; j<BG_FRAME_NUM; j++) {
      // tempSum += powf(bgData[i+j*64]-avgCal[i], 2);
      tempSquareSum += powf(bgData[i+j*64],2)/BG_FRAME_NUM;
    }
    squareAvg[i] = tempSquareSum;
    stdCal[i] = sqrtf(squareAvg[i]-powf(avgCal[i], 2));
    // stdCal[i] = sqrtf(tempSum/ BG_FRAME_NUM);
  }
}

void zscoreCalculation() {
  float tempMax = -30.0f;
  float tempMin = 30.0f;
  for(int i=0; i<64; i++) {
    float tempSum = 0;
    for(int j=0; j<RAW_DATA_FRAME_NUM; j++) {
      tempSum += rawData[i+j*64];
    }
    float sampleAvg = tempSum / RAW_DATA_FRAME_NUM;
    int map_index = i + (64*rawDataFrameIndex);
    zscoreData[map_index] = (sampleAvg - avgCal[i]) / stdCal[i];

    tempSum = 0;
    for(int j=0; j<RAW_DATA_FRAME_NUM; j++) {
      tempSum += zscoreData[i+j*64];
    }
    zscore[i] = tempSum / RAW_DATA_FRAME_NUM;
    // zscore_int16_t[i] = (int)(zscore[i]*100);

    if(zscore[i] > tempMax) {
      tempMax = zscore[i];
    }

    if(zscore[i] < tempMin) {
      tempMin = zscore[i];
    }
  }
  z_max = (int)(tempMax*100);
  z_min = (int)(tempMin*100);
}

void rotateColor(uint8_t* color, uint8_t* rColor)
{
    int x_grid = (RIGHT_UP - LEFT_UP) / 7;
    int y_grid = (LEFT_DOWN - LEFT_UP) / 7;
    for(int i=0; i<8; i++) {
        for (int j=0; j<8; j++) {
            rColor[i*8+j] = color[LEFT_UP + x_grid*j + y_grid*i];
        }
    }
//	    for(int i=0; i<8; i++) {
//	        for (int j=0; j<8; j++) {
//	            rColor[i*8+j] = color[i*8+j];
//	        }
//	    }
}

void reset_log_variable(uint8_t* color)
{
    for(int i=0; i<64; i++) {
        color[i] = WHITE;
    }

    x_weight_coordinate = -1.0;
    y_weight_coordinate = -1.0;
    xh_weight_coordinate = -1.0;
    yh_weight_coordinate = -1.0;

    yellowGroupH = 0;
    yellowGroupL = 0;
    orangeGroupH = 0;
    orangeGroupL = 0;
    redGroupH = 0;
    redGroupL = 0;
}

void labelPixel(int* largestSubset, int maxSubsetLen, uint8_t* color)
{
    for(int i=0; i<maxSubsetLen; i++) {
        int pixelIndex = largestSubset[i];
        if (zscore[pixelIndex] > zscore_threshold_hot) {
            color[pixelIndex] = RED;
        } else if (zscore[pixelIndex] > zscore_threshold_high) {
            color[pixelIndex] = ORANGE;
        } else if (zscore[pixelIndex] > zscore_threshold_low) {
            color[pixelIndex] = YELLOW;
        }
    }
/*
    for(int i=0; i<64; i++){
        // if ((pixelTemp[i]-roomTemp) < THRESHOLD_LEVEL1){
        if (zscore[i] < 8.0f){
            color[i]=WHITE;
        // }else if ((pixelTemp[i]-roomTemp) < THRESHOLD_LEVEL2){
        }else if (zscore[i] < 10.0f){
            color[i]=YELLOW; // YELLOW
        }else{
            color[i]=ORANGE;
        }
    }
*/
    // if ((pixelTemp[maxPixelIndex]-roomTemp) < THRESHOLD_LEVEL1){
    //     color[maxPixelIndex]=WHITE;
    // }else{
    //     color[maxPixelIndex]=ORANGE;
    // }
}

void findGroup(uint8_t* color)
{
    for(int i=0; i<32; i++){
        uint8_t cl = color[i];
        if(cl == YELLOW){
            yellowGroupL |= 1<<i;
        }else if(cl == ORANGE){
            orangeGroupL |= 1<<i;
        }else if(cl == RED){
            redGroupL |= 1<<i;
        }
        cl = color[i+32];
        if(cl == YELLOW){
            yellowGroupH |= 1<<i;
        }else if(cl == ORANGE){
            orangeGroupH |= 1<<i;
        }else if(cl == RED){
            redGroupH |= 1<<i;
        }
    }
}


void get_eight_neighbor(int loc, int* neighbor) //neighbor length maximum is 8
{
        if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc+1; neighbor[1]=loc+ARRAY_SIZE; neighbor[2]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE-1; neighbor[3]=loc+ARRAY_SIZE; neighbor[4]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+ARRAY_SIZE-1; neighbor[2]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)> 0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-ARRAY_SIZE+1; neighbor[2]=loc+1; neighbor[3]=loc+ARRAY_SIZE; neighbor[4]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-ARRAY_SIZE+1; neighbor[3]=loc-1; neighbor[4]=loc+1; neighbor[5]=loc+ARRAY_SIZE-1; neighbor[6]=loc+ARRAY_SIZE; neighbor[7]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-1; neighbor[3]=loc+ARRAY_SIZE-1; neighbor[4]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-ARRAY_SIZE+1; neighbor[2]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-ARRAY_SIZE+1; neighbor[3]=loc-1; neighbor[4]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-1;
        }
}

void get_four_neighbor(int loc, int* neighbor) //neighbor length maximum is 4
{
        if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc+1; neighbor[1]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)> 0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+1; neighbor[3]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1;
        }
}

bool label_neighbor(int result[], int subsetNumber){
    bool hasNeighbor = false;
    for(int i=0; i<64; i++){
        if(result[i]==subsetNumber){
            int neighbor[8]={-1,-1,-1,-1,-1,-1,-1,-1};
            get_eight_neighbor(i, neighbor);
            for(int i=0; i<8;i++){
                if(neighbor[i] != -1 && result[neighbor[i]] == WAITTOCHECK){
                    result[neighbor[i]]=NEIGHBOR;
                    hasNeighbor = true;
                }
            }
        }
    }
    return hasNeighbor;
}

void label_subset(int testset[], int testsetLen, int result[], int subsetNumber){
    while(label_neighbor(result, subsetNumber)){
        for(int i=0; i< testsetLen; i++){
            if(result[testset[i]] == NEIGHBOR){
                result[testset[i]]=subsetNumber;
            }
        }
        for(int i=0; i<64; i++){
            if(result[i] == NEIGHBOR){
                result[i]=CHECKED;
            }
        }
    }
}

bool get_startIndex(int testset[], int testsetLen, int result[], int* startIndex){
    for(int i=0; i<testsetLen; i++){
        if(result[testset[i]] == WAITTOCHECK){
            *startIndex = i;
            return true;
        }
    }
    return false;
}

void select_largest_subset(int testset[], int testsetLen, int result[], int subsetNumber, int* maxSubsetLen, int* largestSubset){
    int maxSubsetNumber = 0;
    for(int i=0; i<subsetNumber; i++){
        int lengthCount = 0;
        for(int j=0; j<testsetLen; j++){
            if(result[testset[j]] == i){
                lengthCount++;
            }
        }
        if(lengthCount > *maxSubsetLen){ // if equal, no solution currently
            *maxSubsetLen = lengthCount;
            maxSubsetNumber = i;
        }
    }

    // largestSubset = (int*)malloc(sizeof(int)*(*maxSubsetLen));
    int index=0;
    for(int i=0; i<64; i++){
        if(result[i]==maxSubsetNumber){
            largestSubset[index]=i;
            index++;
        }
    }
}

void find_largestSubset(int testset[], int testsetLen, int* maxSubsetLen, int* largestSubset){
    int result[64];
    for(int i=0; i<64;i++){result[i]=WAITTOCHECK;}
    int subsetNumber = 0;
    int startIndex = 0;
    while(get_startIndex(testset, testsetLen, result, &startIndex)){
        result[testset[startIndex]]=subsetNumber;
        label_subset(testset, testsetLen, result, subsetNumber);
        subsetNumber++;
    }
    select_largest_subset(testset, testsetLen, result, subsetNumber, maxSubsetLen, largestSubset);
}

void clear_zscore_weight() {
    for(int i=0; i<64; i++){
        zscoreWeight[i]=false;
    }
}

bool check_neighbor_zscore_weight(int index) {
    int neighbor[4]={-1,-1,-1,-1};
    get_four_neighbor(index, neighbor);
    for(int i=0; i<4;i++){
        if (neighbor[i] != -1) {
            if (zscoreWeight[neighbor[i]]) {
                return true;
            }
        }
    }
    return false;
}

void get_largest_subset(int* largestSubset, int* maxSubsetLen){
    int testset[64]={0};
    int pixelCount=0;
    for(int i=0; i<64; i++){
        if (zscore[i] > zscore_threshold_high) {
            testset[pixelCount]=i;
            pixelCount++;
            zscoreWeight[i]=true;
        } else if (zscore[i] > zscore_threshold_low) {
            if (zscoreWeight[i] || check_neighbor_zscore_weight(i)) {
                testset[pixelCount]=i;
                pixelCount++;
                zscoreWeight[i]=true;
            }
        } else {
            zscoreWeight[i]=false;
        }
    }
    testset[pixelCount] = BOUNDARY;

    find_largestSubset(testset, pixelCount, maxSubsetLen, largestSubset);

    // If subset only has one pixel higher than zscore threshold, it will be consideredd as a noise.
    // This subset will be reset here.
    if (*maxSubsetLen == 1) {
      *maxSubsetLen = 0;
      largestSubset[0] = -1; // reset
    }
    // // calculate_height(maxSubsetLen);
    // // calculate_xy(maxSubsetLen, largestSubset);
    // count = (count+1) % 10;
    // return true;
}

 void rotateXY() {
//     int x_grid = (RIGHT_UP - LEFT_UP) / 7;
//     int y_grid = (LEFT_DOWN - LEFT_UP) / 7;
//     float r_pixelIndex = LEFT_UP + (x_grid * x_weight_coordinate) + (y_grid * y_weight_coordinate);
//     x_weight_coordinate = fmod(r_pixelIndex, 8);
//     y_weight_coordinate = r_pixelIndex / 8;
//
//     float rh_pixelIndex = LEFT_UP + (x_grid * xh_weight_coordinate) + (y_grid * yh_weight_coordinate);
//	 xh_weight_coordinate = fmod(rh_pixelIndex, 8);
//	 yh_weight_coordinate = rh_pixelIndex / 8;
     int x_grid = -8;
     int y_grid = 1;
//     float r_pixelIndex = 56 + (x_grid * x_weight_coordinate) + (y_grid * y_weight_coordinate);
     float rx_pixelIndex = 56 + (y_grid * y_weight_coordinate);
     float ry_pixelIndex = 56 + (x_grid * x_weight_coordinate);

     x_weight_coordinate = fmod(rx_pixelIndex, 8);
     y_weight_coordinate = ry_pixelIndex / 8;

//     float rh_pixelIndex = 56 + (x_grid * xh_weight_coordinate) + (y_grid * yh_weight_coordinate);
     float rhx_pixelIndex = 56 + (y_grid * yh_weight_coordinate);
     float rhy_pixelIndex = 56 + (x_grid * xh_weight_coordinate);

     if(xh_weight_coordinate >= 0){
    	 xh_weight_coordinate = fmod(rhx_pixelIndex, 8);
     }
//     xh_weight_coordinate = fmod(rhx_pixelIndex, 8) >= 0 ? fmod(rhx_pixelIndex, 8) : xh_weight_coordinate;
//     xh_weight_coordinate = fmod(rhx_pixelIndex, 8) ;
     yh_weight_coordinate = yh_weight_coordinate >= 0 ? rhy_pixelIndex / 8 : yh_weight_coordinate;
 }

void get_filtered_xy(int* largestSubset, int maxSubsetLen) {

    float x_weight_zscore_sum = 0.0f;
    float y_weight_zscore_sum = 0.0f;
    float zscore_sum = 0.0f;
    uint8_t low_zscore_length = 0;

    float xh_weight_zscore_sum = 0.0f;
    float yh_weight_zscore_sum = 0.0f;
    float zscore_sum_h = 0.0f;
    uint8_t hot_zscore_length = 0;

    for(int i=0; i<maxSubsetLen; i++) {
        float _zscore = zscore[largestSubset[i]];
        if (_zscore > zscore_threshold_hot) {
            zscore_sum_h += _zscore;
            hot_zscore_length += 1;
        } else if (_zscore > zscore_threshold_low) {
            zscore_sum += _zscore;
            low_zscore_length += 1;
        }
    }

    for(int i=0; i<maxSubsetLen; i++) {
        int _x = largestSubset[i] % 8;
        int _y = largestSubset[i] / 8;
        float _zscore = zscore[largestSubset[i]];
        if (_zscore > zscore_threshold_hot) {
            xh_weight_zscore_sum += _x * _zscore / zscore_sum_h;
            yh_weight_zscore_sum += _y * _zscore / zscore_sum_h;
        } else if (_zscore > zscore_threshold_low) {
            x_weight_zscore_sum += _x * _zscore / zscore_sum;
            y_weight_zscore_sum += _y * _zscore / zscore_sum;
        }
    }

    if (hot_zscore_length > 0) {
        xh_weight_coordinate = xh_weight_zscore_sum;
        yh_weight_coordinate = yh_weight_zscore_sum;
    }

    if (low_zscore_length > 0) {
        x_weight_coordinate = x_weight_zscore_sum;
        y_weight_coordinate = y_weight_zscore_sum;
    }
}

void get_xy(int* largestSubset, int maxSubsetLen) {

    float x_weight_zscore_sum = 0.0f;
    float y_weight_zscore_sum = 0.0f;
    float zscore_sum = 0.0f;

    for(int i=0; i<maxSubsetLen; i++) {
        float _zscore = zscore[largestSubset[i]];
        zscore_sum += _zscore;
    }

    for(int i=0; i<maxSubsetLen; i++) {
        int _x = largestSubset[i] % 8;
        int _y = largestSubset[i] / 8;
        float _zscore = zscore[largestSubset[i]];
        x_weight_zscore_sum += _x * _zscore / zscore_sum;
        y_weight_zscore_sum += _y * _zscore / zscore_sum;
    }
    x_weight_coordinate = x_weight_zscore_sum;
    y_weight_coordinate = y_weight_zscore_sum;
}

PARAM_GROUP_START(zscore)
PARAM_ADD(PARAM_FLOAT, thre_high, &zscore_threshold_high)
PARAM_ADD(PARAM_FLOAT, thre_low, &zscore_threshold_low)
PARAM_ADD(PARAM_FLOAT, thre_hot, &zscore_threshold_hot)
PARAM_ADD(PARAM_UINT8, zcal, &zcal)
PARAM_ADD(PARAM_UINT8, iscal, &isCalibrate)
PARAM_ADD(PARAM_UINT8, checkcal, &checkCalibrate)
PARAM_GROUP_STOP(zscore)

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, zrange, &range_last)
LOG_ADD(LOG_UINT16, range_down, &range_last_down)
LOG_ADD(LOG_UINT16, range_front_l, &range_last_front_l)
LOG_ADD(LOG_UINT16, range_front_r, &range_last_front_r)
// LOG_ADD(LOG_UINT16, range_top, &range_last_top)
LOG_GROUP_STOP(range)

LOG_GROUP_START(gridEye)
// LOG_ADD(LOG_FLOAT, roomTemp, &roomTemp)
LOG_ADD(LOG_UINT32, yellowGroupH, &yellowGroupH)
LOG_ADD(LOG_UINT32, yellowGroupL, &yellowGroupL)
LOG_ADD(LOG_UINT32, orangeGroupH, &orangeGroupH)
LOG_ADD(LOG_UINT32, orangeGroupL, &orangeGroupL)
LOG_ADD(LOG_UINT32, redGroupH, &redGroupH)
LOG_ADD(LOG_UINT32, redGroupL, &redGroupL)
LOG_GROUP_STOP(gridEye)

LOG_GROUP_START(gridEyeXYZ)
LOG_ADD(LOG_FLOAT, xw, &x_weight_coordinate)
LOG_ADD(LOG_FLOAT, yw, &y_weight_coordinate)
LOG_ADD(LOG_FLOAT, xh, &xh_weight_coordinate)
LOG_ADD(LOG_FLOAT, yh, &yh_weight_coordinate)
// LOG_ADD(LOG_INT16, zh, &z_total_heat)
LOG_ADD(LOG_INT16, zMax, &z_max)
LOG_ADD(LOG_INT16, zMin, &z_min)
LOG_GROUP_STOP(gridEyeXYZ)



// char groupName[10] = "gridEyeR";
// char varName[10] = "rawDataC";
// for(int i=0;i<1;i++)
// {
//   groupName[8] = (char)(i+48);
//   LOG_GROUP_START(groupName)
//   for(int j=0;j<8;j++)
//   {
//     varName[8] = (char)(j+48);
//     LOG_ADD(LOG_UINT16, varName, &rawData[i*8+j]);
//   }
//   LOG_GROUP_STOP(groupName)
// }

// LOG_GROUP_START(gridEyeR0)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[0])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[1])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[2])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[3])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[4])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[5])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[6])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[7])
// LOG_GROUP_STOP(gridEyeR0)

// LOG_GROUP_START(gridEyeR1)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[8])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[9])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[10])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[11])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[12])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[13])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[14])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[15])
// LOG_GROUP_STOP(gridEyeR1)

// LOG_GROUP_START(gridEyeR2)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[16])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[17])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[18])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[19])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[20])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[21])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[22])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[23])
// LOG_GROUP_STOP(gridEyeR2)

// LOG_GROUP_START(gridEyeR3)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[24])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[25])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[26])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[27])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[28])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[29])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[30])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[31])
// LOG_GROUP_STOP(gridEyeR3)

// LOG_GROUP_START(gridEyeR4)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[32])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[33])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[34])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[35])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[36])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[37])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[38])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[39])
// LOG_GROUP_STOP(gridEyeR4)

// LOG_GROUP_START(gridEyeR5)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[40])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[41])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[42])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[43])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[44])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[45])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[46])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[47])
// LOG_GROUP_STOP(gridEyeR5)

// LOG_GROUP_START(gridEyeR6)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[48])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[49])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[50])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[51])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[52])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[53])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[54])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[55])
// LOG_GROUP_STOP(gridEyeR6)

// LOG_GROUP_START(gridEyeR7)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[56])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[57])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[58])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[59])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[60])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[61])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[62])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[63])
// LOG_GROUP_STOP(gridEyeR7)




