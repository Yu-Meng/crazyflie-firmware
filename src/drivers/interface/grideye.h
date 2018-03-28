/*
 * grideye.h
 *
 *  Created on: Jan 1, 2018
 *      Author: Yu Meng
 */

#ifndef _GRIDEYE_H_
#define _GRIDEYE_H_

#include "stabilizer_types.h"
#include "deck_core.h"

#include "i2cdev.h"

//#define TCAADDR 0x70

#define GRIDEYE_DEFAULT_ADDRESS 0b1101000

#define GRIDEYE_TASK_FREQ 100

#define GRIDEYE_ADDR_AUTO_INC 0x80

#define PIXEL0                0x00

#define GRIDEYE_ADDR_BG_TEMP  0x0E

#define THRESHOLD_LEVEL1      2
#define THRESHOLD_LEVEL2      3
#define RED                   4
#define ORANGE                3
#define YELLOW                2
#define WHITE                 1

#define ARRAY_SIZE       8
#define WAITTOCHECK     90
#define CHECKED         91
#define NEIGHBOR        92
#define BOUNDARY        99


void gridEyeInit(void);
bool gridEyeTest(void);
void gridEyeTask(void* arg);


void reset_log_variable(uint8_t* color);
void clear_zscore_weight();
bool check_neighbor_zscore_weight(int index);
void rotateColor(uint8_t* color, uint8_t* rColor);
void labelPixel(int* largestSubset ,int maxSubsetLen, uint8_t* color);
void findGroup(uint8_t* color);
bool check_p2p();
void copy_to_bgData();
void fast_calibration();
void calibration();
void zscoreCalculation();
void get_four_neighbor(int loc, int* neighbor);
void get_eight_neighbor(int loc, int* neighbor);
bool label_neighbor(int result[], int subsetNumber);
void label_subset(int testset[], int testsetLen, int result[], int subsetNumber);
bool get_startIndex(int testset[], int testsetLen, int result[], int* startIndex);
void select_largest_subset(int testset[], int testsetLen, int result[], int subsetNumber, int* maxSubsetLen, int* largestSubset);
void find_largestSubset(int testset[], int testsetLen, int* maxSubsetLen, int* largestSubset);
void get_largest_subset(int* largestSubset, int* maxSubsetLen);
void rotateXY();
void get_filtered_xy(int* largestSubset, int maxSubsetLen);
void get_xy(int* largestSubset, int maxSubsetLen);
// void get_total_heat(int* largestSubset, int maxSubsetLen);

#endif /* _GRIDEYE_H_ */
