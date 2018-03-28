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
 * zranger.h: Z-Ranger deck driver
 */

#ifndef _ZRANGER_H_
#define _ZRANGER_H_

#include "stabilizer_types.h"
#include "deck_core.h"

void zRangerInit(DeckInfo* info);

bool zRangerTest(void);
void zRangerTask(void* arg);

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick);

#endif /* _ZRANGER_H_ */

/*
 * Head for AMG8833 Grid-eye
 * by Yumeng
 */

//#define GRIDEYE_DEFAULT_ADDRESS 0b1101000
//
//#define GRIDEYE_TASK_FREQ 100
//
//#define GRIDEYE_ADDR_AUTO_INC 0x80
//
//#define PIXEL0                0x00
//
//#define GRIDEYE_ADDR_BG_TEMP  0x0E
//
//#define THRESHOLD_LEVEL1      2
//#define THRESHOLD_LEVEL2      3
//#define RED                   4
//#define ORANGE                3
//#define YELLOW                2
//#define WHITE                 1
//
//#define ARRAY_SIZE       8
//#define WAITTOCHECK     90
//#define CHECKED         91
//#define NEIGHBOR        92
//#define BOUNDARY        99





