/*
 * uwb.h
 *
 *  Created on: Nov 20, 2021
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_UWB_UWB_H_
#define INCLUDE_UWB_UWB_H_

#include "hal.h"
#include "uwb/deca_device_api.h"
#include "uwb/deca_regs.h"
#include "uwb/trilateration.h"

#define TAG_ID 0x01
#define MASTER_TAG 0x00
#define MAX_SLAVE_TAG 45
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 4
#define ANCHOR_IND 2  // 0 1 2 3

typedef enum{
	tag=1,
	anchor
}uwb_mode;

void uwb_init(uwb_mode uwb_mode);
void uwb_update(uwb_mode uwb_mode);

#endif /* INCLUDE_UWB_UWB_H_ */
