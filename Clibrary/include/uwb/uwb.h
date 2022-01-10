/*
 * uwb.h
 *
 *  Created on: Nov 20, 2021
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_UWB_UWB_H_
#define INCLUDE_UWB_UWB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"
#include "uwb/deca_device_api.h"
#include "uwb/deca_regs.h"
#include "uwb/trilateration.h"

#define TAG_ID 0xFE
#define MASTER_TAG 0xFE
#define MAX_SLAVE_TAG 0x01
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 4 //3 4
#define ANCHOR_IND 1  // 1 2 3 4

typedef enum{
	tag=1,
	anchor
}uwb_mode;

typedef enum{
	idle=0,
	receive,
	poll,
	resp,
	final,
	dis,
	release,
	release_confirm,
	release_wait,
	statistics
}uwb_states;

bool uwb_init(uwb_mode uwb_mode);
void uwb_update(uwb_mode uwb_mode);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_UWB_UWB_H_ */
