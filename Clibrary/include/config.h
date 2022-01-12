/*
 * config.h
 *
 *  Created on: 2021年7月12日
 *      Author: JackyPan
 */

#pragma once

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#include "hal.h"

/**** 数字舵机PWM频率为320HZ ****
 **** 模拟舵机PWM频率为50HZ  ****
 **** 电机PWM频率为400HZ    ****/
#define PWM_MOTOR_FREQ 400
#define PWM_DIGITAL_SERVO_FREQ 320
#define PWM_ANALOG_SERVO_FREQ 50

#define PWM_SERVO_MIN 500   // 脉宽 500us
#define PWM_SERVO_MAX 2500  // 脉宽 2500us

#define PWM_ESC_MIN 1000	// 脉宽 1000us
#define PWM_ESC_MAX 2000	// 脉宽 2000us
#define PWM_ESC_SPIN_ARM 1100
#define PWM_ESC_SPIN_MIN 1200
#define PWM_ESC_SPIN_MAX 1850

#define PWM_BRUSH_MIN 0		// 脉宽 0us
#define PWM_BRUSH_MAX 2500	// 脉宽 2500us
#define PWM_BRUSH_SPIN_ARM 50
#define PWM_BRUSH_SPIN_MIN 100
#define PWM_BRUSH_SPIN_MAX 2400

/**************遥控信号输入**************/
#define RC_INPUT_PPM	0   		//PPM信号
#define RC_INPUT_SBUS	1  			//SBUS信号
#define RC_INPUT_CHANNELS  	8       //8 RC input channel (value:1000~2000)

/*************遥控信号配置***************/
#define RC_INPUT RC_INPUT_SBUS //配置为PPM模式

//遥控器输入脉宽范围
#define RC_INPUT_MIN 1100.0f // 脉宽 1100us
#define RC_INPUT_MAX 1900.0f // 脉宽 1900us

/*****************COMM*****************
 * *************COMM0:USB**************
 * *************COMM1:Serial1**********
 * *************COMM2:Serial2**********
 * *************COMM3:Serial3**********
 * *************COMM4:Serial4**********/
#define DEV_COMM 	0x01 //自定义模式
#define MAV_COMM  	0x02 //Mavlink模式
#define GPS_COMM  	0x03 //GPS模式
#define TFMINI_COMM 0X04 //TFmini激光测距仪

/*************配置COMM*************/
#define COMM_0 MAV_COMM
#define COMM_1 GPS_COMM
#define COMM_2 MAV_COMM
#define COMM_3 TFMINI_COMM
#define COMM_4 MAV_COMM

//配置GPS
#define USE_GPS 0 // if use gps, set 1; if not use gps, set 0;

//配置UWB
#define USE_UWB 1 // if use uwb, set 1; if don't use uwb, set 0;

//配置flash
/* ***********************************
 * if use FM25VXX(赛普拉斯)-FRAM, set 1;
 * if use FM25XXX(复旦微)-EEPROM, set 2;
 * if not use fram(用内置flash), set 0;
 * ***********************************/
#define USE_FRAM 2

#if USE_FRAM==1
	#define ADDR_FLASH        ((uint16_t)0x0000)
	/* FM25V02->32KB  | 0x0000 - 0x7FFF ,
	 * 1地址表示1byte*/
	/* the first 2 bytes must be the number of all the package,
	 * so we can get how many packages were saved in the flash by reading the first 2 bytes*/
	#define ADDR_FLASH_DATA   ((uint16_t)0x0002) /* the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint16_t)0x0020) /* each data package takes 32 bytes */
#elif USE_FRAM==2
	#define ADDR_FLASH        ((uint16_t)0x0000)
	/* FM25128->16KB  | 0x0000 - 0x3FFF , 256个page
	 * 每个page有64个byte*/
	/* the first 2 bytes must be the number of all the package,
	 * so we can get how many packages were saved in the flash by reading the first 2 bytes*/
	#define ADDR_FLASH_DATA   ((uint16_t)0x0020) /* the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint16_t)0x0020) /* each data package takes 32 bytes */
#else
	#define ADDR_FLASH        ((uint32_t)0x081D0000) /* bank2, Sector22: 128KB | 0x081D0000 - 0x081EFFFF, 1地址表示1byte */
	/* the Sector22 must be the number of all data package,
	 * so we can get how many packages were saved in the flash by reading Sector22*/
	#define ADDR_FLASH_DATA   ((uint32_t)0x081E0000) /* bank2, Sector23: 128KB | 0x081E0000 - 0x081FFFFF, the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint32_t)0x00000020) /* each data package takes 32 bytes */
#endif

//配置WIFI
#define MLINK_VIDEO 1
#define MLINK_MINI	2

#define MLINK MLINK_MINI

#endif /* INCLUDE_CONFIG_H_ */
