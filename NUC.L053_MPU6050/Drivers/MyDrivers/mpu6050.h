/*
 * mpu6050.h
 *
 *  Created on: Jul 18, 2023
 *      Author: yakup
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#endif /* INC_MPU6050_H_ */
#include "main.h"
#include "i2c.h"
#include "string.h"
#include "stdio.h"

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75



void MPU6050_Init (void);
void MPU6050_Read_Accel (void);
void MPU6050_Read_Gyro (void);
