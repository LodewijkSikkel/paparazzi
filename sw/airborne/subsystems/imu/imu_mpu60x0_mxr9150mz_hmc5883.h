/*
 * Copyright (C) 2015 <l.n.c.sikkel@tudelft.nl>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_mpu60x0_memsicR9150M.h
 * Driver for IMU with MPU60x0 gyros via SPI, MXR9150MZ via 3 
 * separate ADCs and HMC5883 via I2C
 */

#ifndef IMU_MPU60x0_MXR9150MZ_HMC5883_H
#define IMU_MPU60x0_MXR9150MZ_HMC5883_H

#define IMU_ACCEL_X_SENS_NUM 1
#define IMU_ACCEL_X_SENS_DEN 1
#define IMU_ACCEL_Y_SENS_NUM 1
#define IMU_ACCEL_Y_SENS_DEN 1
#define IMU_ACCEL_Z_SENS_NUM 1
#define IMU_ACCEL_Z_SENS_DEN 1

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "subsystems/imu/imu_mpu60x0_defaults.h"
#include "peripherals/mpu60x0_spi.h"
#include "peripherals/hmc58xx.h"

struct ImuMpu60x0Mxr9150mzHmc5883 {
  struct Mpu60x0_Spi mpu;
  struct Mxr9150mz mxr;
  struct Hmc58xx hmc;
  int imu_overrun;
};

extern struct ImuMpu60x0Mxr9150mzHmc5883 imu_mpu_mxr_hmc;

extern void imu_mpu_mxr_hmc_event(void);

#define ImuEvent imu_mpu_mxr_hmc_event

#endif /* IMU_MPU60x0_MXR9150MZ_HMC5883_H */
