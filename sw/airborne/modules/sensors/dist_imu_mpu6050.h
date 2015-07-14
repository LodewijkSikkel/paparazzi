/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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
 * @file modules/sensors/dist_imu_mpu6050.h
 * @brief Module for a multi-axis Inertial Measurement Unit (IMU)  
 */

#ifndef DIST_IMU_MPU60X0_H
#define DIST_IMU_MPU60X0_H

#include "std.h"

#include "peripherals/mpu60x0_i2c.h"

struct DistImu {
  struct Mpu60x0_I2c mpu[1];
  struct Int32Rates gyro[1]; // gyroscope measurements in rad/s in BFP with #INT32_RATE_FRAC
  struct Int32Vect3 accel[1]; // accelerometer measurements in m/s^2 in BFP with #INT32_ACCEL_FRAC
  struct Int32Rates gyro_unscaled[1]; // unscaled gyroscope measurements
  struct Int32Vect3 accel_unscaled[1]; // unscaled accelerometer measurements
  struct Int32Rates gyro_neutral[1]; // gyroscope bias
  struct Int32Vect3 accel_neutral[1]; // accelerometer bias
};

extern struct DistImu dist_imu;

extern void dist_imu_init(void);
extern void dist_imu_periodic(void);
extern void dist_imu_event(void);

#endif /* DIST_IMU_MPU60X0_H */
