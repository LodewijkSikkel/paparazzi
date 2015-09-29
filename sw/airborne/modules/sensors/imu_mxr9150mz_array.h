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

#ifndef IMU_MXR9150MZ_ARRAY_H
#define IMU_MXR9150MZ_ARRAY_H

#include "std.h"

#include "peripherals/mxr9150mz.h"
#include "subsystems/imu.h"

struct ImuMxr9150mzArray {
  struct Mxr9150mz mxr[3];
  struct Imu imu[3];
  struct FloatVect3 accel; // linear acceleration components
  struct FloatRates rates_sqr; // angular rates squared
  struct FloatRates	rates_dot; // angular acceleration
  struct FloatRates rates_cross; // angular rate cross components
};

extern struct ImuMxr9150mzArray imu_mxr_array;

extern void imu_mxr_array_init(void);
extern void imu_mxr_array_periodic(void);
extern void imu_mxr_array_event(void);

#endif /* IMU_MXR9150MZ_ARRAY_H */
