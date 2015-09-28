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
#include "subsystems/imu.h"

#ifndef DIST_IMU_NB_IMU // Number of IMUs
#define DIST_IMU_NB_IMU 0
#endif

#ifndef DIST_IMU_FIR_CUTOFF_FREQ // Cut-off frequency
#define DIST_IMU_FIR_CUTOFF_FREQ 256
#endif

#ifndef DIST_IMU_FIR_HALF_WINDOW_SIZE // One-sided buffer window
#define DIST_IMU_FIR_HALF_WINDOW_SIZE 1
#endif

struct CircBuf {
  struct FloatVect3 buffer[(2 * DIST_IMU_FIR_HALF_WINDOW_SIZE) + 1]; // input-output buffer
  size_t size; 
  uint32_t head;
  bool data_available;
};

struct DistImu {
  struct Mpu60x0_I2c mpu[DIST_IMU_NB_IMU];
  struct Imu imu[DIST_IMU_NB_IMU];
  struct CircBuf circ_buf[DIST_IMU_NB_IMU]; // circular input-output buffers
  float fir_lp_coef[(2 * DIST_IMU_FIR_HALF_WINDOW_SIZE)+1]; // FIR low-pass filter coefficients
  struct FloatVect3 accel; // linear acceleration components
  struct FloatRates rates_sqr; // angular rates squared
  struct FloatRates	rates_dot; // angular acceleration
  struct FloatRates rates_cross; // angular rate cross components
};

extern struct DistImu dist_imu;

extern void dist_imu_init(void);
extern void dist_imu_periodic(void);
extern void dist_imu_event(void);

#endif /* DIST_IMU_MPU60X0_H */
