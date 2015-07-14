/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl
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
 * @file modules/sensors/dist_imu_mpu6050.c
 */

#include "dist_imu_mpu6050.h"

#include <math.h>

// #include "led.h"
#include "messages.h"
#include "mcu_periph/i2c.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/imu/imu_mpu60x0_defaults.h"

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

// MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU60X0_LOWPASS_FILTER && !defined  IMU_MPU60X0_SMPLRT_DIV
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU60X0_SMPLRT_DIV 3
#endif

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef IMU_MPU60X0_ACCEL_RANGE
#define IMU_MPU60X0_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

#ifndef IMU_MPU60X0_I2C_ADDR
#define IMU_MPU60X0_I2C_ADDR MPU60X0_ADDR
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif

#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif

struct DistImu dist_imu;

void dist_imu_init(void)
{
  mpu60x0_i2c_init(&dist_imu.mpu[0], &(IMU_MPU60X0_I2C_DEV), IMU_MPU60X0_I2C_ADDR);
  // Change the default configuration
  dist_imu.mpu[0].config.smplrt_div = IMU_MPU60X0_SMPLRT_DIV;
  dist_imu.mpu[0].config.dlpf_cfg = IMU_MPU60X0_LOWPASS_FILTER;
  dist_imu.mpu[0].config.gyro_range = IMU_MPU60X0_GYRO_RANGE;
  dist_imu.mpu[0].config.accel_range = IMU_MPU60X0_ACCEL_RANGE;

  INT_VECT3_ZERO(dist_imu.accel_neutral[0]);
  INT_RATES_ZERO(dist_imu.gyro_neutral[0]);
}

void dist_imu_periodic(void)
{
  mpu60x0_i2c_periodic(&dist_imu.mpu[0]);
}

void dist_imu_event(void)
{
  // uint32_t now_ts = get_sys_time_usec();

  mpu60x0_i2c_event(&dist_imu.mpu[0]);
  if (dist_imu.mpu[0].data_available) {
    RATES_COPY(dist_imu.gyro_unscaled[0], dist_imu.mpu[0].data_rates.rates);
    VECT3_COPY(dist_imu.accel_unscaled[0], dist_imu.mpu[0].data_accel.vect);
    dist_imu.mpu[0].data_available = FALSE;

    dist_imu.gyro[0].p = ((dist_imu.gyro_unscaled[0].p - dist_imu.gyro_neutral[0].p) * IMU_GYRO_P_SIGN *
                      IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN;
    dist_imu.gyro[0].q = ((dist_imu.gyro_unscaled[0].q - dist_imu.gyro_neutral[0].q) * IMU_GYRO_Q_SIGN *
                      IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN;
    dist_imu.gyro[0].r = ((dist_imu.gyro_unscaled[0].r - dist_imu.gyro_neutral[0].r) * IMU_GYRO_R_SIGN *
                      IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN;

    dist_imu.accel[0].x = ((dist_imu.accel_unscaled[0].x - dist_imu.accel_neutral[0].x) * IMU_ACCEL_X_SIGN *
                       IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN;
    dist_imu.accel[0].y = ((dist_imu.accel_unscaled[0].y - dist_imu.accel_neutral[0].y) * IMU_ACCEL_Y_SIGN *
                       IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN;
    dist_imu.accel[0].z = ((dist_imu.accel_unscaled[0].z - dist_imu.accel_neutral[0].z) * IMU_ACCEL_Z_SIGN *
                       IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN;

    RunOnceEvery(10, { 
      DOWNLINK_SEND_DIST_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
                                         &dist_imu.gyro[0].p,
                                         &dist_imu.gyro[0].q,
                                         &dist_imu.gyro[0].r);
    });
    
    RunOnceEvery(10, { 
      DOWNLINK_SEND_DIST_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
                                         &dist_imu.accel[0].x,
                                         &dist_imu.accel[0].y,
                                         &dist_imu.accel[0].z);
    });
  }
}
