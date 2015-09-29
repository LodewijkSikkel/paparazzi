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
 * @file modules/sensors/imu_mxr9150mz_array.c
 * The separate mxr9150mz IMUs are are aligned to an arbitrary common body frame,
 * the ADCs are polled at 512Hz and if data is available it is processed in the
 * event loop.
 */

// Include own header
#include "imu_mxr9150mz_array.h"

#include <math.h>

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_orientation_conversion.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#if define ACCEL_1_CHANNEL_X & defined ACCEL_1_CHANNEL_Y & defined ACCEL_1_CHANNEL_X
#define USE_ACCEL_1 1
#endif

#if define ACCEL_2_CHANNEL_X & defined ACCEL_2_CHANNEL_Y & defined ACCEL_2_CHANNEL_X
#define USE_ACCEL_2 1
#endif

#if define ACCEL_3_CHANNEL_X & defined ACCEL_3_CHANNEL_Y & defined ACCEL_3_CHANNEL_X
#define USE_ACCEL_3 1
#endif

#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
 
struct DistImu imu_mxr_array;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

struct ImuMxr9150mzArray imu_mxr_array;

static void send_accel_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ARRAY_ACCEL_SCALED(trans, dev, AC_ID,
                                       &imu_mxr_array.imu[0].accel.x,
                                       &imu_mxr_array.imu[0].accel.y,
                                       &imu_mxr_array.imu[0].accel.z,
                                       &imu_mxr_array.imu[1].accel.x,
                                       &imu_mxr_array.imu[1].accel.y,
                                       &imu_mxr_array.imu[1].accel.z,
                                       &imu_mxr_array.imu[2].accel.x,
                                       &imu_mxr_array.imu[2].accel.y,
                                       &imu_mxr_array.imu[2].accel.z);
}

static void send_accel(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ARRAY_ACCEL(trans, dev, AC_ID,
                                &imu_mxr_array.accel.x,
                                &imu_mxr_array.accel.y,
                                &imu_mxr_array.accel.z);
}

static void send_rates(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ARRAY_RATES(trans, dev, AC_ID,
                                &imu_mxr_array.rates_sqr.p,
                                &imu_mxr_array.rates_sqr.q,
                                &imu_mxr_array.rates_sqr.r,
                                &imu_mxr_array.rates_dot.p,
                                &imu_mxr_array.rates_dot.q,
                                &imu_mxr_array.rates_dot.r);
}

#endif

void imu_mxr_array_init(void)
{

struct FloatEulers orientation_eulers;

#if USE_ACCEL_1
  mxr9150mz_init(&mxr[0], ACCEL_1_CHANNEL_X, ACCEL_1_CHANNEL_Y, ACCEL_1_CHANNEL_Z);

  orientation_eulers = {ACCEL_1_BODY_TO_IMU_X, ACCEL_1_BODY_TO_IMU_Y, ACCEL_1_BODY_TO_IMU_Z};
  
  orientationSetEulers_f(&imu_mxr_array.imu[0].body_to_imu, &orientation_eulers);
  
  orientationCalcRMat_i(&imu_mxr_array.imu[0].body_to_imu);

  INT_VECT3_ZERO(imu_mxr_array.imu[0].accel_neutral);
#endif

#if USE_ACCEL_2
  mxr9150mz_init(&mxr[1], ACCEL_2_CHANNEL_X, ACCEL_2_CHANNEL_Y, ACCEL_2_CHANNEL_Z);

  orientation_eulers = {ACCEL_2_BODY_TO_IMU_X, ACCEL_2_BODY_TO_IMU_Y, ACCEL_2_BODY_TO_IMU_Z};
  
  orientationSetEulers_f(&imu_mxr_array.imu[1].body_to_imu, &orientation_eulers);
  
  orientationCalcRMat_i(&imu_mxr_array.imu[1].body_to_imu);

  INT_VECT3_ZERO(imu_mxr_array.imu[1].accel_neutral);
#endif 

#if USE_ACCEL_3
  mxr9150mz_init(&mxr[2], ACCEL_3_CHANNEL_X, ACCEL_3_CHANNEL_Y, ACCEL_3_CHANNEL_Z);

  orientation_eulers = {ACCEL_3_BODY_TO_IMU_X, ACCEL_3_BODY_TO_IMU_Y, ACCEL_3_BODY_TO_IMU_Z};
  
  orientationSetEulers_f(&imu_mxr_array.imu[2].body_to_imu, &orientation_eulers);
  
  orientationCalcRMat_i(&imu_mxr_array.imu[2].body_to_imu);

  INT_VECT3_ZERO(imu_mxr_array.imu[2].accel_neutral);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_ACCEL_SCALED", send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_ACCEL", send_accel);
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_RATES", send_rates);
#endif // DOWNLINK

}

void imu_mxr_array_periodic(void)
{
  for (int i = 0; i < 3; i++) {
    mxr9150mz_periodic(&imu_mxr_array.mxr[i]);
  }
}

void imu_mxr_array_event(void)
{
  for (int i = 0; i < 3; i++) {
    if (imu_mxr_array.mxr[i].data_available) {
      // Copy the data from the device to the IMU struct
      VECT3_COPY(imu_mxr_array.imu[i].accel_unscaled, imu_mxr_array.mxr[i].data_accel.vect);
      imu_mxr_array.mxr[i].data_available = FALSE;

      // Scale the accelerometer measurements
      imu_scale_accel(&imu_mxr_array.imu[i]);

      // Rotate the local accelerometer frame to the body reference frame
      struct Int32Vect3 accel;
      int32_rmat_transp_vmult(&accel, &imu_mxr_array.imu[i].body_to_imu.rmat_i, &imu_mxr_array.imu[i].accel);
      ACCELS_FLOAT_OF_BFP(dist_imu.imu[i].accel_f, accel);
    }
  }

  /* 
   * Compute the angular rates squared and angular acceleration, the
   * inverse of the kinematic matrix is done in MATLAB
   */
  // dist_imu.accel.x = .6*dist_imu.imu[0].accel_f.x+.1333*dist_imu.imu[0].accel_f.z+
  //   .4*dist_imu.imu[1].accel_f.x-.1333*dist_imu.imu[1].accel_f.z;
  // dist_imu.accel.z = -.1333*dist_imu.imu[0].accel_f.x+.6*dist_imu.imu[0].accel_f.z+
  //   .1333*dist_imu.imu[1].accel_f.x+.4*dist_imu.imu[1].accel_f.z;
  // dist_imu.rates_sqr.q = -13.3333*dist_imu.imu[0].accel_f.x+13.3333*dist_imu.imu[1].accel_f.x;
  // dist_imu.rates_dot.q = -13.3333*dist_imu.imu[0].accel_f.z+13.3333*dist_imu.imu[1].accel_f.z; 
}
