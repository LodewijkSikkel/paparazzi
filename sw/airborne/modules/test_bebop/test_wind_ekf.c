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
 * @file modules/test_bebop/test_wind_ekf.c
 */

#include "test_wind_ekf.h"

#include <stdio.h>

#include "std.h"
#include "state.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"
#include "subsystems/imu.h"

#define DT 1. / PERIODIC_FREQUENCY

#define DRAG_COEFF 0.2

#define MASS 0.3962

#define CUT_OFF_FREQ 1

#define CUT_OFF_TIME_CONSTANT .25 / 2. / M_PI / CUT_OFF_FREQ

static abi_event gyro_ev;

static abi_event accel_ev;

struct FilterVect3 {
  Butterworth2LowPass x;
  Butterworth2LowPass y;
  Butterworth2LowPass z;
};

struct FilterVect3 filter;

struct FloatVect3 accel_meas; // accelerometer measurements

struct FloatRates u; // input vector (gyro measurements)

struct StateVector {
  float phi; // roll angle [rad]
  float theta; // pitch angle [rad]
  float vx; // longitudinal ground speed [m/s] in body coordinates
  float vy; // lateral ground speed [m/s] in body coordinates
  float vz; // downward ground speed [m/s] in body coordinates
  float wx; // wind speed along the longitudinal body axis
  float wy; // wind speed along the lateral body axis
};

struct StateVector cur_state; // state vector

struct StateVector state_hat; // predicted state vector

float residual[5];

float cov[7][7] = {{0}}; // covariance matrix

float cov_hat[7][7] = {{0}}; //predicted covariance matrix

float F[7][7] = {{0}}, H[5][7] = {{0}}; // state prediction matrix, measurement matrix

float Q[7][7] = {{0}}, R[5][5] = {{0}}; // prediction noise, measurement noise

float I[7][7] = {{0}};

float FP[7][7]; // predicted covariance estimate (working matrix)

float HP[5][7], HPHT[5][5]; // innovation covariance (working matrices)

float K[7][5], COF[5][5], COFT[5][5], PHT[7][5]; // Kalman gain (working matrices)

float KY[7], KH[7][7]; // coveriance update (working matrices)

bool ESTIMATE_AVAILABLE = false;

struct FloatVect3 gps_ned_vel_f;

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel);
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_state(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_WIND_ESTIMATION_FILTER(trans, dev, AC_ID,
                                       &cur_state.phi, 
                                       &cur_state.theta, 
                                       &cur_state.vx, 
                                       &cur_state.vy, 
                                       &cur_state.vz, 
                                       &cur_state.wx, 
                                       &cur_state.wy,
                                       &gps_ned_vel_f.x,
                                       &gps_ned_vel_f.y,
                                       &gps_ned_vel_f.z);
}

#endif

static void init_butterworth_2_low_pass_vect3(void)
{
  init_butterworth_2_low_pass(&filter.x, CUT_OFF_TIME_CONSTANT, DT, 0);
  init_butterworth_2_low_pass(&filter.y, CUT_OFF_TIME_CONSTANT, DT, 0);
  init_butterworth_2_low_pass(&filter.z, CUT_OFF_TIME_CONSTANT, DT, 0);
}

static void update_butterworth_2_low_pass_vect3(struct FloatVect3 vect)
{
  update_butterworth_2_low_pass(&filter.x, vect.x);
  update_butterworth_2_low_pass(&filter.y, vect.y);
  update_butterworth_2_low_pass(&filter.z, vect.z);
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  // Process the incoming accelerometer measurements
  ACCELS_FLOAT_OF_BFP(accel_meas, *accel);

  update_butterworth_2_low_pass_vect3(accel_meas);

  // struct FloatVect3 accel_filt;
  // accel_filt.x = get_butterworth_2_low_pass(&filter.x);
  // accel_filt.y = get_butterworth_2_low_pass(&filter.y);
  // accel_filt.z = get_butterworth_2_low_pass(&filter.z);

  // DOWNLINK_SEND_FILTER_ACCEL(DefaultChannel, DefaultDevice,
  //                            &accel_filt.x, &accel_filt.y, &accel_filt.z);
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
  // Process the incoming gyro measurements
  RATES_FLOAT_OF_BFP(u, *gyro);
}

static float sec(float val)
{
  return 1. / cos(val);
}

static void ekf_time_update(void)
{
  // Compute the state propagation matrix
  matmn_clear(7, 7, F); 
  F[0][0] = tan(cur_state.theta) * (cos(cur_state.phi) * u.q - sin(cur_state.phi) * u.r) * DT;
  F[0][1] = sec(cur_state.theta) * sec(cur_state.theta) * (sin(cur_state.phi) * u.q + cos(cur_state.phi) * u.r) * DT;
  F[1][0] = (-sin(cur_state.phi) * u.q - cos(cur_state.phi) * u.r) * DT;
  F[2][1] = -9.81 * cos(cur_state.theta) * DT;
  F[2][2] = -DRAG_COEFF / MASS * DT;
  F[2][3] = u.r * DT;
  F[2][4] = -u.q * DT;
  F[2][5] = DRAG_COEFF / MASS * DT;
  F[3][0] = 9.81 * cos(cur_state.phi) * cos(cur_state.theta) * DT;
  F[3][1] = -9.81 * sin(cur_state.phi) * sin(cur_state.theta) * DT;
  F[3][2] = -u.r * DT;
  F[3][3] = -DRAG_COEFF / MASS * DT;
  F[3][4] = u.p * DT;
  F[3][6] = DRAG_COEFF / MASS * DT;
  F[4][0] = -9.81 * sin(cur_state.phi) * cos(cur_state.theta) * DT;
  F[4][1] = -9.81 * cos(cur_state.phi) * sin(cur_state.theta) * DT;
  F[4][2] = u.q * DT;
  F[4][3] = -u.p * DT;
  matmn_add(7, 7, F, I);

  // Compute the linear body drag
  struct FloatVect3 drag = { 
    -DRAG_COEFF *(cur_state.vx - cur_state.wx),
    -DRAG_COEFF *(cur_state.vy - cur_state.wy), 
    0
  };

  // Compute the Coriolis acceleration
  struct FloatVect3 coriolis = {
    cur_state.vy *u.r - cur_state.vz *u.q, 
    cur_state.vz *u.p - cur_state.vx *u.r, 
    cur_state.vx *u.q - cur_state.vy *u.p
  };

  // Predict the state update using the state propagation function
  state_hat.phi = cur_state.phi + (u.p + sin(cur_state.phi) * tan(cur_state.theta) * u.q + cos(cur_state.phi) * tan(cur_state.theta) * u.r) * DT;
  state_hat.theta = cur_state.theta + (cos(cur_state.phi) * u.q - sin(cur_state.phi) * u.r) * DT;
  state_hat.vx = cur_state.vx + (-9.81 * sin(cur_state.theta) + coriolis.x + drag.x / MASS) * DT;
  state_hat.vy = cur_state.vy + (9.81 * sin(cur_state.phi) * cos(cur_state.theta) + coriolis.y + drag.y / MASS) * DT;
  state_hat.vz = cur_state.vz + (9.81 * cos(cur_state.phi) * cos(cur_state.theta) + coriolis.z + get_butterworth_2_low_pass(&filter.z)) * DT;
  state_hat.wx = cur_state.wx;
  state_hat.wy = cur_state.wy;

  // Predict the covariance update using the Jacobian matrix
  matmn_mult(7, 7, 7, FP, F, cov);
  matmn_mult_transp(7, 7, 7, cov_hat, FP, F);
  matmn_add(7, 7, cov_hat, Q);

  ESTIMATE_AVAILABLE = true; // a new state estimate is available
}

static void ekf_measurement_update(void)
{
  // Compute the measurement matrix
  H[0][2] = -DRAG_COEFF / MASS;
  H[0][3] = u.r;
  H[0][4] = -u.q;
  H[0][5] = DRAG_COEFF / MASS;
  H[1][2] = -u.r;
  H[1][3] = -DRAG_COEFF / MASS;
  H[1][4] = u.p;
  H[1][6] = DRAG_COEFF / MASS;
  H[2][2] = 1;
  H[3][3] = 1;
  H[4][4] = 1;

  // Compute the estimated linear body drag
  struct FloatVect3 drag = {
    -DRAG_COEFF * (state_hat.vx - state_hat.wx),
    -DRAG_COEFF * (state_hat.vy - state_hat.wy),
    0,
  };

  // Compute the estimated Coriolis acceleration
  struct FloatVect3 coriolis = {
    state_hat.vy * u.r - state_hat.vz * u.q,
    state_hat.vz * u.p - state_hat.vx * u.r,
    state_hat.vx * u.q - state_hat.vy * u.p,
  };

  // Compute the ground speed in body coordinates
  struct NedCoor_i gps_ned_vel_i;
  ned_of_ecef_vect_i(&gps_ned_vel_i, &state.ned_origin_i, &gps.ecef_vel); 
  gps_ned_vel_f.x = gps_ned_vel_i.x / 100.;
  gps_ned_vel_f.y = gps_ned_vel_i.y / 100.;
  gps_ned_vel_f.z = gps_ned_vel_i.z / 100.;
  struct FloatVect3 gps_body_vel;
  float_rmat_vmult(&gps_body_vel, stateGetNedToBodyRMat_f(), &gps_ned_vel_f);

  /* Compute the measurement residual by subtracting the transformed
   * states, by the output function h(), from the measured accelerometer data.
   */
  residual[0] = get_butterworth_2_low_pass(&filter.x) - (coriolis.x + drag.x / MASS);
  residual[1] = get_butterworth_2_low_pass(&filter.y) - (coriolis.y + drag.y / MASS);
  residual[2] = gps_body_vel.x-state_hat.vx;
  residual[3] = gps_body_vel.y-state_hat.vy;
  residual[4] = gps_body_vel.z-state_hat.vz;

  /* Compute the residual covariance using the Jacobian matrix and
   * include the measurement noise covariance R
   */
  matmn_mult(5, 7, 7, HP, H, cov_hat);
  matmn_mult_transp(5, 7, 5, HPHT, HP, H);
  matmn_add(5, 5, HPHT, R);

  // Compute the Kalman update gain
  matnn_cof(COF, HPHT, 5);
  matmn_transp(5, 5, COFT, COF);
  matmn_smult(5, 5, COFT, 1. / matnn_det(HPHT, 5));
  matmn_mult_transp(7, 7, 5, PHT, cov_hat, H);
  matmn_mult(7, 5, 5, K, PHT, COFT);

  // Update the state estimate
  matmn_vmult(7, 5, KY, K, residual);
  cur_state.phi = state_hat.phi + KY[0];
  cur_state.theta = state_hat.theta + KY[1];
  cur_state.vx = state_hat.vx + KY[2];
  cur_state.vy = state_hat.vy + KY[3];
  cur_state.vz = state_hat.vz + KY[4];
  cur_state.wx = state_hat.wx + KY[5];
  cur_state.wy = state_hat.wy + KY[6];

  // Update the covariance estimate
  matmn_mult(7, 5, 7, KH, K, H);
  matmn_smult(7, 7, KH, -1);
  matmn_add(7, 7, KH, I);
  matmn_mult(7, 7, 7, cov, KH, cov_hat);

  ESTIMATE_AVAILABLE = false; // the estimate has been used
}

void wind_ekf_init(void)
{
  // Initialize the second-order Butterworth filter vector
  init_butterworth_2_low_pass_vect3();

  // Bind the ABI messages
  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, accel_cb);

#if PERIODIC_TELEMETRY
  // Register the periodic messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_ESTIMATION_FILTER, send_state);
#endif

  // Define the initial covariance diagonal
  matmn_sdiag(7, 7, cov, 1e2);

  // Define the prediction error
  matmn_sdiag(7, 7, Q, 1e1);

  // Define the measurement error
  matmn_sdiag(5, 5, R, 1e2);
  R[3][3] = 1e-5;
  R[4][4] = 1e-5;
  R[4][4] = 1e-5;

  // Define the identity matrix
  matmn_sdiag(7, 7, I, 1.);

  // Set all the state variable equal to zero 
  cur_state.phi = 0;
  cur_state.theta = 0;
  cur_state.vx = 0;
  cur_state.vy = 0;
  cur_state.vz = 0;
  cur_state.wx = 0;
  cur_state.wy = 0;
}

void wind_ekf_periodic(void)
{
  if (ESTIMATE_AVAILABLE)
    ekf_measurement_update();
  ekf_time_update();
}

void wind_ekf_event(void)
{
  ImuEvent();
}