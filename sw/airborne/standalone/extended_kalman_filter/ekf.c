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
 * @brief Standalone Extended Kalman Filter (EKF)
 *
 * @description The Extended Kalman Filter is run on a separate board in 
 * order to reduce the computational load on the autopilot. It receives the
 * sensor (accelerometer, gyroscope and GPS) measurements and sends the 
 * filter state at each periodic cycle. 
 */

#include "ekf.h"

#include "std.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#define DT 1. / PERIODIC_FREQUENCY

#define DRAG_COEFF 0.2

#define MASS 0.3962

#define CUT_OFF_FREQ 1

#define CUT_OFF_TIME_CONSTANT 1 / 2. / M_PI / CUT_OFF_FREQ

struct Sensors sensors;

struct State state, state_hat; // state vector, predicted state vector

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

static float sec(float val)
{
  return 1. / cos(val);
}

static void ekf_time_update(void)
{
  // Compute the state propagation matrix
  matmn_clear(7, 7, F); 
  F[0][0] = tan(state.theta) * (cos(state.phi) * sensors.gyro_f.q - sin(state.phi) * sensors.gyro_f.r) * DT;
  F[0][1] = sec(state.theta) * sec(state.theta) * (sin(state.phi) * sensors.gyro_f.q + cos(state.phi) * sensors.gyro_f.r) * DT;
  F[1][0] = (-sin(state.phi) * sensors.gyro_f.q - cos(state.phi) * sensors.gyro_f.r) * DT;
  F[2][1] = -9.81 * cos(state.theta) * DT;
  F[2][2] = -DRAG_COEFF / MASS * DT;
  F[2][3] = sensors.gyro_f.r * DT;
  F[2][4] = -sensors.gyro_f.q * DT;
  F[2][5] = DRAG_COEFF / MASS * DT;
  F[3][0] = 9.81 * cos(state.phi) * cos(state.theta) * DT;
  F[3][1] = -9.81 * sin(state.phi) * sin(state.theta) * DT;
  F[3][2] = -sensors.gyro_f.r * DT;
  F[3][3] = -DRAG_COEFF / MASS * DT;
  F[3][4] = sensors.gyro_f.p * DT;
  F[3][6] = DRAG_COEFF / MASS * DT;
  F[4][0] = -9.81 * sin(state.phi) * cos(state.theta) * DT;
  F[4][1] = -9.81 * cos(state.phi) * sin(state.theta) * DT;
  F[4][2] = sensors.gyro_f.q * DT;
  F[4][3] = -sensors.gyro_f.p * DT;
  matmn_add(7, 7, F, I);

  // Compute the linear body drag
  struct FloatVect3 drag = { 
    -DRAG_COEFF *(state.vx - state.wx),
    -DRAG_COEFF *(state.vy - state.wy), 
    0
  };

  // Compute the Coriolis acceleration
  struct FloatVect3 coriolis = {
    state.vy * sensors.gyro_f.r - state.vz * sensors.gyro_f.q, 
    state.vz * sensors.gyro_f.p - state.vx * sensors.gyro_f.r, 
    state.vx * sensors.gyro_f.q - state.vy * sensors.gyro_f.p
  };

  // Predict the state update using the state propagation function
  state_hat.phi = state.phi + (sensors.gyro_f.p + sin(state.phi) * tan(state.theta) * sensors.gyro_f.q + cos(state.phi) * tan(state.theta) * sensors.gyro_f.r) * DT;
  state_hat.theta = state.theta + (cos(state.phi) * sensors.gyro_f.q - sin(state.phi) * sensors.gyro_f.r) * DT;
  state_hat.vx = state.vx + (-9.81 * sin(state.theta) + coriolis.x + drag.x / MASS) * DT;
  state_hat.vy = state.vy + (9.81 * sin(state.phi) * cos(state.theta) + coriolis.y + drag.y / MASS) * DT;
  state_hat.vz = state.vz + (9.81 * cos(state.phi) * cos(state.theta) + coriolis.z + sensors.accel_f.z) * DT;
  state_hat.wx = state.wx;
  state_hat.wy = state.wy;

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
  H[0][3] = sensors.gyro_f.r;
  H[0][4] = -sensors.gyro_f.q;
  H[0][5] = DRAG_COEFF / MASS;
  H[1][2] = -sensors.gyro_f.r;
  H[1][3] = -DRAG_COEFF / MASS;
  H[1][4] = sensors.gyro_f.p;
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
    state_hat.vy * sensors.gyro_f.r - state_hat.vz * sensors.gyro_f.q,
    state_hat.vz * sensors.gyro_f.p - state_hat.vx * sensors.gyro_f.r,
    state_hat.vx * sensors.gyro_f.q - state_hat.vy * sensors.gyro_f.p,
  };

  /* Compute the measurement residual by subtracting the transformed
   * states, by the output function h(), from the measured accelerometer data.
   */
  residual[0] = sensors.accel_f.x - (coriolis.x + drag.x / MASS);
  residual[1] = sensors.accel_f.y - (coriolis.y + drag.y / MASS);
  residual[2] = (sensors.gps_body_vel_f.x / 100.) - state_hat.vx;
  residual[3] = (sensors.gps_body_vel_f.y / 100.) - state_hat.vy;
  residual[4] = (sensors.gps_body_vel_f.z / 100.) - state_hat.vz;

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
  state.phi = state_hat.phi + KY[0];
  state.theta = state_hat.theta + KY[1];
  state.vx = state_hat.vx + KY[2];
  state.vy = state_hat.vy + KY[3];
  state.vz = state_hat.vz + KY[4];
  state.wx = state_hat.wx + KY[5];
  state.wy = state_hat.wy + KY[6];

  // Update the covariance estimate
  matmn_mult(7, 5, 7, KH, K, H);
  matmn_smult(7, 7, KH, -1);
  matmn_add(7, 7, KH, I);
  matmn_mult(7, 7, 7, cov, KH, cov_hat);

  ESTIMATE_AVAILABLE = false; // the estimate has been used
}

void parse_sensor_measurements(int32_t gyro_p, int32_t gyro_q, int32_t gyro_r,
                               int32_t accel_x, int32_t accel_y, int32_t accel_z,
                               int32_t gps_body_vel_x, int32_t gps_body_vel_y, int32_t gps_body_vel_z)
{
  sensors.gyro_i.p = (int16_t)gyro_p; // the BFP value still needs to be scaled with #INT32_RATE_FRAC
  sensors.gyro_i.q = (int16_t)gyro_q;
  sensors.gyro_i.r = (int16_t)gyro_r;
  sensors.accel_i.x = (int16_t)accel_x; // the BFP value still needs to be scaled with #INT32_ACCEL_FRAC
  sensors.accel_i.y = (int16_t)accel_y;
  sensors.accel_i.z = (int16_t)accel_z;
  sensors.gps_body_vel_i.x = (int16_t)gps_body_vel_x; // in cm/s
  sensors.gps_body_vel_i.y = (int16_t)gps_body_vel_y;
  sensors.gps_body_vel_i.z = (int16_t)gps_body_vel_z;

  // sensors.gyro_f.p = RATE_FLOAT_OF_BFP(gyro_p); // -> REDO
  // sensors.gyro_f.q = RATE_FLOAT_OF_BFP(gyro_q);
  // sensors.gyro_f.r = RATE_FLOAT_OF_BFP(gyro_r);
  // sensors.accel_f.x = ACCEL_FLOAT_OF_BFP(accel_x);
  // sensors.accel_f.y = ACCEL_FLOAT_OF_BFP(accel_y);
  // sensors.accel_f.z = ACCEL_FLOAT_OF_BFP(accel_z);
  // sensors.gps_body_vel_f.x = SPEED_FLOAT_OF_BFP(gps_body_vel_x);
  // sensors.gps_body_vel_f.y = SPEED_FLOAT_OF_BFP(gps_body_vel_y);
  // sensors.gps_body_vel_f.z = SPEED_FLOAT_OF_BFP(gps_body_vel_z);
}

void ekf_init(void)
{
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
  state.phi = 0;
  state.theta = 0;
  state.vx = 0;
  state.vy = 0;
  state.vz = 0;
  state.wx = 0;
  state.wy = 0;
}

void ekf_periodic(void)
{
  if (ESTIMATE_AVAILABLE)
    ekf_measurement_update();
  ekf_time_update();
}

void ekf_event(void)
{
}