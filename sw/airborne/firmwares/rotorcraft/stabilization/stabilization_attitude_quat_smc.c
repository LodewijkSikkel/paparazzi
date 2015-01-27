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

/** @file stabilization_attitude_quat_indi.c
 *  NEEDS TO BE UPDATED: UGLY CODE
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

int32_t stabilization_att_indi_cmd[COMMANDS_NB];

struct FloatRates filt_ang_rate = {0., 0., 0.};
struct FloatRates filt_ang_rate_dot = {0., 0., 0.};
struct FloatRates filt_ang_rate_ddot = {0., 0., 0.};

#define STABILIZATION_ATTITUDE_ANG_VEL_2_RPM 60 / 2 / M_PI

struct FloatVect3 smc_delta_rpm = {0., 0., 0.};

#define STABILIZATION_STABILIZATION_ATTITUDE_FILT_OMEGA_FILT_OMEGA_SQR STABILIZATION_ATTITUDE_FILT_OMEGA * STABILIZATION_ATTITUDE_FILT_OMEGA

#define STABILIZATION_ATTITUDE_FILT_DT 1 / 512.0

#ifndef STABILIZATION_ATTITUDE_LAMBDA_0 
#error LAMBDA_0 has to be defined 
#endif

struct FloatVect3 lambda_0 = STABILIZATION_ATTITUDE_LAMBDA_0;

#ifndef STABILIZATION_ATTITUDE_LAMBDA_1 
#error LAMBDA_1 has to be defined 
#endif

struct FloatVect3 lambda_1 = STABILIZATION_ATTITUDE_LAMBDA_1;

#ifndef STABILIZATION_ATTITUDE_INP_DIST_MAT
#error STABILIZATION_ATTITUDE_INP_DIST_MAT has to be defined
#endif

struct FloatMat33 inp_dist_mat = { STABILIZATION_ATTITUDE_INP_DIST_MAT };

struct FloatRates inverse_b = {0., 0., 0.};

#ifndef STABILIZATION_ATTITUDE_K
#error STABILIZATION_ATTITUDE_K has to be defined 
#endif 

struct FloatVect3 K = STABILIZATION_ATTITUDE_K;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ahrs_ref_quat(void) {
  struct Int32Quat* quat = stateGetNedToBodyQuat_i();
  DOWNLINK_SEND_AHRS_REF_QUAT(DefaultChannel, DefaultDevice,
      &stab_att_ref_quat.qi,
      &stab_att_ref_quat.qx,
      &stab_att_ref_quat.qy,
      &stab_att_ref_quat.qz,
      &(quat->qi),
      &(quat->qx),
      &(quat->qy),
      &(quat->qz));
}

static void send_att_smc(void) 
{
}

#endif

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_INDI", send_att_smc);
#endif
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  FLOAT_RATES_ZERO(filt_ang_rate);
  FLOAT_RATES_ZERO(filt_ang_rate_dot);
  FLOAT_RATES_ZERO(filt_ang_rate_ddot);

  FLOAT_VECT3_ZERO(smc_delta_rpm);

  inverse_b.p = 1 / (lambda_1.x * inp_dist_mat[0]);
  inverse_b.q = 1 / (lambda_1.y * inp_dist_mat[5]);
  inverse_b.r = 1 / (lambda_1.z * inp_dist_mat[9]);
}

void stabilization_attitude_set_failsafe_setpoint(void) {
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy) {
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading) {
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

#define BOUND_CONTROLS(_v, _min, _max) 
{ 
  _v = _v < _min ? _min : _v > _max ? _max : _v; 
}

static float stabilization_smc_sign(float value) 
{
  return value < 0 ? -1 : (value == 0 ? 0 : 1);
}

static void stabilization_smc_filter(void) {
  filt_ang_rate.p = filt_ang_rate.p + filt_ang_rate_dot.p * STABILIZATION_FILT_DT;
  filt_ang_rate.q = filt_ang_rate.q + filt_ang_rate_dot.q * STABILIZATION_FILT_DT;
  filt_ang_rate.r = filt_ang_rate.r + filt_ang_rate_dot.r * STABILIZATION_FILT_DT;
  
  filt_ang_rate_dot.p = filt_ang_rate_dot.p + filt_ang_rate_ddot.p * STABILIZATION_FILT_DT;
  filt_ang_rate_dot.q = filt_ang_rate_dot.q + filt_ang_rate_ddot.q * STABILIZATION_FILT_DT;
  filt_ang_rate_dot.r = filt_ang_rate_dot.r + filt_ang_rate_ddot.r * STABILIZATION_FILT_DT;
  
  filt_ang_rate_ddot.p = -filt_ang_rate_dot.p * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->p - filtered_rate.p ) * STABILIZATION_FILT_OMEGA_SQR;
  filt_ang_rate_ddot.q = -filt_ang_rate_dot.q * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->q - filtered_rate.q ) * STABILIZATION_FILT_OMEGA_SQR;
  filt_ang_rate_ddot.r = -filt_ang_rate_dot.r * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->r - filtered_rate.r ) * STABILIZATION_FILT_OMEGA_SQR;
}

static void attitude_run_smc(int32_t indi_commands[], struct Int32Quat *att_err)
{
  struct FloatRates h, u;

  uint16_t rpm_motor[4] = actuators_bebop.rpm_obs;

  h.p = lambda_0.x * stateGetBodyRates_f()->p + lambda_1.x * filt_ang_rate_dot.p;
  h.q = lambda_0.y * stateGetBodyRates_f()->q + lambda_1.y * filt_ang_rate_dot.q;
  h.r = lambda_0.z * stateGetBodyRates_f()->r + lambda_1.z * filt_ang_rate_dot.r;

  smc_delta_rpm.x = -(h.p + K.x * stabilization_smc_sign(lambda_0.x * att_err->qx 
                  + lambda_1.x * stateGetBodyRates_f()->p)) * inverse_b.p;
  smc_delta_rpm.y = -(h.q + K.y * stabilization_smc_sign(lambda_0.y * att_err->qy 
                  + lambda_1.y * stateGetBodyRates_f()->q)) * inverse_b.q;
  smc_delta_rpm.z = -(h.r + K.z * stabilization_smc_sign(lambda_0.z * att_err->qz 
                  + lambda_1.z * stateGetBodyRates_f()->r)) * inverse_b.r;

  u.p = -rpm_motor[0] + rpm_motor[1] + rpm_motor[2] - rpm_motor[3];
  u.q =  rpm_motor[0] + rpm_motor[1] - rpm_motor[2] - rpm_motor[3];
  u.r =  rpm_motor[0] - rpm_motor[1] + rpm_motor[2] - rpm_motor[3];

  u.p = u.p + smc_delta_rpm.x;
  u.q = u.q + smc_delta_rpm.y;
  u.r = u.r + smc_delta_rpm.z;

  BOUND_CONTROLS(u.p, -4500, 4500);
  BOUND_CONTROLS(u.q, -4500, 4500);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST]/2);
  BOUND_CONTROLS(u.r, -half_thrust, half_thrust);

  //Don't increment if thrust is off
  if(stabilization_cmd[COMMAND_THRUST]<300) {
    FLOAT_VECT3_ZERO(smc_delta_rpm);
  }

  /*  SMC feedback */
  indi_commands[COMMAND_ROLL] = u.p;
  indi_commands[COMMAND_PITCH] = u.q;
  indi_commands[COMMAND_YAW] = u.r;
}

void stabilization_attitude_run(bool_t enable_integrator) {

  /* Propagate the second order filter on the gyroscopes */
  stabilization_indi_filter_gyro();

  /* attitude error */
  struct Int32Quat att_err;
  struct Int32Quat* att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);

  /* wrap it in the shortest direction */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /* compute the INDI command */
  attitude_run_smc(stabilization_att_indi_cmd, &att_err);

  stabilization_cmd[COMMAND_ROLL] = stabilization_att_indi_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_indi_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_indi_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}