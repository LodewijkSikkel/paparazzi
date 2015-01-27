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
 * 
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

struct FloatRates fil_ang_rate = {0., 0., 0.};
struct FloatRates fil_ang_rate_dot = {0., 0., 0.};
struct FloatRates fil_ang_rate_ddot = {0., 0., 0.};

#define STABILIZATION_ANG_VEL_2_RPM 60 / 2 / M_PI

struct FloatRates smc_delta_rpm = {0., 0., 0.};

#define STABILIZATION_FILT_OMEGA_SQR STABILIZATION_SMC_FILT_OMEGA * STABILIZATION_SMC_FILT_OMEGA

#define STABILIZATION_FILT_DT 1 / 512.0

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
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_INDI", send_att_indi);
#endif
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  FLOAT_RATES_ZERO(fil_ang_rate);
  FLOAT_RATES_ZERO(fil_ang_rate_dot);
  FLOAT_RATES_ZERO(fil_ang_rate_ddot);

  FLOAT_RATES_ZERO(smc_delta_rpm);
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

#define SIGN(_v, _s) 
{
  _s = _v < 0 ? -1 : 1;
}

#define BOUND_CONTROLS(_v, _min, _max) 
{ 
  _v = _v < _min ? _min : _v > _max ? _max : _v; 
}

static void stabilization_indi_filter_gyro(void) {
  fil_ang_rate.p = fil_ang_rate.p + fil_ang_rate_dot.p * STABILIZATION_FILT_DT;
  fil_ang_rate.q = fil_ang_rate.q + fil_ang_rate_dot.q * STABILIZATION_FILT_DT;
  fil_ang_rate.r = fil_ang_rate.r + fil_ang_rate_dot.r * STABILIZATION_FILT_DT;
  
  fil_ang_rate_dot.p = fil_ang_rate_dot.p + fil_ang_rate_ddot.p * STABILIZATION_FILT_DT;
  fil_ang_rate_dot.q = fil_ang_rate_dot.q + fil_ang_rate_ddot.q * STABILIZATION_FILT_DT;
  fil_ang_rate_dot.r = fil_ang_rate_dot.r + fil_ang_rate_ddot.r * STABILIZATION_FILT_DT;
  
  fil_ang_rate_ddot.p = -fil_ang_rate_dot.p * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->p - filtered_rate.p ) * STABILIZATION_FILT_OMEGA_SQR;
  fil_ang_rate_ddot.q = -fil_ang_rate_dot.q * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->q - filtered_rate.q ) * STABILIZATION_FILT_OMEGA_SQR;
  fil_ang_rate_ddot.r = -fil_ang_rate_dot.r * 2 * STABILIZATION_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->r - filtered_rate.r ) * STABILIZATION_FILT_OMEGA_SQR;
}

static void attitude_run_indi(int32_t indi_commands[], struct Int32Quat *att_err)
{
  angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                      - reference_acceleration.rate_p * filtered_rate.p;
  angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                      - reference_acceleration.rate_q * filtered_rate.q;
  angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                      - reference_acceleration.rate_r * filtered_rate.r;

  indi_du.p = inv_control_effectiveness.p * (angular_accel_ref.p - filtered_rate_deriv.p);
  indi_du.q = inv_control_effectiveness.q * (angular_accel_ref.q - filtered_rate_deriv.q);
  indi_du.r = inv_control_effectiveness.r * (angular_accel_ref.r - filtered_rate_deriv.r);

  u_in.p = indi_u.p + indi_du.p;
  u_in.q = indi_u.q + indi_du.q;
  u_in.r = indi_u.r + indi_du.r;

  BOUND_CONTROLS(u_in.p, -4500, 4500);
  BOUND_CONTROLS(u_in.q, -4500, 4500);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST]/2);
  BOUND_CONTROLS(u_in.r, -half_thrust, half_thrust);

  //Propagate input filters
  stabilization_indi_filter_inputs();

  //Don't increment if thrust is off
  if(stabilization_cmd[COMMAND_THRUST]<300) {
    FLOAT_RATES_ZERO(indi_u);
    FLOAT_RATES_ZERO(indi_du);
    FLOAT_RATES_ZERO(u_act_dyn);
    FLOAT_RATES_ZERO(u_in);
    FLOAT_RATES_ZERO(udot);
    FLOAT_RATES_ZERO(udotdot);
  }

  //Save error for displaying purposes
  att_err_x = QUAT1_FLOAT_OF_BFP(att_err->qx);

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = u_in.p;
  indi_commands[COMMAND_PITCH] = u_in.q;
  indi_commands[COMMAND_YAW] = u_in.r;
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
  attitude_run_indi(stabilization_att_indi_cmd, &att_err);

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