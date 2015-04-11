/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

// Own header
#include "doublet_module.h"

// Paparazzi headers
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#define deg2rad M_PI/180.

/* Initialize a timer */
uint32_t time_cur;

/* Record the previous time instance */
uint32_t time_prev;

/* Initialize the doublet setpoiunt */
struct doublet_setpoint_t doublet_setpoint;

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Set rool/pitch to 0 degrees and psi to current heading */
  doublet_setpoint.cmd.phi = 0;
  doublet_setpoint.cmd.theta = 0;
  doublet_setpoint.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  /* Do nothing */
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&doublet_setpoint.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

void doublet_module_init(void)
{
  doublet_setpoint.t_mill_first = 100;
  doublet_setpoint.t_mill_second = 100;
}

void doublet_module_start(void) 
{
}

void doublet_module_run(void)
{
  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }

  /* The module runs on the event loop, implying that the time increment
   * of each passing is 1/512s \approx 2ms
   *
   * TODO: Standardize the code
   */
  time_cur = get_sys_time_msec();
  time_prev = time_cur;
  while (( time_cur - time_prev) < doublet_setpoint.t_mill_first)
  {
  	doublet_setpoint.cmd.phi = 10 * deg2rad;
  	time_cur = get_sys_time_msec();
  }

  time_cur = get_sys_time_msec();
  time_prev = time_cur;
  while (( time_cur - time_prev) < doublet_setpoint.t_mill_second)
  {
  	doublet_setpoint.cmd.phi = -10 * deg2rad;
  	time_cur = get_sys_time_msec();
  }
}

void doublet_module_stop(void) 
{

}