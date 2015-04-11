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

/**
 * @file modules/testing/doublet_module.h
 * @brief Doublet manoeuver 
 *
 * The aircraft is excited along one of the controlled axis (TODO: Create a 
 * #define to define this axis, currently only the roll axis is used) by 
 * overwriting the module horizontal guidance loop and immediately the 
 * horizontal guidance loop is returned to the default attitude control loop
 */

#ifndef DOUBLET_MODULE_H
#define DOUBLET_MODULE_H

#include "std.h"
#include "math/pprz_algebra_int.h"

/* The opticflow stabilization */
struct doublet_setpoint_t {
  uint16_t t_mill_first;        ///< Duration in milliseconds of first step input (max: 65536 millseconds)
  int32_t t_mill_second;        ///< Duration in milliseconds of second step input (max: 65536 millseconds)
  struct Int32Eulers cmd;   	///< The commands that are send to the stabilization loop
};
extern struct doublet_setpoint_t doublet_setpoint;

// Overwrite the horizontal guidance loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);

// Module functions
extern void doublet_module_init(void);
extern void doublet_module_run(void);
extern void doublet_module_start(void);
extern void doublet_module_stop(void);

#endif /* DOUBLET_MODULE_H */