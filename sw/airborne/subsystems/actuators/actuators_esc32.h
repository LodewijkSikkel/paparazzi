/*
 * Copyright (C) 2014 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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

/** @file actuators_esc32.h
 *  Actuator driver for AutoQuad ESC32 motor controllers.
 */

#ifndef ACTUATORS_ESC32_H
#define ACTUATORS_ESC32_H

#include "subsystems/actuators/actuators_can.h"

#include <std.h>
#include <string.h>

#define MOTORS_CELL_VOLTS 3.7f
#define MOTORS_SCALE ((1<<12) - 1)

#define MOTORS_NUM 4
#define MOTORS_CAN_GROUP_SIZE 4
#define MOTORS_CAN_TELEM_RATE 100 // Hz

enum esc32_states {
  ESC32_STATE_DISARMED = 0,
  ESC32_STATE_STOPPED,
  ESC32_STATE_NOCOMM,
  ESC32_STATE_STARTING,
  ESC32_STATE_RUNNING
};

typedef struct {
    unsigned int state   : 3;
    unsigned int vin     : 12; // x 100
    unsigned int amps    : 14; // x 100
    unsigned int rpm     : 15;
    unsigned int duty    : 8;  // x (255/100)
    unsigned int temp    : 9;  // (Deg C + 32) * 4
    unsigned int errCode : 3;
}  __attribute__((packed)) esc32_can_status_t;

typedef struct {
  can_nodes_t *can[MOTORS_NUM];
  can_group16_t can_groups[MOTORS_NUM/MOTORS_CAN_GROUP_SIZE];
  esc32_can_status_t can_status[MOTORS_NUM];
  uint8_t num_groups;
  uint16_t cmds[MOTORS_NUM];
} esc32_struct_t;

extern enum esc32_states esc32_state[MOTORS_NUM];

extern esc32_struct_t esc32_data;

static inline float motors_max(void) {
    return MOTORS_SCALE;
}

// initialize all esc32 speed controllers
extern void esc32_init(void);

// arm the esc32 speed controllers
extern int esc32_arm(void);

// disarm the esc32 speed controllers
extern void esc32_disarm(void);

// commit the motor commands
extern void esc32_commit(void);

// set the motor commands
extern void esc32_set(uint8_t i, uint16_t v);

// process a configuration command
extern void esc32_config_cmd(uint8_t i);

// set the telemetry (default: status)
extern void esc32_set_telem_value(uint32_t tt, uint8_t t_id, uint8_t index, uint8_t value);

// define the telemetery rate (default: 100)
extern void esc32_set_telem_rate(uint32_t tt, uint8_t t_id, uint16_t rate);

// process telemetry 
extern void esc32_receive_telem(uint8_t can_id, uint8_t doc, void *data_ptr);

// paparazzi placeholders
#define ActuatorESC32Set(_i, _v) { esc32_set(_i, _v); }
#define ActuatorsESC32Init() esc32_init()
#define ActuatorsESC32Commit() esc32_commit()

#endif /* ACTUATORS_ESC32_H */
