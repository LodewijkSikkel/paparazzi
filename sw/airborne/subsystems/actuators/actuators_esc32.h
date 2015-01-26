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
  uint16_t *can_ptrs[MOTORS_NUM];
  can_group16_t can_groups[MOTORS_NUM/MOTORS_CAN_GROUP_SIZE];
  esc32_can_status_t can_status[MOTORS_NUM];
  uint32_t can_status_time[MOTORS_NUM];
  uint32_t can_telem_req_time[MOTORS_NUM];
  uint8_t num_groups;
} motors_struct_t;

extern motors_struct_t motors_data;

static inline float motors_max(void) {
    return MOTORS_SCALE;
}

// process incoming telemetry
void esc32_receive_telem(uint8_t can_id, uint8_t doc, void *data_ptr);

// initialize all esc32 speed controllers
extern void esc32_init(void);

// send values to the esc32 nodes
void esc32_send_values(uint8_t motor_id, uint16_t value);

// send values to the esc32 group
void esc32_can_send_groups(void);

// arm the esc32 speed controllers
extern int esc32_arm(void);

// disarm the esc32 speed controllers
extern void esc32_disarm(void);

// paparazzi placeholders
#define ActuatorESC32Set(_i, _v) { esc32_send_values(_i, _v); }
#define ActuatorsESC32Init() esc32_init()
#define ActuatorsESC32Commit() esc32_can_send_groups()

#endif /* ACTUATORS_ESC32_H */
