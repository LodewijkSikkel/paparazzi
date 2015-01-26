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
 * @file actuator_can_driver.h
 *
 */

#ifndef ACTUATORS_CAN
#define ACTUATORS_CAN

#ifndef MOTORS_CAN_RETRIES
#define MOTORS_CAN_RETRIES 3
#endif

#ifndef CAN_TIMEOUT
#define CAN_TIMEOUT 250 //ms
#endif  

#include <std.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Logical Communications Channel
// 2 bits [28:27]
#define CAN_LCC_MASK            ((uint32_t)0x3<<30)
#define CAN_LCC_EXCEPTION       ((uint32_t)0x0<<30)
#define CAN_LCC_HIGH            ((uint32_t)0x1<<30)
#define CAN_LCC_NORMAL          ((uint32_t)0x2<<30)
#define CAN_LCC_INFO            ((uint32_t)0x3<<30)

// Target Type
// 1 bit [26:26]
#define CAN_TT_MASK             ((uint32_t)0x1<<29)
#define CAN_TT_GROUP            ((uint32_t)0x0<<29)
#define CAN_TT_NODE             ((uint32_t)0x1<<29)

// Function ID
// 4 bits [25:22]
#define CAN_FID_MASK            ((uint32_t)0xf<<25)
#define CAN_FID_RESET_BUS       ((uint32_t)0x0<<25)
#define CAN_FID_ACK             ((uint32_t)0x1<<25)
#define CAN_FID_NACK            ((uint32_t)0x2<<25)
#define CAN_FID_CMD             ((uint32_t)0x3<<25)
#define CAN_FID_GET             ((uint32_t)0x4<<25)
#define CAN_FID_SET             ((uint32_t)0x5<<25)
#define CAN_FID_REPLY           ((uint32_t)0x6<<25)
#define CAN_FID_REQ_ADDR        ((uint32_t)0x7<<25)
#define CAN_FID_GRANT_ADDR      ((uint32_t)0x8<<25)
#define CAN_FID_ERROR           ((uint32_t)0x9<<25)
#define CAN_FID_PING            ((uint32_t)0xa<<25)
#define CAN_FID_TELEM           ((uint32_t)0xb<<25)

// Data Object Code
// 6 bits [21:16]
#define CAN_DOC_MASK            ((uint32_t)0x3f<<19)

// Source ID
// 5 bits [15:11]
#define CAN_SID_MASK            ((uint32_t)0x1f<<14)

// Target ID
// 5 bits [10:6]
#define CAN_TID_MASK            ((uint32_t)0x1f<<9)

// Sequence ID
// 6 bits [5:0]
#define CAN_SEQ_MASK            ((uint32_t)0x3f<<3)

// types
enum {
  CAN_TYPE_ESC = 1,
  CAN_TYPE_SERVO,
  CAN_TYPE_SENSOR,
  CAN_TYPE_SWITCH,
  CAN_TYPE_OSD,
  CAN_TYPE_UART,
  CAN_TYPE_HUB,
  CAN_TYPE_NUM
};

// commands
enum {
  CAN_CMD_DISARM = 1,
  CAN_CMD_ARM,
  CAN_CMD_START,
  CAN_CMD_STOP,
  CAN_CMD_SETPOINT10,
  CAN_CMD_SETPOINT12,
  CAN_CMD_SETPOINT16,
  CAN_CMD_RPM,
  CAN_CMD_CFG_READ,
  CAN_CMD_CFG_WRITE,
  CAN_CMD_CFG_DEFAULT,
  CAN_CMD_TELEM_RATE,
  CAN_CMD_TELEM_VALUE,
  CAN_CMD_BEEP,
  CAN_CMD_POS,
  CAN_CMD_USER_DEFINED,
  CAN_CMD_RESET,
  CAN_CMD_STREAM,
  CAN_CMD_ON,
  CAN_CMD_OFF
};

// data types
enum {
  CAN_DATA_GROUP = 1,
  CAN_DATA_TYPE,
  CAN_DATA_ID,
  CAN_DATA_INPUT_MODE,
  CAN_DATA_RUN_MODE,
  CAN_DATA_STATE,
  CAN_DATA_PARAM_ID,
  CAN_DATA_TELEM,
  CAN_DATA_VERSION,
  CAN_DATA_VALUE,
  CAN_DATA_PARAM_NAME1,
  CAN_DATA_PARAM_NAME2
};

// telemetry values
enum {
  CAN_TELEM_NONE = 0,
  CAN_TELEM_STATUS,
  CAN_TELEM_STATE,
  CAN_TELEM_TEMP,
  CAN_TELEM_VIN,
  CAN_TELEM_AMPS,
  CAN_TELEM_RPM,
  CAN_TELEM_ERRORS,
  CAN_TELEM_VALUE,
  CAN_TELEM_NUM
};

typedef struct {
    uint16_t value1;
    uint16_t value2;
    uint16_t value3;
    uint16_t value4;
} __attribute__((packed)) can_group16_t;

typedef struct {
    uint32_t uuid;
    uint8_t network_id;
    uint8_t type;
    uint8_t can_id;
    uint8_t group_id;
    uint8_t subgroup_id;
} can_nodes_t;

typedef void can_telem_callback_t(uint8_t node_id, uint8_t doc, void *p);

typedef struct {
    can_nodes_t nodes[(CAN_TID_MASK>>9)+1];

    can_telem_callback_t *telem_funcs[CAN_TYPE_NUM-1];
    uint32_t timeouts;

    uint8_t response_data[64*8]; // must be word aligned
    volatile uint8_t responses[64]; // arbitrary size

    uint8_t next_node_slot;
    uint8_t seq_id;
    uint8_t initialized;
} can_struct_t;

extern can_struct_t can_data;

// set the values of a group
uint8_t *can_set_group(uint8_t tid, uint8_t gid, uint8_t sgid);

// initialize the CAN driver
extern void init_can(void);

// send a message over the CAN bus
extern uint8_t can_send(uint32_t id, uint8_t tid, uint8_t length, uint8_t *data);

// CAN bus callback function
extern void actuators_can_rx_callback(uint32_t id, uint8_t *data, int len);

// send a beep command to a node or group
extern void can_command_beep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur);

// get the state of a node
uint8_t *can_get_state(uint8_t tid);

// arm a node or group
extern void can_command_arm(uint32_t tt, uint8_t tid);

// disarm a node or group
extern void can_command_disarm(uint32_t tt, uint8_t tid);

// find the pointer to a CAN node (default: 3 retries)
extern can_nodes_t *can_find_node(uint8_t type, uint8_t can_id);

// set the telemetry (default: status)
extern void can_set_telem_value_no_wait(uint32_t tt, uint8_t tid, uint8_t index, uint8_t value);

// define the telemetery rate (default: 100)
extern void can_set_telem_rate_no_wait(uint32_t tt, uint8_t tid, uint16_t rate);

// start the node or group
extern void can_command_start(uint32_t tt, uint8_t tid);

// start the node or group
uint8_t *can_command_stop(uint32_t tt, uint8_t tid);

// reset the CAN bus
extern void can_reset_bus(void);

// register the telemetry the callback function
extern void can_telem_register(can_telem_callback_t *func, uint8_t type);

// send a 16-bit setpoint to a node or group
extern void can_command_setpoint16(uint8_t tid, uint8_t *data);

#endif /* ACTUATORS_CAN */