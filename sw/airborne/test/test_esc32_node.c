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
 * @file test_esc32.c
 *
 */

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/can.h"

#include <std.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Logical Communications Channel
// 2 bits [28:27]
#define ESC32_CAN_LCC_MASK            ((uint32_t)0x3<<30)
#define ESC32_CAN_LCC_EXCEPTION       ((uint32_t)0x0<<30)
#define ESC32_CAN_LCC_HIGH            ((uint32_t)0x1<<30)
#define ESC32_CAN_LCC_NORMAL          ((uint32_t)0x2<<30)
#define ESC32_CAN_LCC_INFO            ((uint32_t)0x3<<30)

// Target Type
// 1 bit [26:26]
#define ESC32_CAN_TT_MASK             ((uint32_t)0x1<<29)
#define ESC32_CAN_TT_GROUP            ((uint32_t)0x0<<29)
#define ESC32_CAN_TT_NODE             ((uint32_t)0x1<<29)

// Function ID
// 4 bits [25:22]
#define ESC32_CAN_FID_MASK            ((uint32_t)0xf<<25)
#define ESC32_CAN_FID_RESET_BUS       ((uint32_t)0x0<<25)
#define ESC32_CAN_FID_ACK             ((uint32_t)0x1<<25)
#define ESC32_CAN_FID_NACK            ((uint32_t)0x2<<25)
#define ESC32_CAN_FID_CMD             ((uint32_t)0x3<<25)
#define ESC32_CAN_FID_GET             ((uint32_t)0x4<<25)
#define ESC32_CAN_FID_SET             ((uint32_t)0x5<<25)
#define ESC32_CAN_FID_REPLY           ((uint32_t)0x6<<25)
#define ESC32_CAN_FID_REQ_ADDR        ((uint32_t)0x7<<25)
#define ESC32_CAN_FID_GRANT_ADDR      ((uint32_t)0x8<<25)
#define ESC32_CAN_FID_ERROR           ((uint32_t)0x9<<25)
#define ESC32_CAN_FID_PING            ((uint32_t)0xa<<25)
#define ESC32_CAN_FID_TELEM           ((uint32_t)0xb<<25)

// Data Object Code
// 6 bits [21:16]
#define ESC32_CAN_DOC_MASK            ((uint32_t)0x3f<<19)

// Source ID
// 5 bits [15:11]
#define ESC32_CAN_SID_MASK            ((uint32_t)0x1f<<14)

// Target ID
// 5 bits [10:6]
#define ESC32_CAN_TID_MASK            ((uint32_t)0x1f<<9)

// Sequence ID
// 6 bits [5:0]
#define ESC32_CAN_SEQ_MASK            ((uint32_t)0x3f<<3)

// types
enum {
  ESC32_CAN_TYPE_ESC = 1,
  ESC32_CAN_TYPE_SERVO,
  ESC32_CAN_TYPE_SENSOR,
  ESC32_CAN_TYPE_SWITCH,
  ESC32_CAN_TYPE_OSD,
  ESC32_CAN_TYPE_UART,
  ESC32_CAN_TYPE_HUB,
  ESC32_CAN_TYPE_NUM
};

// commands
enum {
  ESC32_CAN_CMD_DISARM = 1,
  ESC32_CAN_CMD_ARM,
  ESC32_CAN_CMD_START,
  ESC32_CAN_CMD_STOP,
  ESC32_CAN_CMD_SETPOINT10,
  ESC32_CAN_CMD_SETPOINT12,
  ESC32_CAN_CMD_SETPOINT16,
  ESC32_CAN_CMD_RPM,
  ESC32_CAN_CMD_CFG_READ,
  ESC32_CAN_CMD_CFG_WRITE,
  ESC32_CAN_CMD_CFG_DEFAULT,
  ESC32_CAN_CMD_TELEM_RATE,
  ESC32_CAN_CMD_TELEM_VALUE,
  ESC32_CAN_CMD_BEEP,
  ESC32_CAN_CMD_POS,
  ESC32_CAN_CMD_USER_DEFINED,
  ESC32_CAN_CMD_RESET,
  ESC32_CAN_CMD_STREAM,
  ESC32_CAN_CMD_ON,
  ESC32_CAN_CMD_OFF
};

// data types
enum {
  ESC32_CAN_DATA_GROUP = 1,
  ESC32_CAN_DATA_TYPE,
  ESC32_CAN_DATA_ID,
  ESC32_CAN_DATA_INPUT_MODE,
  ESC32_CAN_DATA_RUN_MODE,
  ESC32_CAN_DATA_STATE,
  ESC32_CAN_DATA_PARAM_ID,
  ESC32_CAN_DATA_TELEM,
  ESC32_CAN_DATA_VERSION,
  ESC32_CAN_DATA_VALUE,
  ESC32_CAN_DATA_PARAM_NAME1,
  ESC32_CAN_DATA_PARAM_NAME2
};

// telemetry values
enum {
  ESC32_CAN_TELEM_NONE = 0,
  ESC32_CAN_TELEM_STATUS,
  ESC32_CAN_TELEM_STATE,
  ESC32_CAN_TELEM_TEMP,
  ESC32_CAN_TELEM_VIN,
  ESC32_CAN_TELEM_AMPS,
  ESC32_CAN_TELEM_RPM,
  ESC32_CAN_TELEM_ERRORS,
  ESC32_CAN_TELEM_VALUE,
  ESC32_CAN_TELEM_NUM
};

typedef struct {
    uint32_t uuid;
    uint8_t network_ID;
    uint8_t type;
    uint8_t can_ID;
    uint8_t group_ID;
    uint8_t subgroup_ID;
} can_nodes_t;

typedef struct {
    can_nodes_t nodes[(ESC32_CAN_TID_MASK>>9)+1];
    uint8_t next_node_slot;
    uint8_t seq_ID;
    bool initialized;
} can_struct_t;

can_struct_t can_data;

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

void esc32_node_can_init(void);
uint8_t esc32_can_send(uint32_t ID, uint8_t t_ID, uint8_t length, uint8_t *data);

void esc32_node_can_rx_cb(uint32_t ID, uint8_t *data, int len);

void esc32_node_can_init(void)
{
  // initialize the CAN bus
  ppz_can_init(esc32_node_can_rx_cb);
}

uint8_t esc32_can_send(uint32_t ID, uint8_t t_ID, uint8_t length, uint8_t *data)
{
  uint8_t seq_ID = 0;

  ppz_can_transmit((ID >> 3) | ((t_ID & 0x1f) << 6) | seq_ID, data, length);
  
  return 0;
}

static inline void esc32_can_send_get_addr(void) 
{
    uint8_t d[8];
    uint32_t uuid = 0;

    *((uint32_t *)&d[0]) = uuid;

    d[4] = ESC32_CAN_TYPE_ESC;
    d[5] = 0;

    LED_ON(2);
    esc32_can_send(ESC32_CAN_LCC_NORMAL | ESC32_CAN_TT_NODE | ESC32_CAN_FID_REQ_ADDR, 0, 6, d);
}

static void esc32_node_can_bus_reset(void)
{
  // ask for new address
  esc32_can_send_get_addr();
}

void esc32_node_can_rx_cb(uint32_t id, uint8_t *data, int len __attribute__((unused)))
{
  if ((id<<3)==(ESC32_CAN_LCC_EXCEPTION | ESC32_CAN_TT_GROUP | ESC32_CAN_FID_RESET_BUS)) {
    LED_ON(2);
    esc32_node_can_bus_reset();
  } else if {
    LED_ON(3);
  }
}

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }

  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((0.5 / PERIODIC_FREQUENCY), NULL);

  esc32_node_can_init();
}

static inline void main_periodic_task(void)
{
}

static inline void main_event_task(void)
{
}
