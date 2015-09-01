/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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
 *
 */

#include "logger_uart.h"

#include "state.h"
#include "led.h"
#include "subsystems/imu.h"
#include "mcu_periph/uart.h"

struct logger_uart_data_struct logger_uart_data;

void logger_uart_init(void)
{
}

void logger_uart_periodic(void)
{
  logger_uart_data.start = 0xFFFF; // start byte

  logger_uart_data.acc_x = imu.accel_unscaled.x;
  logger_uart_data.acc_y = imu.accel_unscaled.y;
  logger_uart_data.acc_z = imu.accel_unscaled.z;

  uint8_t crc = 0;
  uint8_t *p = (uint8_t*) &logger_uart_data;
  for (int i=0; i<8; i++)
  {
    crc += p[i];
    uart_transmit(&uart3, p[i]);
  }
  uart_transmit(&uart3, crc);
}


