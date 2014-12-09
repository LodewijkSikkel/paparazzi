/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/com/bluegiga.c"
 * @author C. De Wagter
 * Communicate through BlueGiga SPI modules
 */

#include "modules/com/bluegiga.h"

#include "mcu_periph/gpio.h"

void bluegiga_init()
{
  gpio_setup_output(GPIOC, GPIO6);
}

void bluegiga_periodic()
{
  gpio_toggle(GPIOC, GPIO6);
}

void bluegiga_event() {}


