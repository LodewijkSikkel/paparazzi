/*
 * Copyright (C) 2015 Lodewijk Sikke <l.n.c.sikkel@tudelft.nl>
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
 * @file peripherals/mxr9150mz.h
 *
 * Driver for the MXR9150MZ
 */

#ifndef MXR9150MZ_H
#define MXR9150MZ_H

#define NB_ANALOG_MXR_ADC 3

#include "std.h"
#include "math/pprz_algebra_int.h"

struct Mxr9150mz {
	int overrun;
	bool_t initialized;
	union {
    	struct Int32Vect3 vect; ///< accel data vector in accel coordinate system
    	uint16_t values[3];     ///< accel data values accessible by channel index
  } data_accel;
  bool_t data_available;
};

// Functions
extern void mxr9150mz_init(struct Mxr9150mz *mxr, uint8_t adc_channel_x, uint8_t adc_channel_y, uint8_t adc_channel_z);
extern void mxr9150mz_start_configure(struct Mxr9150mz *mxr);
extern void mxr9150mz_read(struct Mxr9150mz *mxr);
extern void mxr9150mz_event(struct Mxr9150mz *mxr);

/// convenience function: read or start configuration if not already initialized
static inline void mxr9150mz_periodic(struct Mxr9150mz *mxr)
{
  if (mxr->initialized) {
    mxr9150mz_read(mxr);
  } else {
    mxr9150mz_start_configure(mxr);
  }
}

#endif // MXR9150MZ_H
