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
 * @file peripherals/mxr9150mz.c
 *
 * Driver for the MXR9150MZ
 */

#include "mcu_periph/adc.h"
#include "peripherals/mpu60x0_spi.h"

static struct adc_buf mxr_adc_buf[NB_ANALOG_MXR_ADC];

void mpu60x0_spi_init(struct Mxr9150mz *mxr)
{
  // Set overrun to 0
  mxr->overrun = 0;

  mxr->initialized = FALSE;

  #ifdef ADC_CHANNEL_GYRO_P
    adc_buf_channel(ADC_CHANNEL_ACCEL_X, &mxr_adc_buf[0], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  #endif
  #ifdef ADC_CHANNEL_GYRO_Q
    adc_buf_channel(ADC_CHANNEL_ACCEL_Y, &mxr_adc_buf[1], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  #endif
  #ifdef ADC_CHANNEL_GYRO_R
    adc_buf_channel(ADC_CHANNEL_ACCEL_Z, &mxr_adc_buf[2], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  #endif
}

// Configuration function called once before normal use
void mxr9150mz_start_configure(struct Mxr9150mz *mxr)
{
  mxr->initialized = TRUE; 
}

void mxr9150mz_read(struct Mxr9150mz *mxr)
{
  if (mxr->initialized) {
    
  }
}