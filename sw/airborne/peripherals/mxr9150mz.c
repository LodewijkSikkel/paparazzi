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

// Include own header
#include "peripherals/mxr9150mz.h"

#include "mcu_periph/adc.h"

#include "led.h"
#include "subsystems/datalink/downlink.h"

static struct adc_buf mxr_adc_buf[NB_ANALOG_MXR_ADC];

void mxr9150mz_init(struct Mxr9150mz *mxr, uint8_t adc_channel_x, uint8_t adc_channel_y, uint8_t adc_channel_z)
{
  // Set overrun to 0
  mxr->overrun = 0;

  mxr->initialized = FALSE;

  adc_buf_channel(adc_channel_x, &mxr_adc_buf[0], ADC_CHANNEL_ACCEL_NB_SAMPLES);

  adc_buf_channel(adc_channel_y, &mxr_adc_buf[1], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  
  adc_buf_channel(adc_channel_z, &mxr_adc_buf[2], ADC_CHANNEL_ACCEL_NB_SAMPLES);
}

// Configuration function called once before normal use
void mxr9150mz_start_configure(struct Mxr9150mz *mxr)
{
  mxr->initialized = TRUE; 
}

void mxr9150mz_read(struct Mxr9150mz *mxr)
{
  if (mxr->initialized) {
    // Actual nb of ADC measurements per channel per periodic loop
    static int last_head = 0;

    mxr->overrun = mxr_adc_buf[0].head - last_head;
    if (mxr->overrun < 0) {
      mxr->overrun += ADC_CHANNEL_ACCEL_NB_SAMPLES;
    }
    last_head = mxr_adc_buf[0].head;

    // Read All Measurements
    mxr->data_accel.vect.x = mxr_adc_buf[0].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
    mxr->data_accel.vect.y = mxr_adc_buf[1].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
    mxr->data_accel.vect.z = mxr_adc_buf[2].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;

    mxr->data_available = TRUE;
  }
}