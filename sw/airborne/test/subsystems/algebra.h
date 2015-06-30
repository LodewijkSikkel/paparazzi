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
 * @file test/subsystems/algebra.h
 */

#ifndef ALGEBRA_H
#define ALGEBRA_H

#include <stdlib.h>
#include <inttypes.h>

extern void matmn_sdiag(uint8_t m, uint8_t n, float (*mat)[n], float scalar);
extern void matmn_transp(uint8_t m, uint8_t n, float (*mat_o)[n], float (*mat_i)[n]); // nb row mat_i, nb columns mat_i
extern void matmn_add(uint8_t m, uint8_t n, float (*mat_o)[n], float (*mat_i)[n]); // nb row mat_i, nb columns mat_i
extern void matmn_mul(uint8_t m, uint8_t n, uint8_t nn, float (*mat_o)[nn], float (*mat_i)[n], float (*mat_ii)[nn]); // nb row mat_i, nb columns mat_i, nb columns mat_ii 
extern void matmn_mul_transp(uint8_t m, uint8_t n, uint8_t nn, float (*mat_o)[nn], float (*mat_i)[n], float (*mat_ii)[nn]); // nb row mat_i, nb columns mat_i, nb rows mat_ii 
extern void matmn_vmul(uint8_t m, uint8_t n, float vec_o[m], float (*mat_i)[n], float vec_i[n]); // nb row mat_i, nb columns mat_i
extern void matmn_smul(uint8_t m, uint8_t n, float (*mat)[n], float scalar);

extern float matnn_det(float mat[][2], uint8_t k);
extern void matnn_cof(float cof[][2], float mat[][2], uint8_t k);
extern void matnn_cof_transp(float mat[][2],float det, uint8_t k);

#endif // ALGEBRA_H