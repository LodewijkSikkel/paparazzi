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
 * @file test/subsystems/algebra.c
 */

#include "test/subsystems/algebra.h"

#include "math/pprz_algebra_int.h"

// Macro to fill the diagonal of a square matrix with a scalar
inline void matmn_sdiag(uint8_t m, uint8_t n, float (*mat)[n], float scalar)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    mat[m][m] = scalar;
  }
}

// Macro to return the copied transpose of a matrix
inline void matmn_transp(uint8_t m, uint8_t n, float (*mat_o)[n], float (*mat_i)[n]) 
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat_o[i][j] = mat_i[j][i];
    }
  }
}

// Macro to add two matrices of similar size 
inline void matmn_add(uint8_t m, uint8_t n, float (*mat_o)[n], float (*mat_i)[n])
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat_o[i][j] += mat_i[i][j];
    }
  }
}

// Macro to multiply matrix i with matrix ii
inline void matmn_mul(uint8_t m, uint8_t n, uint8_t nn, float (*mat_o)[nn], float (*mat_i)[n], float (*mat_ii)[nn])
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int jj = 0; jj < nn; jj++) { // loop over the rows of the second matrix
      mat_o[i][jj] = 0;
      for (int j = 0; j < n; j++) { // loop over the columns of the first matrix
        mat_o[i][jj] += mat_i[i][j]*mat_ii[j][jj]; 
      }
    }
  }
}

// Macro to multiply matrix i with the transpose of matrix ii
inline void matmn_mul_transp(uint8_t m, uint8_t n, uint8_t nn, float (*mat_o)[nn], float (*mat_i)[n], float (*mat_ii)[nn]) 
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int jj = 0; jj < nn; jj++) { // loop over the columns of the second matrix
      mat_o[i][jj] = 0;
      for (int j = 0; j < n; j++) { // loop over the columns of the first matrix
        mat_o[i][jj] += mat_i[i][j]*mat_ii[jj][j]; 
      }
    }
  }
}


// Macro to multiply matrix i with vector v 
inline void matmn_vmul(uint8_t m, uint8_t n, float vec_o[m], float (*mat_i)[n], float vec_i[n])
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns of the matrix
      vec_o[i] = mat_i[i][j]*vec_i[j]; 
    }
  }
}

// Macro to multiply a matrix with a constant
inline void matmn_smul(uint8_t m, uint8_t n, float (*mat)[n], float scalar)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat[i][j] = mat[i][j]*scalar; 
    }
  }
}

/*
 * Macro to compute the determinant of a n x n matrix, due to the
 * limitation of libc on the stm32 the dynamic matrix allocation has
 * been removed.
 */
float matnn_det(float mat[][2], uint8_t k) {
  float det = 0;
  float minor[2][2]; // minor matrix

  int i, j, c, m, n;

  if (k < 1) { // this should never be called
  	return 0;
  } else if (k == 1) {
    return mat[0][0];
  } else if (k == 2) {
	  return det = mat[0][0]*mat[1][1]-mat[1][0]*mat[0][1];
  } else {
   	for (c = 0; c < k; c++) { // define the working column
      m = 0;
      n = 0;
  	  for (i = 0; i < k; i++) { // loop over the rows
    		for (j = 0; j < k; j++) {
          minor[i][j] = 0;
    		  if (i != 0 && j != c) { // check if the current column unequal to the working column
      		  minor[m][n] = mat[i][j]; // fill the minor matrix from the first element
            if (n < (k-2)) // define the index of the minor matrix
              n++;
            else {
              n = 0;
              m++;
            }
          }
    		}
  	  }
  	  det += pow(-1,c)*mat[0][c]*matnn_det(minor, k-1); // compute the determinant
	  }
  }
  return det;
}

// Macro that will fill in the cofactor matrix
void matnn_cof(float cof[][2], float mat[][2], uint8_t k)
{
  // Allocate an auxilary temporary minor matrix 
  float minor[2][2];  

  int i, ii, j, jj, m, n;

  for (i = 0; i < k; i++) { // define the working row
  	for (j = 0; j < k; j++) { // define the working column
      m = 0;
      n = 0;
      for (ii = 0; ii < k; ii++) { // loop over the rows of mat
        for (jj = 0; jj < k; jj++) { // loop over the columns of mat
          if (i != ii && j != jj) { // skip the working column abd working row
            minor[m][n] = mat[ii][jj];
            if (n < (k-2)) // Define the index of the minor matrix
                n++;
            else {
              n = 0;
              m++;
            }
          }
        }
      }
      cof[i][j] = pow(-1.,(i+j))*matnn_det(minor, k-1); // compute the cofactor element
  	}
  }
}

// Macro that transposes the cofactor matrix 
void matnn_cof_transp(float mat[][2],float det, uint8_t k) 
{
  float elmt;
  for (int i = 1; i < k; i++) {
  	for (int j = 0; j < i; j++) {
      if (i != j) {
  		  elmt = mat[i][j];
  		  mat[i][j] = mat[j][i]/det;
  		  mat[j][i] = elmt/det;
  	  } 
  	}
  }
}