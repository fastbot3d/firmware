/*
 * Unicorn 3D Printer Firmware
 * vector.h
 * vector library for auto bed leveling
 *
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * Unicorn is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Unicorn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _VECTOR_H
#define _VECTOR_H

#include "parameter.h"

typedef struct {
    float x;
    float y;
    float z;
} vector_t;

typedef struct {
  volatile float matrix[9];
} matrix_t;

#if defined (__cplusplus)
extern "C" {
#endif

void vector_cross(vector_t left, vector_t right, vector_t *result);
float vector_get_length(vector_t v);
vector_t vector_get_normal(vector_t v);
void  vector_normalize(vector_t *v);
void vector_apply_rotation(vector_t *v, matrix_t m);

matrix_t matrix_create_from_rows(vector_t row_0, vector_t row_1, vector_t row_2);
matrix_t matrix_create_look_at(vector_t target);
void matrix_transpos(matrix_t original, matrix_t *target);
void matrix_set_to_identity(matrix_t *m);

void apply_rotation_xyz(matrix_t m, float *x, float *y, float *z);
void debug_matrix(matrix_t *m);

#if defined (__cplusplus)
}
#endif
#endif
