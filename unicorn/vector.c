/*
 * Unicorn 3D Printer Firmware
 * vector.c
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <math.h>

#include "vector.h"

void vector_cross(vector_t left, vector_t right, vector_t *result)
{
    result->x = left.y * right.z - left.z * right.y;
    result->y = left.z * right.x - left.x * right.z;
    result->z = left.x * right.y - left.y * right.x;
}

float vector_get_length(vector_t v)
{
    float len = sqrt((v.x * v.x) + (v.y *v.y) + (v.z * v.z));
    return len;
}

vector_t vector_get_normal(vector_t v)
{
	vector_t v_normal = v;
	vector_normalize(&v_normal);
	return v_normal;
}

void vector_normalize(vector_t *v)
{
    float length = vector_get_length(*v);
    v->x /= length;
    v->y /= length;
    v->z /= length;
}

void vector_apply_rotation(vector_t *v, matrix_t m)
{
    float result_x = v->x * m.matrix[3 * 0 + 0]
                   + v->y * m.matrix[3 * 1 + 0]
                   + v->z * m.matrix[3 * 2  + 0];

    float result_y = v->x * m.matrix[3 * 0 + 1]
                   + v->y * m.matrix[3 * 1 + 1]
                   + v->z * m.matrix[3 * 2  + 1];

    float result_z = v->x * m.matrix[3 * 0 + 2]
                   + v->y * m.matrix[3 * 1 + 2]
                   + v->z * m.matrix[3 * 2  + 2];

    v->x = result_x;
    v->y = result_y;
    v->z = result_z;
}

void apply_rotation_xyz(matrix_t m, float *x, float *y, float *z)
{
    vector_t v;
    v.x = *x;
    v.y = *y;
    v.z = *z;
    
    vector_apply_rotation(&v, m);
    
    *x = v.x;
    *y = v.y;
    *z = v.z;
}

matrix_t matrix_create_from_rows(vector_t row_0, vector_t row_1, vector_t row_2)
{
    matrix_t m;
    m.matrix[0] = row_0.x;
    m.matrix[1] = row_0.y;
    m.matrix[2] = row_0.z;

    m.matrix[3] = row_1.x;
    m.matrix[4] = row_1.y;
    m.matrix[5] = row_1.z;

    m.matrix[6] = row_2.x;
    m.matrix[7] = row_2.y;
    m.matrix[8] = row_2.z;
    return m;
}

void matrix_set_to_identity(matrix_t *m)
{
    m->matrix[0] = 1.0; m->matrix[1] = 0;   m->matrix[2] = 0;
    m->matrix[3] = 0;   m->matrix[4] = 1.0; m->matrix[5] = 0;
    m->matrix[6] = 0;   m->matrix[7] = 0;   m->matrix[8] = 1.0;
}

void debug_matrix(matrix_t *m)
{
#if 0
printf("matrix 0-8:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    m->matrix[0], m->matrix[1], m->matrix[2],
    m->matrix[3], m->matrix[4], m->matrix[5],
    m->matrix[6], m->matrix[7], m->matrix[8]);
#endif
}


matrix_t matrix_create_look_at(vector_t target)
{
    vector_t x_row, y_row, z_row;

    z_row.x = target.x;
    z_row.y = target.y;
    z_row.z = target.z;
    vector_normalize(&z_row);

    x_row.x = 1;
    x_row.y = 0;
    x_row.z = -target.x/target.z; 
    vector_normalize(&x_row);

	vector_cross(z_row, x_row, &y_row);
    vector_normalize(&y_row);

    matrix_t rot = matrix_create_from_rows(x_row, y_row, z_row);
    return rot;
}

void matrix_transpos(matrix_t original, matrix_t *target)
{
    target->matrix[0] = original.matrix[0];
    target->matrix[1] = original.matrix[3];
    target->matrix[2] = original.matrix[6];

    target->matrix[3] = original.matrix[1];
    target->matrix[4] = original.matrix[4];
    target->matrix[5] = original.matrix[7];

    target->matrix[6] = original.matrix[2];
    target->matrix[7] = original.matrix[5];
    target->matrix[8] = original.matrix[8];
}

