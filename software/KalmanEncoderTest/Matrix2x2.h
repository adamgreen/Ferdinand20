/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef MATRIX_2x2_H_
#define MATRIX_2x2_H_

#include <dsp.h>

class Vector2
{
public:
    Vector2(float e0, float e1)
    {
        m_0 = e0;
        m_1 = e1;
    }

    Vector2()
    {
    }

    Vector2 divide(float div)
    {
        Vector2 v;

        v.m_0 = m_0 / div;
        v.m_1 = m_1 / div;

        return v;
    }

    Vector2 multiply(float s)
    {
        Vector2 v;

        v.m_0 = m_0 * s;
        v.m_1 = m_1 * s;

        return v;
    }

    Vector2 add(const Vector2& v)
    {
        Vector2 sum;

        sum.m_0 = m_0 + v.m_0;
        sum.m_1 = m_1 + v.m_1;

        return sum;
    }

    float m_0;
    float m_1;
};

class Matrix2x2
{
public:
    Matrix2x2(float m00, float m01,
              float m10, float m11)
    {
        m_00 = m00;
        m_01 = m01;

        m_10 = m10;
        m_11 = m11;
    }

    Matrix2x2()
    {
    }

    Vector2 multiply(const Vector2& vec)
    {
        Vector2 prod;

        prod.m_0 = m_00*vec.m_0 + m_01*vec.m_1;
        prod.m_1 = m_10*vec.m_0 + m_11*vec.m_1;

        return prod;
    }

    Matrix2x2 multiply(float s)
    {
        Matrix2x2 prod;

        prod.m_00 = m_00*s;
        prod.m_01 = m_01*s;

        prod.m_10 = m_10*s;
        prod.m_11 = m_11*s;

        return prod;
    }

    Matrix2x2 multiply(const Matrix2x2& m)
    {
        Matrix2x2 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_10;
        prod.m_01 = m_00*m.m_01 + m_01*m.m_11;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_10;
        prod.m_11 = m_10*m.m_01 + m_11*m.m_11;

        return prod;
    }

    Matrix2x2 multiplyTransposed(const Matrix2x2& m)
    {
        Matrix2x2 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_01;
        prod.m_01 = m_00*m.m_10 + m_01*m.m_11;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_01;
        prod.m_11 = m_10*m.m_10 + m_11*m.m_11;

        return prod;
    }

    Matrix2x2 inverse()
    {
        Matrix2x2               result;
        arm_matrix_instance_f32 instanceIn = {2, 2, (float*)m_data};
        arm_matrix_instance_f32 instanceOut = {2, 2, (float*)result.m_data};

        arm_mat_inverse_f32(&instanceIn, &instanceOut);

        return result;
    }

    Matrix2x2 add(const Matrix2x2& m)
    {
        Matrix2x2 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01 + m.m_01;

        sum.m_10 = m_10 + m.m_10;
        sum.m_11 = m_11 + m.m_11;

        return sum;
    }

    // This function is an optimization of the general add() method which can be used when parameter m is known to only
    // contain zeroes in its non-diagonal elements.
    Matrix2x2 addDiagonal(const Matrix2x2& m)
    {
        Matrix2x2 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01;

        sum.m_10 = m_10;
        sum.m_11 = m_11 + m.m_11;

        return sum;
    }

    Matrix2x2 subtract(const Matrix2x2& m)
    {
        Matrix2x2 sum;

        sum.m_00 = m_00 - m.m_00;
        sum.m_01 = m_01 - m.m_01;

        sum.m_10 = m_10 - m.m_10;
        sum.m_11 = m_11 - m.m_11;

        return sum;
    }

    union
    {
        float m_data[2][2];
        struct
        {
            float m_00;
            float m_01;

            float m_10;
            float m_11;
        };
    };
};

#endif /* MATRIX_2x2_H_ */
