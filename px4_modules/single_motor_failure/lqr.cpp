/****************************************************************************
 *
 *   Copyright (C) 2024 Inter-IIT Team 62. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file lqr.cpp
 * Implementation of LQR class
 *
 */

#include "lqr.h"
#include <iostream>
#include <vector>
#include <cmath>


using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;




// Computes the inverse of a 2x2 matrix
Matrix LQR::inverse2x2(const Matrix &a) {
    double determinant = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (determinant <= 0) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }
    double invDet = 1.0 / determinant;
    Matrix result(2, std::vector<double>(2));
    result[0][0] =  a[1][1] * invDet;
    result[0][1] = -a[0][1] * invDet;
    result[1][0] = -a[1][0] * invDet;
    result[1][1] =  a[0][0] * invDet;
    return result;
}

// Returns the transpose of a matrix
Matrix LQR::transpose(const Matrix &a) {
    int rows = a.size();
    int cols = a[0].size();
    Matrix result(cols, std::vector<double>(rows));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j][i] = a[i][j];
        }
    }
    return result;
}

// Creates an identity matrix of a given size
Matrix LQR::identityMatrix(int size) {
    Matrix identity(size, std::vector<double>(size, 0.0));
    for (int i = 0; i < size; ++i) {
        identity[i][i] = 1.0;
    }
    return identity;
}

// Multiplies a matrix by a vector
Vector LQR::multiply(const Matrix &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            result[i] += a[i][j] * b[j];
        }
    }
    return result;
}

// Multiplies two matrices
Matrix LQR::multiply(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(b[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < b[0].size(); ++j) {
            result[i][j] = 0;
            for (size_t k = 0; k < b.size(); ++k) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return result;
}
// Adds two matrices element-wise
Matrix LQR::matrixAdd(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(a[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < a[0].size(); ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
    return result;
}

// Subtracts two matrices element-wise
Matrix LQR::matrixSubtract(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(a[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < a[0].size(); ++j) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
    return result;
}

// Clamps each element in the vector to be within the specified min and max values
Vector LQR::clampVector(const Vector &vec, double minVal, double maxVal) {
    Vector clamped = vec;
    for (size_t i = 0; i < clamped.size(); ++i) {
        if (clamped[i] < minVal) {
            clamped[i] = minVal;
        } else if (clamped[i] > maxVal) {
            clamped[i] = maxVal;
        }
    }
    return clamped;
}

// Computes the Euclidean norm of a vector
double LQR::norm(const Vector &vec) {
    double sum = 0.0;
    for (double val : vec) {
        sum += val * val;
    }
    return std::sqrt(sum);
}

// Adds two vectors element-wise
Vector LQR::vectorAdd(const Vector &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

Matrix LQR::getB(double deltat) {
    Matrix B = {
        {0                  ,    l*deltat/ I_xxt},
        {l*deltat/ I_xxt    ,                  0},
        {0                  ,                  0},
        {0                  ,                  0}
    };
    return B;
}

Matrix LQR::getA(double deltat) {
    Matrix A = {
        {0+1                ,a_const*deltat      ,    0             ,                 0},
        {-a_const*deltat  ,0 +1                  ,    0             ,                 0},
        {0                ,-nz_eq*deltat      ,    0   +1          ,       r_eq*deltat},
        {nz_eq*deltat     ,0                   ,  -r_eq*deltat    ,                 0 + 1}
    };
    return A;
}


Vector LQR::lqr(const Vector &actual_state, const Vector &desired_state, const Matrix &Q, const Matrix &R, const Matrix &A, const Matrix &B, double dt) {


    Vector x_error = actual_state;
    for (size_t i = 0; i < actual_state.size(); ++i) {
        x_error[i] = desired_state[i] - actual_state[i];
    }

    int N = 500;
    std::vector<Matrix> P(N + 1);
    P[N] = Q;
    Matrix P_next = Q;

    for (int i = N; i > 0; --i) {


        Matrix P_current = matrixAdd(Q, multiply(transpose(A), multiply(P_next, A)));
        P_current = matrixSubtract(P_current,
                     multiply(multiply(transpose(A), multiply(P_next, B)),
                     multiply(inverse2x2(matrixAdd(R, multiply(transpose(B), multiply(P_next, B)))),
                     multiply(transpose(B), multiply(P_next, A)))));
        P[i - 1] = P_current;
        P_next = P_current;
    }

    Matrix K;
    Vector u;

    K = multiply(inverse2x2(matrixAdd(R, multiply(transpose(B), multiply(P[N], B)))),
                     multiply(transpose(B), multiply(P[N], A)));
    u = multiply(K, x_error);

    return u;
}

Vector LQR::getControlInputs(Vector actual_state, int detected_motor) {

    if(detected_motor == 1 || detected_motor == 3){
        r_eq = +8.5;
        a_const = ((I_xxt - I_zzt)*r_eq/I_xxt) + I_zzp*( w1_eq + w2_eq + w3_eq + w4_eq ) / I_xxt ;
    }
    else{
        r_eq = -8.5;
        a_const = ((I_xxt - I_zzt)*r_eq/I_xxt) + I_zzp*( w1_eq + w2_eq + w3_eq + w4_eq ) / I_xxt ;
    }

    double dt = 1.0;

    Vector desired_state;


    if(detected_motor == 1){
     desired_state = {0.0, 2.53, 0.0, 0.2855};
    }
    else if (detected_motor == 2)
    {
     desired_state = {0.0, -2.53, 0.0, 0.2855};
    }
    else if (detected_motor == 3)
    {
     desired_state = {2.53, 0.0, -0.2855, 0.0};
    }
    else if(detected_motor == 4)
    {
     desired_state = {-2.53, 0.0, 0.2855, 0.0};
    }



    Matrix R = {{1   ,  0},
                {0   ,  1}};

    Matrix Q = {{1     ,0      ,0     ,0},
                {0     ,1      ,0     ,0},
                {0     ,0      ,20    ,0},
                {1     ,0      ,0    ,20}};



    Vector state_error = actual_state;
    for (size_t j = 0; j < actual_state.size(); ++j) {
            state_error[j] -= desired_state[j];
    }

    Vector desired_state_error = {0.0,  0.0,   0.0,   0.0};


    Matrix A = getA(dt);
    Matrix B = getB(dt);
    Vector optimal_control_input = lqr(state_error, desired_state_error, Q, R, A, B, dt);


    return optimal_control_input;
}


