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



// Returns Input vector using the K matrix tuned using pydrake
Vector LQR::lqr(const Vector &actual_state, const Vector &desired_state) {


    Vector x_error = actual_state;
    for (size_t i = 0; i < actual_state.size(); ++i) {
        x_error[i] = desired_state[i] - actual_state[i];
    }



    Matrix K;
    Vector u;
    
    // K matrix coming from LQR_Optimiser.ipynb
    
    K = {
    {-3.31812040e-02, -4.21937270e-01, 5.36364498e-01, 6.35371466e-01, 6.31023742e-01},
    {4.55570092e-01, 2.92259473e-02, -5.68221882e-01, -7.48228833e-01, 5.84679071e-01},
    {6.02633476e-02, 4.56929927e-01, 8.18210808e-01, -7.01855853e-01, -6.19868705e-01}
    };

    u = multiply(K, x_error);


    return u;
}

Vector LQR::getControlInputs(Vector actual_state, int detected_motor) {



    Vector desired_state;


    // desired state coming from LQR_Optimiser.ipynb
     desired_state = {0.0, 2.25361451700798, 11.694423666728527, 0.0, 0.1892};


    Vector state_error = actual_state;
    for (size_t j = 0; j < actual_state.size(); ++j) {
            state_error[j] -= desired_state[j];
    }


    Vector desired_state_error = {0.0,  0.0,   0.0,   0.0,  0.0};



    Vector optimal_control_input = lqr(state_error, desired_state_error);


    return optimal_control_input;
}

