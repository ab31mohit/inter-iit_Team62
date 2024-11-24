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
 * @file lqr.h
 * Header file for LQR class
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <cmath>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class LQR
{
public:
    LQR() {}
    ~LQR() {}

    Vector getControlInputs(Vector actual_state, int detected_motor);


    Matrix multiply(const Matrix &a, const Matrix &b);
    Vector multiply(const Matrix &a, const Vector &b);
    Matrix transpose(const Matrix &a);
    Matrix inverse2x2(const Matrix &a);
    Matrix identityMatrix(int size);
    Vector clampVector(const Vector &vec, double minVal, double maxVal);
    double norm(const Vector &vec);
    Vector vectorAdd(const Vector &a, const Vector &b);
    Matrix matrixAdd(const Matrix &a, const Matrix &b);
    Matrix matrixSubtract(const Matrix &a, const Matrix &b);
    Vector lqr(const Vector &actual_state, const Vector &desired_state, const Matrix &Q, const Matrix &R, const Matrix &A, const Matrix &B, double dt);
    Vector state_space_model(const Matrix &A, const Vector &state, const Matrix &B, Vector control_input);
    Matrix getB(double deltat);
    Matrix getA(double deltat);


    //Vehicle Constants
    double I_xxt = 0.02166666666666667;
    double I_yyt = 0.02166666666666667;
    double I_zzt = 0.04000000000000001;
    double I_zzp = 1.1928e-4;
    double l = 0.25;




    // Equilibrium state variables
    double nx_eq;
    double ny_eq;
    double nz_eq;
    double p_eq;
    double q_eq;
    double r_eq = -8.5;
    double w1_eq = 738.0 ;
    double w2_eq = 522.0 ;
    double w3_eq =  738.0;
    double w4_eq = 0.0;

   // A matrix variable
    double a_const = ((I_xxt - I_zzt)*r_eq/I_xxt) + I_zzp*( w1_eq + w2_eq + w3_eq + w4_eq ) / I_xxt ;



private:

};
