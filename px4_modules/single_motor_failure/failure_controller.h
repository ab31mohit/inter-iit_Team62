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
 * @file failure_controller.h
 *  Header of Controller class
 *
 */
#pragma once

#include <px4_platform_common/app.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/drone_state.h>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class Controller
{
public:
    Controller() {}
    ~Controller() {}

    int main(int detected_motor);

    static px4::AppState appState; /* track requests to terminate app */


    double* quaternionToRPY (double qw, double qx, double qy, double qz);
    double* vectorQuaternionTransformation (double* q, double* v);
    double* hamiltonianProduct (double* q1, double* q2);
    double* vectorFRD2Plus (double* v);
    Vector vectorFRD2Plus (Vector v);
    double* AorBodyFrame (double* q, double* N);
    double* rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate);
    double calculateNorm(Vector vec);

    void innerLoop();
    void outerLoop();
    Matrix calculateRotationMatrixInverse(double roll, double pitch, double yaw);
    Vector vectorAdd(const Vector &a, const Vector &b);
    Vector multiply(const Matrix &a, const Vector &b);

    drone_state_s state_data = {};  // Zero-initialized
    orb_advert_t drone_state_publisher = orb_advertise(ORB_ID(drone_state), &state_data);


private:

};
