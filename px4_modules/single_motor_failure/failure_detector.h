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
 * @file failure_detector.cpp
 * Header for Detector Class
 *
 */

#pragma once

#include <cmath>
#include <vector>
#include <array>
#include <px4_platform_common/app.h>
#include <uORB/topics/vehicle_odometry.h>

class Detector
{
public:
    Detector() {}
    ~Detector() {}

    int main();

    static px4::AppState appState; /* track requests to terminate app */

    // Current state variables
    double ax = 0.0, ay = 0.0, az = 0.0;
    double roll_rate = 0.0, pitch_rate = 0.0, yaw_rate = 0.0;
    // double threshold_acceleration = 4.0;  // Example threshold
    int detected_motor_failure = -1;
    int failed_motor = -1;
    int timestamp = 0;
    int motor_failed_timestamp = 0;
    int failure_detected_timestamp = 0;

    double threshold_acceleration = 3.0, threshold_roll_rate = 50;
    double slope_sensitivity_factor_az = 1;
    double slope_sensitivity_factor_roll_rate = 10;

    // Vectors for storing the last 20 data points
    std::vector<std::array<double, 4>> last_20;

    // Previous state variables for calculating deltas
    double prev_vx = 0.0, prev_vy = 0.0, prev_vz = 0.0;
    double prev_roll_rad = 0.0, prev_pitch_rad = 0.0, prev_yaw_rad = 0.0;
    double prev_timestamp = 0.0;

    // Position and velocity variables
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // Function declarations
    double* quaternionToRPY (double w, double x, double y, double z);
    void calculateAccelerations(int timestmp);
    double normalizeAngle(double angle);
    double calculateAngularRate(double current, double previous, double dt);
    void calculateAttitudeRates(int timestmp, double roll_rad, double pitch_rad, double yaw_rad);
    void detectFailure();
    void odometryUpdate(int vehicle_odometry_fd, vehicle_odometry_s &vehicle_odometry);




private:

};
