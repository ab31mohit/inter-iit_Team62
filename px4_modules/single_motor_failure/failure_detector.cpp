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
 * @file  failure_detector.cpp
 * Implementation of Detector Class
 * Detects motor failure using adaptive thresholding on the accelation values
 */




#include "failure_detector.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <array>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_odometry.h>

px4::AppState Detector::appState;

int Detector::main(){

    appState.setRunning(true);
    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;


    while (!appState.exitRequested()) {

        px4_sleep(0.001);

        //Update odometry

        odometryUpdate(vehicle_odometry_fd, odometry);


        //Run detect_failure

         if (detected_motor_failure == -1) {
            detectFailure();
           }

         else{
            return (detected_motor_failure+1);
         }


    }



    return 0;
}

// Calculates roll, pitch, yaw from quaternion
double* Detector::quaternionToRPY (double qw, double qx, double qy, double qz){

    double* rpy = new double[3];


   // Roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    double qroll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    double qpitch;
    if (std::abs(sinp) >= 1)
        qpitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
        qpitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double qyaw = std::atan2(siny_cosp, cosy_cosp);

    rpy[0] = qroll;
    rpy[1] = qpitch;
    rpy[2] = qyaw;

    return rpy;

}

// Calculates x, y, z accelerations
    void Detector::calculateAccelerations(int timestmp) {
        if (prev_timestamp <= 0.0) {
            ax = ay = az = 0.0;
            return;
        }

        double dt = (timestmp - prev_timestamp) * 1e-6;
        if (dt > 0) {
            ax = (vx - prev_vx) / dt;
            ay = (vy - prev_vy) / dt;
            az = (vz - prev_vz) / dt;
        }
    }

// Calculate normalised angles
    double Detector::normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

// Calculates angular rate
    double Detector::calculateAngularRate(double current, double previous, double dt) {
        double diff = normalizeAngle(current - previous);
        return dt > 0 ? diff / dt : 0.0;
    }

// Calculates p,q,r rates
    void Detector::calculateAttitudeRates(int timestmp, double roll_rad, double pitch_rad, double yaw_rad) {
        if (prev_timestamp <= 0.0) {
            roll_rate = pitch_rate = yaw_rate = 0.0;
            return;
        }

        double dt = (timestmp - prev_timestamp) * 1e-6;
        if (dt > 0) {
            double roll_rate_rad = calculateAngularRate(roll_rad, prev_roll_rad, dt);
            double pitch_rate_rad = calculateAngularRate(pitch_rad, prev_pitch_rad, dt);
            double yaw_rate_rad = calculateAngularRate(yaw_rad, prev_yaw_rad, dt);

            roll_rate = roll_rate_rad * (180.0 / M_PI);
            pitch_rate = pitch_rate_rad * (180.0 / M_PI);
            yaw_rate = yaw_rate_rad * (180.0 / M_PI);
        }
    }


    void Detector::detectFailure() {

        //For calculating average over 20 values
        if (last_20.size() < 20) {
            last_20.push_back({az, roll_rate, pitch_rate, yaw_rate});
        } else {
            last_20.erase(last_20.begin());
            last_20.push_back({az, roll_rate, pitch_rate, yaw_rate});
        }

        double avg_az = 0.0, avg_roll_rate = 0.0, avg_pitch_rate = 0.0, avg_yaw_rate = 0.0;
        for (const auto& data : last_20) {
            avg_az += data[0];
            avg_roll_rate += data[1];
            avg_pitch_rate += data[2];
            avg_yaw_rate += data[3];
        }
        int size = last_20.size();
        avg_az /= size;
        avg_roll_rate /= size;
        avg_pitch_rate /= size;
        avg_yaw_rate /= size;

        // Calculating average slope for az and roll_rate

        double az_slope = 0.0, roll_rate_slope = 0.0;
        if (size >= 5) {
            az_slope = (last_20[size - 1][0] - last_20[size - 5][0]) / 5.0;
            roll_rate_slope = (last_20[size - 1][1] - last_20[size - 5][1]) / 5.0;
        }

        // Dynamic threshold adjustment based on slope
        if (az_slope > 0) {
            threshold_acceleration = std::max(1.5, 3.0 - az_slope * slope_sensitivity_factor_az);
        } else {
            threshold_acceleration = 3.0;
        }

        threshold_roll_rate = std::max(25.0, 50.0 - abs(roll_rate_slope) * slope_sensitivity_factor_roll_rate);
        // threshold_acceleration = 3.0;
        // threshold_roll_rate = 50.0;
        // Motor failure detection
        if (avg_az >= threshold_acceleration || abs(avg_roll_rate)>threshold_roll_rate) {
            // failure_detected_timestamp = timestamp;
            if (avg_pitch_rate < 0 && avg_yaw_rate < 0) {
                detected_motor_failure = 0;
            }
            else if (avg_pitch_rate > 0 && avg_yaw_rate < 0) {
                detected_motor_failure = 1;
            }
            else if (avg_pitch_rate < 0 && avg_yaw_rate > 0) {
                detected_motor_failure = 2;
            }
            else if (avg_pitch_rate > 0 && avg_yaw_rate > 0) {
                detected_motor_failure = 3;
            }
        }
    }

// Updates odometry to get latest values
    void Detector::odometryUpdate(int vehicle_odometry_fd, vehicle_odometry_s &odometry) {

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);

        double current_timestamp = odometry.timestamp;

        timestamp = current_timestamp;

        x = odometry.position[0];
        y = odometry.position[1];
        z = odometry.position[2];

        vx = odometry.velocity[0];
        vy = odometry.velocity[1];
        vz = odometry.velocity[2];

        calculateAccelerations(current_timestamp);

        double* rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        double roll_rad = rpy_rad[0];
        double pitch_rad = rpy_rad[1];
        double yaw_rad = rpy_rad[2];

        calculateAttitudeRates(current_timestamp, roll_rad, pitch_rad, yaw_rad);

        roll = roll_rad * (180.0 / M_PI);
        pitch = pitch_rad * (180.0 / M_PI);
        yaw = yaw_rad * (180.0 / M_PI);

        prev_vx = vx;
        prev_vy = vy;
        prev_vz = vz;
        prev_roll_rad = roll_rad;
        prev_pitch_rad = pitch_rad;
        prev_yaw_rad = yaw_rad;
        prev_timestamp = timestamp;

    }
