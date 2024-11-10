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
 * @file failure_controller.cpp
 *  Implementation of Controller class
 *  Controls in case of motor failure
 */




#include "failure_controller.h"
#include "lqr.h"
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
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>


using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState Controller::appState;  /* track requests to terminate app */

int Controller::main(int detected_motor){

    appState.setRunning(true);

    int detected_motor_index = detected_motor - 1;
    int opposite_motor_index  = (detected_motor_index < 2) ? (1 - detected_motor_index) : (5 - detected_motor_index);
    int diagonally_working_1 =  (detected_motor_index < 2) ? 2 : 0;
    int diagonally_working_2 =  (detected_motor_index < 2) ? 3 : 1;


    actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    double* rpy_rad;

    // double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double* normal;
    double n_x;
    double n_y;
    // double n_z;


    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;

    double u1;
    double u2;
    double f1;                  //      Anticlockwise first motor to the failed
    double f2;                  //      Opposite to failed modtor
    double f3;                  //      Clockwise first motor to the failed

    LQR lqr;

    Vector current_state;
    Vector inputs;

    while (!appState.exitRequested()) {

        // px4_sleep(0.5);
        usleep(1000);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);


        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        // roll_rad = rpy_rad[0];
        pitch_rad = rpy_rad[1];
        yaw_rad = rpy_rad[2];

        roll_rate = odometry.angular_velocity[0];
        pitch_rate = odometry.angular_velocity[1];
        yaw_rate = odometry.angular_velocity[2];

        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];

        normal = vectorAlongNormal(pitch_rad, yaw_rad);
        n_x = normal[0];
        n_y = normal[1];
        // n_z = normal[2];

        current_state = {p_s, q_s, n_x, n_y};        // Get current state

        inputs = lqr.getControlInputs(current_state, detected_motor);     // Apply LQR controller


        u1 = inputs[0];
        u2 = inputs[1];

        // Thrust equations after solving input relation

        f2 = u2 + 8.83/2;
        f1 = ( 8.83 - u2 > 0) ? 8.83 - u2 : 0.0 ;
        f3 = (8.83 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 8.83 + (u1 - u2)/2  ;

        // Apply control for different cases of failure
        // Use only two opposite motors first to accelerate to desired yaw rate and then apply control

        if( abs(yaw_rate) > 5 && odometry.position[2] < -2.5f){


        if(detected_motor == 1){
        act.control[0] = (float)nan("1");
        act.control[1] = f2/ 14.56;
        act.control[2] = f1/14.56;
        act.control[3] = f3/14.56;

        }

        else if (detected_motor == 2)
        {
            act.control[0] = f2/ 14.56;
            act.control[1] = (float)nan("1");
            act.control[2] = f3/14.56;
            act.control[3] = f1/14.56;

        }

         else if (detected_motor == 3)
        {
            act.control[0] = f3/14.56;
            act.control[1] = f1/14.56;
            act.control[2] = (float)nan("1");
            act.control[3] = f2/ 14.56;

        }

         else if (detected_motor == 4)
        {
            act.control[0] = f1/14.56;
            act.control[1] = f3/14.56;
            act.control[2] = f2/ 14.56;
            act.control[3] = (float)nan("1");

        }

        printf("u1 = %f, u2 = %f , f1 = %f , f2 = %g , f3 = %f yaw_rate = %f\n",u1, u2 ,f1, f2, f3, yaw_rate);

        }

        else if(abs(yaw_rate) < 5 && odometry.position[2] < -2.5f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = 0.7;
        act.control[diagonally_working_2] = 0.7;
        printf("Two motors only , yaw_rate = %f\n",yaw_rate);
        }

        else if(odometry.position[2] > -2.5f)
        {
        act.control[0] = 0;
        act.control[1] = 0;
        act.control[2] = 0;
        act.control[3] = 0;
        }

        // printf("Alt = %f \n" , (double)odometry.position[2]);

        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);



    }



    return 0;
}


double* Controller::quaternionToRPY (double qw, double qx, double qy, double qz){

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

// Calculates unit vector along normal
double* Controller::vectorAlongNormal (double pitch_rad, double yaw_rad){
    double* vector_along_normal = new double[3];
    vector_along_normal[0] = -1.0 * cos(yaw_rad) * sin(pitch_rad);
    vector_along_normal[1] = -1.0 * sin(yaw_rad) * sin(pitch_rad);
    vector_along_normal[2] = cos(pitch_rad);
    return vector_along_normal;
}

// Converts rpy_rate to plus configuration
double* Controller::rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate){
    double roll_rate_rad = roll_rate * (M_PI / 180.0);
    double pitch_rate_rad = pitch_rate * (M_PI / 180.0);
    double yaw_rate_rad = yaw_rate * (M_PI / 180.0);

    double* rpy_rate_plus = new double[3];

    rpy_rate_plus[0] = cos(M_PI / 4) * roll_rate_rad - sin(M_PI / 4) * pitch_rate_rad;
    rpy_rate_plus[1] = - cos(M_PI / 4) * pitch_rate_rad - sin(M_PI / 4) * roll_rate_rad;
    rpy_rate_plus[2] = -1.0 * yaw_rate_rad;

    return rpy_rate_plus;
}
