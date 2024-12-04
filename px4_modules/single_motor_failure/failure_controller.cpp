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
#include <uORB/topics/drone_state.h>

#include <thread>
#include <mutex>




using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState Controller::appState;  /* track requests to terminate app */


// Shared state structure, for accessing of variables between outer and inner loops

struct State {
    // double desired_position = 0.0;  // Outer loop target
    // double current_position = 0.0; // System state
    double desired_velocity = 0.0; // Inner loop target
    double current_velocity = 0.0; // System state

    int check_flag = 0;
    int detected_motor;

    double alt_error;////

    double nx_des = 0.0;
    double ny_des = 0.0;
    double nz_des = 1.0;

    double fsum = 11.0;
};

State shared_state;
std::mutex shared_state_mutex; // Mutex to protect shared state while updating


// Outer loop: Position control based on PD controller
void Controller::outerLoop() {

    // Subscribing odometry values
    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    Vector desired_position = {0.0 , 0.0 , -11.0};    // Desired position
    Vector current_position = {0.0 , 0.0 , 0.0};      // Current position
    Vector pos_deviation = {0.0 , 0.0 , 0.0};         // Position deviation

    Vector desired_accel_plus  = {0.0 , 0.0 , 0.0};   // Desired acceleration in plus configuration
    

    // Desired Axis of Rotation (for x, y control) 
    double nx_des = 0.0;    
    double ny_des = 0.0;
    double nz_des = 1.0;

    // Total thrust for z control
    double fsum = 19.0;
    double alt_error;   // Altitude error
    
    // Propotional constants for x,y,z
    Vector Kp = {0.0, 0.0, 4.0};
    // Vector Kp = {0, 0, 4};
    // Vector Kp = {0.05, 0.05, 5};


    // Derivative constants for x,y,z
    Vector Kd = {0.0, 0.0, 0.0};



    // Plus configuration desired and current  
    Vector desired_position_plus = {0.0 , 0.0 , 0.0};
    Vector current_velocity_plus = {0.0 , 0.0 , 0.0};
    Vector current_position_plus = {0.0 , 0.0 , 0.0};

    double prev_desired_position_z = -11.0;
    // Previous desired position, for adapting in case the alt error is too high 


    while (!appState.exitRequested()) {


        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);



        current_position[0] = (double)odometry.position[0];
        current_position[1] = (double)odometry.position[1];
        current_position[2] = (double)odometry.position[2];

        desired_position[2] = current_position[2] - 1.0;
        desired_position[2] = (desired_position[2] < prev_desired_position_z) ? desired_position[2] : desired_position[2];
        // desired_position[2] = (desired_position[2] < prev_desired_position_z) ? desired_position[2] : prev_desired_position_z;
        prev_desired_position_z = desired_position[2];

        current_position_plus = vectorFRD2Plus(current_position);
        desired_position_plus = vectorFRD2Plus(desired_position);
        current_velocity_plus = vectorFRD2Plus({(double)odometry.velocity[0], (double)odometry.velocity[1], (double)odometry.velocity[2]});


        pos_deviation[0] = current_position_plus[0] - desired_position_plus[0];
        pos_deviation[1] = current_position_plus[1] - desired_position_plus[1];
        pos_deviation[2] = current_position_plus[2] - desired_position_plus[2];


        // PD control Law

        desired_accel_plus[0] = -Kd[0]*(double)current_velocity_plus[0] - Kp[0]*pos_deviation[0];
        desired_accel_plus[1] = -Kd[1]*(double)current_velocity_plus[1] - Kp[1]*pos_deviation[1];
        desired_accel_plus[2] = -Kd[2]*(double)current_velocity_plus[2] - Kp[2]*pos_deviation[2];

        desired_accel_plus[2]+= 9.8; // d_des - g


        // printf("Desired Accel = %f %f %f \n", desired_accel_plus[0], desired_accel_plus[1], desired_accel_plus[2]);

        desired_accel_plus[2] = (desired_accel_plus[2] < 0) ? 0 : desired_accel_plus[2];

        // Desired Axis of Rotation (for x, y control) 
        Vector ndes = desired_accel_plus;
        
        ndes[0] = ndes[0] * (1.535 / 0.982);
        ndes[1] = ndes[1] * (1.535 / 0.982);
        ndes[2] = ndes[2] * (1.535 / 0.982);
        
        // printf("desired n before normalizing = %f %f %f \n", ndes[0], ndes[1], ndes[2]);

        fsum = calculateNorm(ndes);

        ndes[0] = ndes[0] / fsum;
        ndes[1] = ndes[1] / fsum;
        ndes[2] = ndes[2] / fsum;

        nx_des = ndes[0];
        ny_des = ndes[1];
        nz_des = ndes[2];

        // printf("desired AOR = %f %f %f \n", nx_des, ny_des, nz_des);

        alt_error = pos_deviation[2];

        {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        state_data.nx_aor = nx_des;
        state_data.ny_aor = ny_des;
        state_data.nz_aor = nz_des;
        shared_state.nx_des = nx_des;
        shared_state.ny_des = ny_des;
        shared_state.nz_des = nz_des;
        shared_state.fsum = fsum;

        shared_state.alt_error = alt_error;
        }

        // Unlock automatically when lock_guard goes out of scope
        usleep(2000); // Sleep for 2000 us
    }



}

// Inner loop: Attitude control
void Controller::innerLoop() {
    
    int detected_motor;
     {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        detected_motor = shared_state.detected_motor;
    }


    int detected_motor_index = detected_motor - 1;
    int opposite_motor_index  = (detected_motor_index < 2) ? (1 - detected_motor_index) : (5 - detected_motor_index);
    int diagonally_working_1 =  (detected_motor_index < 2) ? 2 : 0;
    int diagonally_working_2 =  (detected_motor_index < 2) ? 3 : 1;


    actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    int vehicle_attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_attitude_s attitude;

    double* rpy_rad;

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double* normal;
    double n_x;
    double n_y;
    double n_z;


    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;
    double r_s;

    double u1;
    double u2;
    double u3;
    double f1;                  //      Anticlockwise first motor to the failed
    double f2;                  //      Opposite to failed modtor
    double f3;                  //      Clockwise first motor to the failed

    LQR lqr;

    Vector current_state;       //  p, q, r, nx, ny
    Vector inputs;




    while (!appState.exitRequested()) {

        // px4_sleep(0.5);
        usleep(100);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
        orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_fd, &attitude);


        // Roll, Pitch, Yaw
        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        roll_rad = rpy_rad[0];
        pitch_rad = rpy_rad[1];
        yaw_rad = rpy_rad[2];
        
        //Roll, Pitch, Yaw rates
        roll_rate = odometry.angular_velocity[0];
        pitch_rate = odometry.angular_velocity[1];
        yaw_rate = odometry.angular_velocity[2];


        // Conversion of rates from x to plus configuration
        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];
        r_s = rpy_plus_rate[2];


        // Inverse quaternion for calculation of nx, ny, nz
        double* q_inverse = new double[4];
        q_inverse[0] = attitude.q[0];
        q_inverse[1] = -1.0 * (double)attitude.q[1];
        q_inverse[2] = -1.0 * (double)attitude.q[2];
        q_inverse[3] = -1.0 * (double)attitude.q[3];
        // q_inverse[0] = odometry.q[0];
        // q_inverse[1] = -1.0 * (double)odometry.q[1];
        // q_inverse[2] = -1.0 * (double)odometry.q[2];
        // q_inverse[3] = -1.0 * (double)odometry.q[3];

        {
            std::lock_guard<std::mutex> lock(shared_state_mutex);
            double* axis_of_rotation = new double [3];
            axis_of_rotation[0] = shared_state.nx_des;
            axis_of_rotation[1] = shared_state.ny_des;
            axis_of_rotation[2] = shared_state.nz_des;
            normal = AorBodyFrame(q_inverse, axis_of_rotation);
        }

        // Components of Axis of Rotation resolved in the body frame
        n_x = normal[0];
        n_y = normal[1];
        n_z = normal[2];

     
        current_state = {p_s, q_s, r_s, n_x, n_y};        // Get current state
      

        inputs = lqr.getControlInputs(current_state, detected_motor);     // Apply LQR controller




        // Inputs
        u1 = inputs[0];
        u2 = inputs[1];
        u3 = inputs[2];


        // Thrust equations after solving input relation

        //////////////////////////////
        f1 = u1 + shared_state.fsum * (1.28 / (1 + (2 * 1.28)));
        f2 = u2 + shared_state.fsum * (1 / (1 + (2 * 1.28)));
        f3 = u3 + shared_state.fsum * (1.28 / (1 + (2 * 1.28)));

        (f1 < 0) ? f1 = 14.13 : (f1 > 14.13) ? f1 = 14.13 : f1 = f1;
        (f2 < 0) ? f2 = 14.13 : (f2 > 14.13) ? f2 = 14.13 : f2 = f2;
        (f3 < 0) ? f3 = 14.13 : (f3 > 14.13) ? f3 = 14.13 : f3 = f3;
        //////////////////////////////

      
  
        // Apply control for different cases of failure
        // Use only two opposite motors first to accelerate to desired yaw rate and then apply control

        if( abs(yaw_rate) > 5 && odometry.position[2] < -2.5f){

        
        // Three motor control when yaw rate is > 5
        // For the case when motor 1 is failed 
        act.control[0] = (float)nan("1");
        act.control[1] = f2/ 14.13;
        act.control[2] = f1/14.13;
        act.control[3] = f3/14.13;

   

        state_data.timestamp = odometry.timestamp;
        state_data.u1 = u1;
        state_data.u2 = u2;
        state_data.u3 = u3;
        state_data.f1 = f1;
        state_data.f2 = f2;
        state_data.f3 = f3;
        state_data.fsum = shared_state.fsum;
        state_data.alt_error = shared_state.alt_error;
        state_data.q_r = (double)attitude.q[0];
        state_data.q_i = (double)attitude.q[1];
        state_data.q_j = (double)attitude.q[2];
        state_data.q_k = (double)attitude.q[3];
        // state_data.q_r = (double)odometry.q[0];
        // state_data.q_i = (double)odometry.q[1];
        // state_data.q_j = (double)odometry.q[2];
        // state_data.q_k = (double)odometry.q[3];
        state_data.p_s = p_s;
        state_data.q_s = q_s;
        state_data.r_s = yaw_rate;
        state_data.n_x = n_x;
        state_data.n_y = n_y;
        state_data.n_z = n_z;
        state_data.roll_rad = roll_rad;
        state_data.pitch_rad = pitch_rad;
        state_data.yaw_rad = yaw_rad;
        state_data.x = (double)odometry.position[0];
        state_data.y = (double)odometry.position[1];
        state_data.z = (double)odometry.position[2];

        // printf("-----------------\n");
        // printf("u1 = %f, u2 = %f, u3 = %f, f1 = %f , f2 = %f , f3 = %f , fsum = %f, alt error = %f  \n",u1, u2, u3,f1, f2, f3, shared_state.fsum, shared_state.alt_error);
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
        // printf("x = %f, y = %f, z = %f \n", (double)odometry.position[0],(double)odometry.position[1],(double)odometry.position[2]);
        // printf("nx_des = %f, ny_des = %f, ny_des = %f", shared_state.nx_des, shared_state.ny_des, shared_state.nz_des);
        // printf("\n\n");

        // printf("u1 = %f, u2 = %f, u3 = %f , f1 = %f , f2 = %f , f3 = %f , fsum = %f, alt error = %f yaw_rate = %f   \n",u1, u2 ,u3, f1, f2, f3, f1+f2+f3, shared_state.alt_error, yaw_rate );
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, r_s, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
        // printf("x = %f, y = %f, z = %f  signal = %f \n", (double)odometry.position[0],(double)odometry.position[1],(double)odometry.position[2],signal);
        // printf("nx_des = %f, ny_des = %f, nz_des = %f");

        }


        // Two motor control 
        else if(abs(yaw_rate) < 5 && odometry.position[2] < -2.5f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = 0.67;
        act.control[diagonally_working_2] = 0.67;
        // printf("Two motors only , yaw_rate = %f\n",yaw_rate);
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
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
        orb_publish(ORB_ID(drone_state), drone_state_publisher, &state_data);

        {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        shared_state.nx_des = 0.0;
        shared_state.ny_des = 0.0;
        shared_state.nz_des = 1.0;
        // shared_state.fsum = 23;
        }

    }
}

int Controller::main(int detected_motor){

    appState.setRunning(true);

     {


        // Mutex 
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state

        shared_state.detected_motor = detected_motor;
    }

    std::thread outer_thread(&Controller::outerLoop, this);
    std::thread inner_thread(&Controller::innerLoop, this);


    outer_thread.join();
    inner_thread.join();


            // Let the simulation run for 5 seconds
        // sleep(5);

        // // Detach threads (graceful shutdown logic can be added here)
        // outer_thread.detach();
        // inner_thread.detach();


    return 0;
}

// Quaternion to roll, pitch, yaw
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
// Hamiltonian product for vectorQuaternionTransformation
double* Controller::hamiltonianProduct (double* q1, double* q2){
    double* hamiltonian_product = new double[4];
    hamiltonian_product[0] = (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    hamiltonian_product[1] = (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    hamiltonian_product[2] = (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    hamiltonian_product[3] = (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
    return hamiltonian_product;
}

// For Axis of Rotation calculation in body frame
double* Controller::vectorQuaternionTransformation (double* q, double* v){
    double* new_v = new double[3];
    double* v_q  = new double[4];
    double* new_v_q = new double[4];
    double* q_dash = new double[4];
    v_q[0] = 0;
    v_q[1] = v[0];
    v_q[2] = v[1];
    v_q[3] = v[2];
    q_dash[0] = q[0];
    q_dash[1] = -1.0 * q[1];
    q_dash[2] = -1.0 * q[2];
    q_dash[3] = -1.0 * q[3];

    new_v_q = hamiltonianProduct(hamiltonianProduct(q, v_q), q_dash);
    // new_v_q = hamiltonianProduct(hamiltonianProduct(q_dash, v_q), q);

    new_v[0] = new_v_q[1];
    new_v[1] = new_v_q[2];
    new_v[2] = new_v_q[3];

    delete[] v_q;
    delete[] new_v_q;
    delete[] q_dash;

    return new_v;
}

double* Controller::vectorFRD2Plus (double* v){
    double* new_v = new double[3];
    double transformationMatrix[3][3] =
    {
        {(1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {-1.0 * (1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {0.0, 0.0, -1.0}

    };
    new_v[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    new_v[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    new_v[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];

    return new_v;
}

Vector Controller::vectorFRD2Plus (Vector v){
    Vector new_v(3);
    double transformationMatrix[3][3] =
    {
        {(1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {-1.0 * (1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {0.0, 0.0, -1.0}

    };
    new_v[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    new_v[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    new_v[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];

    return new_v;
}

// Calculates unit vector along normal
double* Controller::AorBodyFrame (double* q, double* N){

    double* vector_along_normal = new double[3];
    double* N_frd = new double[3];
    N_frd = vectorFRD2Plus(N);
    vector_along_normal[0] = N_frd[0];
    vector_along_normal[1] = N_frd[1];
    vector_along_normal[2] = N_frd[2];

    vector_along_normal = vectorFRD2Plus(vectorQuaternionTransformation(q, vector_along_normal));

    return vector_along_normal;
}
// double* Controller::AorBodyFrame (double* q){
//     double* vector_along_normal = new double[3];
//     vector_along_normal[0] = 0.0;
//     vector_along_normal[1] = 0.0;
//     vector_along_normal[2] = -1.0;

//     vector_along_normal = vectorFRD2Plus(vectorQuaternionTransformation(q, vector_along_normal));

//     return vector_along_normal;
// }

// Converts rpy_rate to plus configuration
double* Controller::rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate){
    double roll_rate_rad = roll_rate;// * (M_PI / 180.0);
    double pitch_rate_rad = pitch_rate;// * (M_PI / 180.0);
    double yaw_rate_rad = yaw_rate;// * (M_PI / 180.0);

    double* rpy_rate_plus = new double[3];

    rpy_rate_plus[0] = cos(M_PI / 4) * roll_rate_rad - sin(M_PI / 4) * pitch_rate_rad;
    rpy_rate_plus[1] = - cos(M_PI / 4) * pitch_rate_rad - sin(M_PI / 4) * roll_rate_rad;
    rpy_rate_plus[2] = -1.0 * yaw_rate_rad;

    return rpy_rate_plus;
}

// Function to calculate the inverse of the rotation matrix (transpose of R)
Matrix Controller::calculateRotationMatrixInverse(double roll, double pitch, double yaw) {

    Matrix inverse(3, std::vector<double>(3));
    // Precompute sines and cosines of the angles
    double c_roll = cos(roll);
    double s_roll = sin(roll);
    double c_pitch = cos(pitch);
    double s_pitch = sin(pitch);
    double c_yaw = cos(yaw);
    double s_yaw = sin(yaw);

    // Inverse of the rotation matrix (transpose)
    inverse[0][0] = c_pitch * c_yaw;
    inverse[0][1] = c_pitch * s_yaw;
    inverse[0][2] = -s_pitch;

    inverse[1][0] = c_yaw * s_pitch * s_roll - c_roll * s_yaw;
    inverse[1][1] = s_yaw * s_pitch * s_roll + c_roll * c_yaw;
    inverse[1][2] = c_pitch * s_roll;

    inverse[2][0] = c_roll * c_yaw * s_pitch + s_roll * s_yaw;
    inverse[2][1] = c_roll * s_pitch * s_yaw - s_roll * c_yaw;
    inverse[2][2] = c_pitch * c_roll;

    return inverse;
}



// Adds two vectors element-wise
Vector Controller::vectorAdd(const Vector &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

// Multiplies a matrix by a vector
Vector Controller::multiply(const Matrix &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            result[i] += a[i][j] * b[j];
        }
    }
    return result;
}

// Norm 
double Controller::calculateNorm(Vector vec) {
    double sum = 0.0;
    for (double component : vec) {
        sum += component * component;
    }
    return std::sqrt(sum);
}
