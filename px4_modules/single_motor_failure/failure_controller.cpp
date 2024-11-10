/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */




#include "failure_controller.h"
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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>

px4::AppState Controller::appState;

int Controller::main(){

    appState.setRunning(true);
    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;
    int vehicle_angular_velocity_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    vehicle_angular_velocity_s angular_vel;
    struct actuator_motors_s act;
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    // File for serving data: 
    
    // FILE *file = fopen("/home/ansh/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/control_logs/log.csv", "w");
    // fprintf(file, "Roll, Pitch, Yaw, Roll_Rate, Pitch_Rate, Yaw_Rate, Roll_Acc, Pitch_Acc, Yaw_Acc, S1, S2, S3, S4, S5, Value_of_Sx\n");

    while (!appState.exitRequested()) {
        
        px4_sleep(0.001);
        //Update odometry
        
        odometryUpdate(vehicle_odometry_fd, odometry);
        angularVelocityUpdate(vehicle_angular_velocity_fd, angular_vel);
        updateSwitchingFuncParams();
        updateSDotDes();

        updateActuatorRotorValues();

        publishMotorCommand(act_pub_fd, act);
        
        // fprintf(file, "%f, %f, %f, %f, %f, %f, %f, %f, %f,%f, %f, %f,%f, %f, %f\n", rpy_plus()[0], rpy_plus()[1], rpy_plus()[2], rpy_rate_plus()[0], rpy_rate_plus()[1], rpy_rate_plus()[2], rpy_acc_plus()[0], rpy_acc_plus()[1], rpy_acc_plus()[2], switching_func_params[0], switching_func_params[1], switching_func_params[2], switching_func_params[3], switching_func_params[4], calculateSwitchingFunc());
        
        // fprintf(file, "%f, %f, %f, %f\n", actuator_rotor_values[0], actuator_rotor_values[1], actuator_rotor_values[2], actuator_rotor_values[3]);
        
        // printf("RPY: %f\t%f\t%f\n", rpy_plus()[0], rpy_plus()[1], rpy_plus()[2]);
        // printf("RPY rates: %f\t%f\t%f\n", rpy_rate_plus()[0], rpy_rate_plus()[1], rpy_rate_plus()[2]);
        // printf("RPY double rates: %f\t%f\t%f\n", rpy_acc_plus()[0], rpy_acc_plus()[1], rpy_acc_plus()[2]);
        // printf("SwitchingFuncParams: S1: %f\tS2: %f\tS3: %f\tS4: %f\tS5: %f\n", switching_func_params[0], switching_func_params[1], switching_func_params[2], switching_func_params[3], switching_func_params[4]);
        
        // printf("Switching Function: %f\n",calculateSwitchingFunc());
    }

    // fclose(file);
    // printf("Data successfully written to data.csv\n");

    return 0;
}

double Controller::calculateDeterminant(double a1, double b1, double c1,
                   double a2, double b2, double c2,
                   double a3, double b3, double c3) {
    return a1 * (b2 * c3 - b3 * c2) - 
           b1 * (a2 * c3 - a3 * c2) + 
           c1 * (a2 * b3 - a3 * b2);
}

double* Controller::systemOfLinearEqs() {
    double* solution = new double[3];
    
    double pitch_rad = pitch * (M_PI / 180.0);
    double yaw_rad = yaw * (M_PI / 180.0);

    double* sdot_exp_f_terms = new double[4];
    sdot_exp_f_terms[0] = (-1.0 * l * switching_func_params[3] / Iyy) + (d * switching_func_params[4] / Izz);
    sdot_exp_f_terms[1] = (l * switching_func_params[3] / Iyy) + (d * switching_func_params[4] / Izz);
    sdot_exp_f_terms[2] = (l * switching_func_params[2] / Ixx) + (-1.0 * d * switching_func_params[4] / Izz);
    sdot_exp_f_terms[3] = (-1.0 * l * switching_func_params[2] / Ixx) + (-1.0 * d * switching_func_params[4] / Izz);
    
    std::vector<std::vector<double>> A(3, std::vector<double>(3, 0));
    // If motor_index == 1
        // S_dot Expression
        A[0][0] = sdot_exp_f_terms[1];
        A[0][1] = sdot_exp_f_terms[2];
        A[0][2] = sdot_exp_f_terms[3];
        // Opposite Working = Same
        A[1][0] = 0.0;
        A[1][1] = 1.0;
        A[1][2] = -1.0;
/*
    // If motor_index == 2
        // S_dot Expression
        A[0][0] = sdot_exp_f_terms[0];
        A[0][1] = sdot_exp_f_terms[2];
        A[0][2] = sdot_exp_f_terms[3];
        // Opposite Working = Same
        A[1][0] = 0.0;
        A[1][1] = 1.0;
        A[1][2] = -1.0;
    // If motor_index == 3
        // S_dot Expression
        A[0][0] = sdot_exp_f_terms[0];
        A[0][1] = sdot_exp_f_terms[1];
        A[0][2] = sdot_exp_f_terms[3];
        // Opposite Working = Same
        A[1][0] = 1.0;
        A[1][1] = -1.0;
        A[1][2] = 0.0;
    // If motor_index == 4
        // S_dot Expression
        A[0][0] = sdot_exp_f_terms[0];
        A[0][1] = sdot_exp_f_terms[1];
        A[0][2] = sdot_exp_f_terms[2];
        // Opposite Working = Same
        A[1][0] = 1.0;
        A[1][1] = -1.0;
        A[1][2] = 0.0;
*/
    // Fnet_vertical = mg
    A[2][0] = vectorAlongNormal(pitch_rad, yaw_rad)[2] / sqrt(vectorAlongNormal(pitch_rad, yaw_rad)[0]*vectorAlongNormal(pitch_rad, yaw_rad)[0] + vectorAlongNormal(pitch_rad, yaw_rad)[1]*vectorAlongNormal(pitch_rad, yaw_rad)[1] + vectorAlongNormal(pitch_rad, yaw_rad)[2]*vectorAlongNormal(pitch_rad, yaw_rad)[2]);
    A[2][1] = vectorAlongNormal(pitch_rad, yaw_rad)[2] / sqrt(vectorAlongNormal(pitch_rad, yaw_rad)[0]*vectorAlongNormal(pitch_rad, yaw_rad)[0] + vectorAlongNormal(pitch_rad, yaw_rad)[1]*vectorAlongNormal(pitch_rad, yaw_rad)[1] + vectorAlongNormal(pitch_rad, yaw_rad)[2]*vectorAlongNormal(pitch_rad, yaw_rad)[2]);
    A[2][2] = vectorAlongNormal(pitch_rad, yaw_rad)[2] / sqrt(vectorAlongNormal(pitch_rad, yaw_rad)[0]*vectorAlongNormal(pitch_rad, yaw_rad)[0] + vectorAlongNormal(pitch_rad, yaw_rad)[1]*vectorAlongNormal(pitch_rad, yaw_rad)[1] + vectorAlongNormal(pitch_rad, yaw_rad)[2]*vectorAlongNormal(pitch_rad, yaw_rad)[2]);
    

    std::vector<double> B(3, 0);
    B[0] = s_dot_des - (
        switching_func_params[0] * (rpy_rate_plus()[0] + rpy_rate_plus()[1] * sin(rpy_plus()[0]) * tan(rpy_plus()[1]) + rpy_rate_plus()[2] * cos(rpy_plus()[0]) * tan(rpy_plus()[1]))
    +   switching_func_params[1] * (rpy_rate_plus()[1] * cos(rpy_plus()[0]) - rpy_rate_plus()[2] * sin(rpy_plus()[0]))
    +   switching_func_params[2] * (-1.0 * Kr * rpy_rate_plus()[0] - rpy_rate_plus()[1] * rpy_rate_plus()[2] * (Izz - Iyy)) / Ixx
    +   switching_func_params[3] * (-1.0 * Kr * rpy_rate_plus()[1] - rpy_rate_plus()[0] * rpy_rate_plus()[2] * (Ixx - Izz)) / Iyy
    +   switching_func_params[4] * (-1.0 * Kr * rpy_rate_plus()[2] - rpy_rate_plus()[0] * rpy_rate_plus()[1] * (Iyy - Ixx)) / Izz
    );
    B[1] = 0;
    B[2] = m * g;

    double D = calculateDeterminant(
                        A[0][0], A[0][1], A[0][2],
                        A[1][0], A[1][1], A[1][2],
                        A[2][0], A[2][1], A[2][2]
                        );
    double Dx = calculateDeterminant(
                        B[0], A[0][1], A[0][2],
                        B[1], A[1][1], A[1][2],
                        B[2], A[2][1], A[2][2]
                        );
    double Dy = calculateDeterminant(
                        A[0][0], B[0], A[0][2],
                        A[1][0], B[1], A[1][2],
                        A[2][0], B[2], A[2][2]
                        );
    double Dz = calculateDeterminant(
                        A[0][0], A[0][1], B[0],
                        A[1][0], A[1][1], B[1],
                        A[2][0], A[2][1], B[2]
                        );

    solution[0] = Dx / D;
    solution[1] = Dy / D;
    solution[2] = Dz / D;

    return solution;
}

void Controller::updateActuatorRotorValues () {
    // If motor_index == 1
    double* solution = systemOfLinearEqs();
    actuator_rotor_values[1] = solution[0];
    actuator_rotor_values[2] = solution[1];
    actuator_rotor_values[3] = solution[2];
/*
    // If motor_index == 2
    actuator_rotor_values[0] = solution[0];
    actuator_rotor_values[2] = solution[1];
    actuator_rotor_values[3] = solution[2];
    // If motor_index == 3
    actuator_rotor_values[0] = solution[0];
    actuator_rotor_values[1] = solution[1];
    actuator_rotor_values[3] = solution[2];
    // If motor_index == 4
    actuator_rotor_values[0] = solution[0];
    actuator_rotor_values[1] = solution[1];
    actuator_rotor_values[2] = solution[2];
*/
}

double Controller::sgn (double input){
    return (input > 0) ? (1.0) : ((input < 0) ? -1.0 : 0.0);
}

double Controller::calculateSwitchingFunc (){
    double switching_func = 
        switching_func_params[0] * rpy_plus()[0]
    +   switching_func_params[1] * rpy_plus()[1]
    +   switching_func_params[2] * rpy_rate_plus()[0]
    +   switching_func_params[3] * rpy_rate_plus()[1]
    +   switching_func_params[4] * rpy_rate_plus()[2]
    +   w;
    return switching_func;  
}

double* Controller::rpy_plus(){
    double roll_rad = roll * (M_PI / 180.0);
    double pitch_rad = pitch * (M_PI / 180.0);
    double yaw_rad = yaw * (M_PI / 180.0);
    
    double* rpy_plus = new double[3];    

    rpy_plus[0] = cos(M_PI / 4) * roll_rad + sin(M_PI / 4) * pitch_rad;
    rpy_plus[1] = cos(M_PI / 4) * pitch_rad - sin(M_PI / 4) * roll_rad;
    rpy_plus[2] = yaw_rad;

    return rpy_plus;     
}

double* Controller::rpy_rate_plus(){
    double roll_rate_rad = roll_rate * (M_PI / 180.0);
    double pitch_rate_rad = pitch_rate * (M_PI / 180.0);
    double yaw_rate_rad = yaw_rate * (M_PI / 180.0);

    double* rpy_rate_plus = new double[3];    

    rpy_rate_plus[0] = cos(M_PI / 4) * roll_rate_rad + sin(M_PI / 4) * pitch_rate_rad;
    rpy_rate_plus[1] = cos(M_PI / 4) * pitch_rate_rad - sin(M_PI / 4) * roll_rate_rad;
    rpy_rate_plus[2] = -1.0 * yaw_rate_rad;

    return rpy_rate_plus;     
}


double* Controller::rpy_acc_plus(){
    double* rpy_acc_plus = new double[3];    
    
    rpy_acc_plus[0] = cos(M_PI / 4) * roll_acc_rad + sin(M_PI / 4) * pitch_acc_rad;
    rpy_acc_plus[1] = cos(M_PI / 4) * pitch_acc_rad - sin(M_PI / 4) * roll_acc_rad;
    rpy_acc_plus[2] = -1.0 * yaw_acc_rad;

    return rpy_acc_plus;     
}

double* Controller::rpyDotFunctions(){
    double* state_dot = new double[3];
    state_dot[0] = rpy_rate_plus()[0] + rpy_rate_plus()[1] * sin(rpy_plus()[0]) * tan(rpy_plus()[1]) + rpy_rate_plus()[2] * cos(rpy_plus()[0]) * tan(rpy_plus()[1]);
    state_dot[1] = rpy_rate_plus()[1] * cos(rpy_plus()[0]) - rpy_rate_plus()[2] * sin(rpy_plus()[0]);
    state_dot[2] = (rpy_rate_plus()[1] * sin(rpy_plus()[0]) + rpy_rate_plus()[2] * cos(rpy_plus()[0]))/cos(rpy_plus()[1]);
    return state_dot;
}

double* Controller::vectorAlongNormal (double pitch_rad, double yaw_rad){
    double* vector_along_normal = new double[3];
    vector_along_normal[0] = -1.0 * cos(yaw_rad) * sin(pitch_rad);
    vector_along_normal[1] = -1.0 * sin(yaw_rad) * sin(pitch_rad);
    vector_along_normal[2] = cos(pitch_rad);
    return vector_along_normal;
}





// State vector used - [phi, theta, p, q, r]

// double* Controller::stabilitySurface(double yaw_rate_des){
//     double* switching_func = new double[5];
//     double Ixx = 1, Iyy = 1, w = 0.5;


//     switching_func[2] = Ixx;
//     switching_func[3] = Iyy;
//     switching_func[4] = -w / yaw_rate_des;
//     switching_func[1]=  (roll_rad / (roll_rate_rad *+ pitch_rad - roll_rad * pitch_rate_rad)) * ((switching_func[2] * (roll_rate* M_PI / 180)) + (switching_func[3]/(pitch_rate* M_PI / 180)) + (switching_func[4]/(yaw_rate* M_PI / 180)) + w) / (roll * M_PI / 180);
//     switching_func[0] = -switching_func[1] - ((switching_func[2] * roll_rate/(roll_rad)) + (switching_func[3]*pitch_rate/(roll_rad)) + (switching_func[4] * yaw_rate/(roll_rad* M_PI / 180)) + w);




//     /* 

//     S = 
//     (roll_rad) * switch_func[0]
//     + 
//     (pitch_rad) * switch_func[1]
//     +
//     (roll_rate_rad) * switch_func[2]
//     +
//     (pitch_rate_rad) * switch_func[3]
//     +
//     (yaw_rate_rad) * switch_func[4];


//     S_dot = - eta * sin(S);


//     A = 
//     (roll_rate_rad + pitch_rate_rad * sin(roll_rad) * tan(pitch_rad) + yaw_rate_rad * cos(roll_rad) * tan(pitch_rad)) * switch_func[0]
//     + 
//     (pitch_rate_rad * cos(roll_rad) - yaw_rate_rad * sin(roll_rad)) * switch_func[1]
//     +
//     ((-1.0 * Kr * roll_rate_rad - pitch_rate_rad * yaw_rate_rad * (Izz - Iyy)) / Ixx) * switch_func[2]
//     +
//     ((-1.0 * Kr * pitch_rate_rad - roll_rate_rad * yaw_rate_rad * (Ixx - Izz)) / Iyy) * switch_func[3]
//     +
//     ((-1.0 * Kr * yaw_rate_rad - roll_rate_rad * pitch_rate_rad * (Iyy - Ixx)) / Izz) * switch_func[4]



//     B = 
//     (0) * switch_func[0]
//     + 
//     (0) * switch_func[1]
//     +
//     (l * (f4 - f2) * switch_func[2] / Ixx)
//     +
//     (l * (f3) * switch_func[3] / Iyy)
//     +
//     (d * (f3 - f4 - f2) * switch_func[4] / Izz)


//     (S_dot - A) = B

//     */

//     return switching_func;
// }

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

void Controller::calculateAccelerations(int timestmp) {
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

double Controller::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double Controller::calculateAngularRate(double current, double previous, double dt) {
    double diff = normalizeAngle(current - previous);
    return dt > 0 ? diff / dt : 0.0;
}

void Controller::calculateAttitudeRates(int timestmp, double roll_rad, double pitch_rad, double yaw_rad) {
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



void Controller::odometryUpdate(int vehicle_odometry_fd, vehicle_odometry_s &odometry) {
    
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
// Actuator Rotor Values:
    prev_vx = vx;
    prev_vy = vy;
    prev_vz = vz;
    prev_roll_rad = roll_rad;
    prev_pitch_rad = pitch_rad;
    prev_yaw_rad = yaw_rad;
    prev_timestamp = timestamp;

}

void Controller::angularVelocityUpdate(int vehicle_angular_velocity_fd, vehicle_angular_velocity_s &angular_vel) {
    
    orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_fd, &angular_vel);

    roll_acc_rad = (double)angular_vel.xyz_derivative[0];
    pitch_acc_rad = (double)angular_vel.xyz_derivative[1];
    yaw_acc_rad = (double)angular_vel.xyz_derivative[2];

}

void Controller::updateSwitchingFuncParams() {
    
    switching_func_params[2] = 0 * Ixx;
    switching_func_params[3] = Iyy;
    switching_func_params[4] = -1.0 * w / yaw_rate_des;
    // switching_func_params[1] = (rpy_plus()[0]/(rpyDotFunctions()[0] * rpy_plus()[1] - rpyDotFunctions()[1] * rpy_plus()[0])) * (switching_func_params[2] * (rpy_acc_plus()[0] - rpy_rate_plus()[0] * (rpyDotFunctions()[0]/rpy_plus()[0])) + switching_func_params[3] * (rpy_acc_plus()[1] - rpy_rate_plus()[1] * (rpyDotFunctions()[0]/rpy_plus()[0]))+switching_func_params[4] * (rpy_acc_plus()[2] - rpy_rate_plus()[2] * (rpyDotFunctions()[0]/rpy_plus()[0])) );
    // switching_func_params[0] = -1.0 * (
    //         switching_func_params[1] * rpy_plus()[1]
    //     +   switching_func_params[2] * rpy_rate_plus()[0]
    //     +   switching_func_params[3] * rpy_rate_plus()[1]
    //     +   switching_func_params[4] * rpy_rate_plus()[2]
    //     + w
    // ) / rpy_plus()[0];
    switching_func_params[0] = 1.0;
    switching_func_params[1] = 1.0;

}

void Controller::updateSDotDes() {
    s_dot_des = -1.0 * eta * sgn(calculateSwitchingFunc());
}

void Controller::publishMotorCommand(orb_advert_t act_pub_fd, actuator_motors_s act){
    for(int i = 0; i<12; i++){
        if(i>=4)
            continue;
        else if(i==0) // Motor Failed
            continue;
        else{
            if(actuator_rotor_values[i] > 0 || actuator_rotor_values[i] < 0)
                act.control[i] = actuator_rotor_values[i];
            else
                act.control[i] = (float)nan("1");       
        }
    }
    orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
    // PX4_INFO("Actuator_Motor_Command_Sent");
}