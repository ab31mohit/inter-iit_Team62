/****************************************************************************
 *
 *   Copyright (C) 2024 Inter-IIT Team 62. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
 /**#include <uORB/topics/sensor_accel.h>
 */


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <signal.h>
#include <stdbool.h>
#include <atomic>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_odometry.h>
#include <time.h>
#include <sys/time.h>

#include <pthread.h>


// __EXPORT int gazebo_classic_main(int argc, char *argv[]);
extern "C" __EXPORT int gazebo_classic_main(int argc, char *argv[]);


// Shared state structure
struct State {
    double nx_des;
    double ny_des;
	double nz_des;
	// double az_des;

	// double Th;
};

struct State shared_state;
pthread_mutex_t shared_state_mutex; // Mutex to protect shared state


double* quaternion_to_rad_gazebo_classic(double q[4]) {
    double x = q[0];
    double y = q[1];
    double z = q[2];
    double w = q[3];

    double t0 = (double)2.0 * (w * x + y * z);
    double t1 = (double)1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(t0, t1);

    double t2 = (double)2.0 * (w * y - z * x);
    t2 = t2 > (double)1.0 ? (double)1.0 : t2;
    t2 = t2 < (double)-1.0 ? (double)-1.0 : t2;
    double pitch = asin(t2);

    double t3 = (double)2.0 * (w * z + x * y);
    double t4 = (double)1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(t3, t4);

    double* rpy = (double*)malloc(3 * sizeof(double));
    if (yaw < 0) {
        yaw += 2 * M_PI;  // 6.28... is 2π
    }

    // Return in the order [yaw, pitch, roll] as per your Python function
    rpy[0] = yaw;
    rpy[1] = pitch;
    rpy[2] = roll;

    return rpy;
};

// Function to calculate the norm of a vector
double calculateNorm(const double vec[3]) {
    double sum = 0.0;

    // Iterate through the vector and calculate the sum of squares
    for (int i = 0; i < 3; i++) {
        sum += vec[i] * vec[i];
    }

    // Return the square root of the sum
    return sqrt(sum);
};



// Outer loop: Position control
void* outerLoop(void*) {

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    struct vehicle_odometry_s odometry;

    double desired_position[] = {0.0 , 0.0 , 10.0};   // in FRD Frame of drone
    double current_position[] = {0.0 , 0.0 , 0.0};
    double pos_deviation[] = {0.0 , 0.0 , 0.0};
    double desired_accel[]  = {0.0 , 0.0, 0.0};
	// double current_velocity[] = {0.0 , 0.0 , 0.0};
	double norm_desired_accel;
	double accel_z;

    while (true) {

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);

        current_position[0] = (double)odometry.position[0];
        current_position[1] = (double)odometry.position[1];
        current_position[2] = (double)odometry.position[2];

        pos_deviation[0] = current_position[0] - desired_position[0];   // e_X
        pos_deviation[1] = current_position[1] - desired_position[1];   // e_Y
        pos_deviation[2] = current_position[2] - desired_position[2];   // e_Z

		double alpha = 0.5;
		accel_z = alpha * desired_position[2];  // a_z = alpha * e_Z

		double k_p = 0.5;
		double g = 9.8;
		desired_accel[0] = - k_p * pos_deviation[0];  // -(k_p)(e_X)
		desired_accel[1] = - k_p * pos_deviation[1];  // -(k_p)(e_Y)
		desired_accel[2] = - k_p * (g - accel_z);
	    
		norm_desired_accel = calculateNorm(desired_accel);
	    
		double ndes[] = {0.0 , 0.0, 0.0};
		ndes[0] = desired_accel[0] / norm_desired_accel;
		ndes[1] = desired_accel[1] / norm_desired_accel;
		ndes[2] = desired_accel[2] / norm_desired_accel;
		// ndes[2] = sqrt(1 - (ndes[0]*ndes[0] + ndes[1]*ndes[1]));
        
		pthread_mutex_lock(&shared_state_mutex);
        
        shared_state.nx_des = ndes[0];
        shared_state.ny_des = ndes[1];
		shared_state.nz_des = ndes[2];

		// Unlock the mutex
        pthread_mutex_unlock(&shared_state_mutex);

        // Unlock automatically when lock_guard goes out of scope
        usleep(200); // Sleep for 100 ms
    }

}


void* inner_loop(void*)
// int gazebo_classic_main(int argc, char *argv[])
{

	//Initializing Publisher
	struct actuator_motors_s act;
	// int motor_index_1 = 0;
	// int motor_index_2 = 1;
	memset(&act,0,sizeof(act));
	orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

	//Initializing GPS Subscriber
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s raw;

	//Initializing Attitude Subscriber
	int sensor_sub_2_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_attitude_s raw_att;

	//Initializing Gyro Subscriber
	int sensor_sub_gy_fd = orb_subscribe(ORB_ID(sensor_gyro));
	struct sensor_gyro_s raw_gyro;

	//Initializing Imu acceleration Subscriber
	int sensor_sub_accel_fd = orb_subscribe(ORB_ID(sensor_accel));
	struct sensor_accel_s raw_accel;

	int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    struct vehicle_odometry_s odometry;

	double alt_des = 10.0;

	double nx_curr;
	double ny_curr;
	double nz_curr;
	double nxdes;
	double nydes;
	double Th;
	double p;
	double m = 1.5;
	double g = 9.81;
	// double azdes;

	while(1){
		// px4_sleep(0.005);
		//Reading GPS Data
		orb_copy(ORB_ID(vehicle_global_position), sensor_sub_fd, &raw);
		double alt = (double)raw.alt;
		alt = ceil(alt * 100.0) / 100.0;
		alt = alt - (double)488.08; //Calculating Altitude Relative to Ground Pose
		alt = fmax(0,alt);
		// double yaw_des = (double)raw.lon*((double)3.14)/(double)180.0;


		orb_copy(ORB_ID(vehicle_attitude), sensor_sub_2_fd, &raw_att);
		double quat[4];
		quat[0] = raw_att.q[0]; quat[1] = raw_att.q[1]; quat[2] = raw_att.q[2]; quat[3] = raw_att.q[3];
		double* rpy =  quaternion_to_rad_gazebo_classic(quat);
		
		double roll_curr = rpy[0];
		double pitch_curr = rpy[1];
		double yaw_curr = rpy[2];


		orb_copy(ORB_ID(sensor_gyro), sensor_sub_gy_fd, &raw_gyro);
		double roll_dot_curr = (double)raw_gyro.x;
		double pitch_dot_curr = -(double)raw_gyro.y;

		orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);



		orb_copy(ORB_ID(sensor_accel), sensor_sub_accel_fd, &raw_accel);

		// ********************* Logic for gazebo_classic control *********************
		
		// finding the current n vector in FRD frame of drone's body using the current orientation matrix of drone
		nx_curr = (cos(roll_curr) * sin(pitch_curr) * cos(yaw_curr)) + (sin(roll_curr) * sin(yaw_curr));
		ny_curr = (cos(roll_curr) * sin(pitch_curr) * sin(yaw_curr)) - (sin(roll_curr) * sin(yaw_curr));
		// nz_curr = cos(roll_curr) * cos(pitch_curr);
		nz_curr = sqrt(1 - (nx_curr*nx_curr + ny_curr*ny_curr));

		// getting the desired n vector using the outer loop thread		
		pthread_mutex_lock(&shared_state_mutex);
		nxdes = shared_state.nx_des;
		nydes = shared_state.ny_des;
		// azdes = shared_state.az_des;
		pthread_mutex_unlock(&shared_state_mutex);


		// defining a constant k2 for finding n_dot vector from n vector as n_dot vector = k2 * n vector
		// here we are assuming the n_dot vector approximately some proportional times of n vector
		double k2 = 2 * 0.8 * 1.0;      // k2 = 2*eta*w_n
		
		// finding desired roll, pitch values using n_dot vector
		double roll_dot_des = k2 * (ny_curr - nydes) / nz_curr;
		double pitch_dot_des = - k2 * (nx_curr - nxdes) / nz_curr;

		// defining constants nat_freq and damp_const for finding double derivative of desired roll, pitch from derivative of desired roll and pitch
		double nat_freq = 1.0;
		double damp_const = 0.1;

		double roll_ddot_des = 2 * damp_const * nat_freq * roll_dot_des;
		double pitch_ddot_des = 2 * damp_const * nat_freq * pitch_dot_des;

		double roll_ddot_curr = 2 * damp_const * nat_freq * roll_dot_curr;
		double pitch_ddot_curr = 2 * damp_const * nat_freq * pitch_dot_curr;
        
		// defining the inverse K matri so w2 = 0) x for [w1^2, w3^2, w4^2] = K^-1 [Tx, Ty Th]
		// Moment of inertia of iris drone (in Kg m**2) and mass m in Kg
		double Ix = 0.029125;
		double Iy = 0.029125;
		// double Iz = 0.055225;
		// double m = 1.535;
		// double g = 9.81;

		double a = 120.0;
		// Calculating Desired Torques and Thrust
		double Tx = Ix * a * (roll_ddot_des - roll_ddot_curr);
		double Ty = -Iy * a * (pitch_ddot_des - pitch_ddot_curr);
		// Th = 1.2*m * azdes;
        // double Th = m * (g - az) / (cos(pitch_curr) * cos(roll_curr));

		if ((alt_des - alt) > 0){
			p = (double)1;
		}else{
			p = (double)-1;
		}
		double beta = 2.5;
		Th = (1.0*m * (g) + p * beta * (1 - (1/exp(fabs(alt_des - alt))))); // fabs(cos(pitch_curr) * cos(roll_curr));
		
		// (here we have failed motor 2
		double k = 11.68e0-6;   
		double k1 = 1e0-6;//1.857e0-6;
        
		// k here is motor constant (same for all motors)
		double K[3][3];

		K[0][0] = 1.0 / (2.0 * k);
		K[0][1] = -1.0 / (2.0 * k1);
		K[0][2] = 0.0;

		K[1][0] = 1.0 / (2.0 * k);
		K[1][1] = 0.0;
		K[1][2] = 1.0 / (2.0 * k1);

		K[2][0] = 0.0;
		K[2][1] = 1.0 / (2.0 * k1);
		K[2][2] = -1.0 / (2.0 * k1);

		// finding W_sqr matrix containing the squares of w of three remaining motors
		double W_sqr[3];
		W_sqr[0] = K[0][0] * Th + K[0][1] * Tx + K[0][2] * Ty;
		W_sqr[1] = K[1][0] * Th + K[1][1] * Tx + K[1][2] * Ty;
		W_sqr[2] = K[2][0] * Th + K[2][1] * Tx + K[2][2] * Ty;

		// Calculating the thrust for remaianing individual motors
		double Th_max_motor = 8.016;
		double Thm[3];
		Thm[0] = k * W_sqr[0];
        Thm[1] = k * W_sqr[1];
		Thm[2] = k * W_sqr[2];

		act.control[2] = Thm[0]/Th_max_motor;
		act.control[3] = Thm[1]/Th_max_motor;
		act.control[1] = fmax(0.0,Thm[2]/Th_max_motor);

		printf("nx:%f\t ny:%f\n",nx_curr,ny_curr);

		orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);

		usleep(10);


	};
	
	return 0;
}


int gazebo_classic_main(int argc, char *argv[]){

    pthread_t outer_thread;
	pthread_t inner_thread;
    
    // Initialize the mutex
    if (pthread_mutex_init(&shared_state_mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        return 1;
    }
    
    // // Create the updater thread
    if (pthread_create(&outer_thread, NULL, outerLoop, NULL) != 0) {
        perror("Failed to create outerLoop thread");
        return 1;
    }
    
    // Create the reader thread
    if (pthread_create(&inner_thread, NULL, inner_loop, NULL) != 0) {
        perror("Failed to create innerLoop thread");
        return 1;
    }
    
    // Wait for both threads to complete
    pthread_join(outer_thread, NULL);
    pthread_join(inner_thread, NULL);
    
    // Destroy the mutex
    pthread_mutex_destroy(&shared_state_mutex);

    return 0;

}
