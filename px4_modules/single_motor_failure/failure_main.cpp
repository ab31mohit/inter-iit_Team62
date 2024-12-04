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

/**
 * @file failure_main.cpp
 *   Entry point for single_motor_failure module
 *   smf detect to start detector daemon
 *   smf start {instance} to start injector daemon
 */


#include "failure_injector.h"
#include "failure_detector.h"
#include "failure_state_sub.h"
#include "failure_controller.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <sched.h>
#include <array>
#include <px4_platform_common/init.h>


// ---------------------------
#include <filesystem>
#include <iostream>  // For std::cout
#include <cstdio>    // For printf
#include <fstream>   // For std::ofstream
namespace fs = std::filesystem;
int injected_motor;
//using namespace px4;
std::string log_directory;
std::string csv_filename = "/px4_log_0.csv";
// ----------------------------


static int daemon_injector_task;             /* Handle of deamon task / thread */
static int daemon_detector_task;


int injection_timestamp;


int vehicle_odometry_fd;
vehicle_odometry_s odometry;


// Function for the Injector daemon, injects failure in desired motor

int daemon_injector(int argc, char **argv)
{
	px4::init(argc, argv, "Motor Failure Injector");  // Startup function, prints welcome message


        printf("Starting motor failure injection");

        int motor_index = 1;

	orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);   // Updating odometry value for timestamp
	injection_timestamp = odometry.timestamp;                             // Injection Timestamp to calculate latency

	printf("Injection at %d \n", injection_timestamp);

        // Call the main function of Injector class for failure injection in the desired motor

        Injector injection;
        injection.main(motor_index);

    

	printf("goodbye\n");
	return 0;
}


int daemon_state_sub(int argc, char **argv)
{
	px4::init(argc, argv, "Motor State Subscriber");
        StateSub state_sub;
	state_sub.appState.setRunning(true);
        state_sub.main();
	return 0;
}

// Function for the Detector daemon, detects motor failure and starts control

int daemon_detector(int argc, char **argv)
{
	px4::init(argc, argv, "Motor Failure Detector");

	printf("Starting failure detection \n");
	std::string path = "~/inter-iit_Team62/Flight_Analysis/detection_logs/px4_logs";
	const char* homeDir = std::getenv("HOME");
	log_directory = std::string(homeDir) + path.substr(1);
	Detector detection;

	// Call the main function of Detector class to start detecting failure

	int detected_motor = detection.main();
	// int detected_motor = 1;

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);    // Updating odometry value for timestamp

	int detected_timestamp = odometry.timestamp;                           // Detection Timestamp to calculate latency
	double latency = (detected_timestamp - injection_timestamp) * 1e-6;    // Latency
	printf("Failure detected at motor %d ...... \nDetection Timestamp = %d \nDetection latency = %f \n", detected_motor,detected_timestamp
	, latency);
	printf("goodbye\n");

	// ---------------------------------------------
	int file_index = 0;
	std::string file_path = log_directory + csv_filename;
    // Find a unique file name
    while (fs::exists(file_path)) {
        file_index++;
        csv_filename = "/px4_log_" + std::to_string(file_index) + ".csv";
        file_path = log_directory + csv_filename;
    }
	// printf(file_path.c_str()+"\n");
    // Open the file in append mode and log the values
    std::ofstream csv_file(file_path, std::ios::app);
    if (csv_file.is_open()) {
        csv_file << detected_timestamp << "," << injection_timestamp << "," << latency << ","
                 << detected_motor << "," << injected_motor << "\n";
        csv_file.close();
    } else {
        PX4_WARN("Failed to open CSV log file: %s\n", file_path.c_str());
    }
	// ------------------------------------------------

	// If some motor failure is detected start control

	if(detected_motor > 0) {

	  // Call the main function of Controller class

	  printf("Starting Failure Control \n");
          Controller control;
          control.main(detected_motor);

	}
	return detected_motor;
}


// Entry point for the module, handles the command line inputs to start the desired background task (daemon)


extern "C" __EXPORT int smf_main(int argc, char *argv[]);
int smf_main(int argc, char *argv[])
{

	vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));


	if (argc < 2) {
		PX4_WARN("usage: smf {detect|start|stop|status} {instance} \n(instance = 0 fails all motors) \n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (Injector::appState.isRunning()) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		char *motor_failure_args[2];
                motor_failure_args[0] = argv[2];  // Motor ID to fail
                motor_failure_args[1] = nullptr;   // Null-terminate the array

		// Launch Injector Daemon

		daemon_injector_task =  px4_task_spawn_cmd("Motor Failure",
						 	    SCHED_DEFAULT,
						        SCHED_PRIORITY_MAX - 5,
								2000,
						 		daemon_injector,
						 		motor_failure_args);

		return 0;
	}

	if (!strcmp(argv[1], "detect")) {

		if (Detector::appState.isRunning()) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		// Launch Detector Daemon

		daemon_detector_task =  px4_task_spawn_cmd("Failure Detector",
						 	    SCHED_DEFAULT,
						                SCHED_PRIORITY_MAX - 5,
								2000,
						 		daemon_detector,
						 		(argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		printf("\nvalue of detector is %d",daemon_detector_task);


		return 0;
	}
	if (!strcmp(argv[1], "sub")) {

		if (StateSub::appState.isRunning()) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		// Launch Detector Daemon

		daemon_detector_task =  px4_task_spawn_cmd("State Publisher",
						 	    SCHED_DEFAULT,
						                SCHED_PRIORITY_MAX - 5,
								2000,
						 		daemon_state_sub,
						 		(argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		printf("\nvalue of detector is %d",daemon_detector_task);


		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		Injector::appState.requestExit();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (Injector::appState.isRunning()) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: smf {start|stop|status} \n");
	return 1;
}

////////////////////////////////////////////


