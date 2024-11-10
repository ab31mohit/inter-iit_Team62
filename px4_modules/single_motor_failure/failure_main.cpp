/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_start.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Mark Charlebois <mcharleb@gmail.com>
 */
#include "failure_injector.h"
#include "failure_detector.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>

#include "failure_controller.h"
#include <uORB/Publication.hpp>
#include <px4_platform_common/init.h>


#include <fstream>
#include <filesystem>
#include <string>
static int daemon_injector_task;             /* Handle of deamon task / thread */
static int daemon_detector_task;
namespace fs = std::filesystem;
int injected_motor;
long long int injection_unix;
long long int detection_unix;
//using namespace px4;
std::string log_directory;
std::string csv_filename = "/px4_logs_0.csv";
int vehicle_odometry_fd;
vehicle_odometry_s odometry;
// const char* homeDir = std::getenv("HOME");

#include <iostream>
#include <chrono>
#include <cstdlib> 

int injected_timestamp;
long long int getUnix(){
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    long long int timestamp = duration.count() / 1000;
    return timestamp;
}
std::string expandTilde(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* homeDir = std::getenv("HOME");
        if (homeDir) {
            return std::string(homeDir) + path.substr(1);
        } else {
            PX4_WARN("HOME environment variable is not set.");
            return path; // Return the original path if HOME is not set
        }
    }
    return path;
}
int daemon_detector(int argc, char **argv)
{
	px4::init(argc, argv, "Motor Failure Detector");

	printf("Starting failure detection \n");
    	log_directory = "~/inter-iit_Team62/detection_tests/detection_logs/px4_logs";
	
	Detector detection;

	int detected_motor = detection.main(); 
    bool updated = false;
	orb_check(vehicle_odometry_fd, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
	} else {
		PX4_WARN("No new data available; timestamp might be zero");
	}
	
	detection_unix = getUnix();
	int detected_timestamp = odometry.timestamp;
	int latency_int = (detected_timestamp - injected_timestamp);
	double latency = latency_int * 1e-6;
	printf("Failure detected at motor %d ...... \nDetection Timestamp = %d \nDetection latency = %f \n", detected_motor,detected_timestamp
	, latency);

	printf("Injection UNIX: %lld\nDetected UNIX:  %lld\n", injection_unix, detection_unix);
	printf("goodbye\n");

	std::string expanded_directory = expandTilde(log_directory);

    // Initialize file path and index
    std::string file_path = expanded_directory + csv_filename;
    int file_index = 0;

    // Find a unique file name
    while (fs::exists(file_path)) {
        file_index++;
        csv_filename = "/px4_logs_" + std::to_string(file_index) + ".csv";
        file_path = expanded_directory + csv_filename;
    }

    // Open the file in append mode and log the values
    std::ofstream csv_file(file_path, std::ios::app);
    if (csv_file.is_open()) {
        csv_file << detected_timestamp << "," << injected_timestamp << "," << latency_int << ","
                 << detected_motor << "," << injected_motor << "," << detection_unix << ","
                 << injection_unix << "\n";
        csv_file.close();
    } else {
        PX4_WARN("Failed to open CSV log file: %s\n", file_path.c_str());
    }

	 if(detected_motor > 0) {
	   printf("Starting Failure Control \n");
           Controller control;
           control.main();

	 }
	
	return 0;
}


int daemon_injector(int argc, char **argv)
{
	px4::init(argc, argv, "Motor Failure Injector");

    
	    // Check if we received the motor ID argument
    if (argc > 1) {

        // argv[1] should contain the motor ID

        const char* motor_id = argv[1];
        printf("Starting failure injection at motor instance %s\n", motor_id);

        int motor_index = atoi(motor_id); 

		bool updated = false;
		orb_check(vehicle_odometry_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
		} else {
			PX4_WARN("No new data available; timestamp might be zero");
		}

		injection_unix = getUnix();
		injected_timestamp = odometry.timestamp;

		printf("Injection at %d \n", injected_timestamp);
        Injector injection;
        injection.main(motor_index); 


    } else {
        printf("No motor ID specified. usage: smf {start|stop|status} {instance} \n(instance = 0 fails all motors)");
        return 1;
    }

	printf("goodbye\n");
	return 0;
}

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
		injected_motor = std::atoi(motor_failure_args[0]);
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

		daemon_detector_task =  px4_task_spawn_cmd("Failure Detector",
						 	    SCHED_DEFAULT,
						        SCHED_PRIORITY_MAX - 5,
								2000,
						 		daemon_detector,
						 		(argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
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

