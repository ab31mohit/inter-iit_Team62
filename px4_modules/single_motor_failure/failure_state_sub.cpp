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
 * @file failure_state_sub.cpp
 * Subscriber for state logs
*/



#include "failure_state_sub.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/drone_state.h>



#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <array>

#include <uORB/uORB.h>

#include <iostream>  // For std::cout
#include <cstdio>    // For printf
#include <fstream>   // For std::ofstream
#include <unistd.h>
#include <stdio.h>
#include <filesystem>



using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState StateSub::appState;  /* track requests to terminate app */
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
int StateSub::main(){
    int drone_state_sub = orb_subscribe(ORB_ID(drone_state));
    drone_state_s drone_state_data;

    std::string baseFilePath = expandTilde("~/inter-iit_Team62/Flight_Analysis/flight_logs/flight_log");
    // std::string filePath = baseFilePath+"_0";
    int counter = 0;
    std::string filePath = baseFilePath + "_" + std::to_string(counter);

    // Check if the file exists and modify the filename if necessary
    while (std::filesystem::exists(filePath + ".csv")) {
        counter++;
        filePath = baseFilePath + "_" + std::to_string(counter);
    }
    filePath = filePath + ".csv";

    std::ofstream csvFile(filePath, std::ios::out | std::ios::trunc);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file." << std::endl;
        return -1; // Exit if file cannot be opened
    }
    // File is ready for writing
    std::cout << "Opened file: " << filePath << std::endl;

    while (!appState.exitRequested()) {
        orb_copy(ORB_ID(drone_state), drone_state_sub, &drone_state_data);
        usleep(1000);
        // Save odometry data to CSV
        if(drone_state_data.timestamp == prev_timestamp || drone_state_data.timestamp > 99999999){
            // printf("continue ");
            continue;
        }
        // printf("done ");
        csvFile  << drone_state_data.timestamp << ","
                 << drone_state_data.u1 << ","
                 << drone_state_data.u2 << ","
                 << drone_state_data.u3 << ","
                 << drone_state_data.f1 << ","
                 << drone_state_data.f2 << ","
                 << drone_state_data.f3 << ","
                 << drone_state_data.fsum << ","
                 << drone_state_data.alt_error << ","
                 << drone_state_data.q_r << ","
                 << drone_state_data.q_i << ","
                 << drone_state_data.q_j << ","
                 << drone_state_data.q_k << ","
                 << drone_state_data.p_s << ","
                 << drone_state_data.q_s << ","
                 << drone_state_data.r_s << ","
                 << drone_state_data.n_x << ","
                 << drone_state_data.n_y << ","
                 << drone_state_data.n_z << ","
                 << drone_state_data.nx_aor << ","
                 << drone_state_data.ny_aor << ","
                 << drone_state_data.nz_aor << ","
                 << drone_state_data.roll_rad << ","
                 << drone_state_data.pitch_rad << ","
                 << drone_state_data.yaw_rad << ","
                 << drone_state_data.x << ","
                 << drone_state_data.y << ","
                 << drone_state_data.z
                 << std::endl;
        prev_timestamp = drone_state_data.timestamp;
    }
    csvFile.close(); // Close the file when done

    return 0;
}

