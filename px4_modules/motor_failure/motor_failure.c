/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
 * @file motor_failure.c
 * Application to simulate motor failure for testing purposes
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>

static void usage(const char *reason);
static void fail_motor(int motor_id, bool continuous);

__EXPORT int motor_failure_main(int argc, char *argv[]);

// Flag for continuous mode control
static volatile bool _task_should_exit = false;

static void usage(const char *reason)
{
    if (reason != NULL) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
Utility to simulate motor failure.

WARNING: Use this command with caution and only in simulation.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("motor_failure", "command");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start motor failure simulation");
    PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 12, "Motor to fail (1...12)", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('c', "Continuous mode - keep running until stopped", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop motor failure simulation");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print current status");
}

static void fail_motor(int motor_id, bool continuous)
{
    // Adjust motor_id to 0-based index
    int motor_index = motor_id - 1;
    
    // Create and initialize actuator_motors message
    struct actuator_motors_s act;
    memset(&act, 0, sizeof(act));
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    // Set all motors to normal operation (1.0) except the failed motor (0.0)
    for (int i = 0; i < 12; i++) {
        act.control[i] = (i == motor_index) ? 0.0f : 1.0f;
    }

    if (continuous) {
        _task_should_exit = false;
        while (!_task_should_exit) {
            orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
            PX4_INFO("Motor %d failure simulation active", motor_id);
            // px4_usleep(100000); // 100ms delay
        }
    } else {
        // Single publication
        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
        PX4_INFO("Motor %d failure simulated once", motor_id);
    }

    orb_unadvertise(act_pub_fd);
}

int motor_failure_main(int argc, char *argv[])
{
    int ch;
    int motor_id = -1;
    bool continuous = false;
    
    int myoptind = 1;
    const char *myoptarg = NULL;

    // Parse command line arguments
    while ((ch = px4_getopt(argc, argv, "m:c", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'm':
                motor_id = strtol(myoptarg, NULL, 10);
                break;
            case 'c':
                continuous = true;
                break;
            default:
                usage(NULL);
                return 1;
        }
    }

    if (argc <= 1) {
        usage(NULL);
        return 1;
    }

    if (myoptind >= argc) {
        usage(NULL);
        return 1;
    }

    if (strcmp(argv[myoptind], "start") == 0) {
        if (motor_id < 1 || motor_id > 12) {
            usage("Invalid motor ID. Must be between 1 and 12.");
            return 1;
        }
        
        PX4_INFO("Starting motor failure simulation for motor %d", motor_id);
        fail_motor(motor_id, continuous);
        return 0;

    } else if (strcmp(argv[myoptind], "stop") == 0) {
        _task_should_exit = true;
        PX4_INFO("Stopping motor failure simulation");
        return 0;

    } else if (strcmp(argv[myoptind], "status") == 0) {
        PX4_INFO("Motor failure simulation status:");
        if (motor_id != -1) {
            PX4_INFO("Currently simulating failure of motor %d", motor_id);
        } else {
            PX4_INFO("No motor failure simulation active");
        }
        return 0;
    }

    usage(NULL);
    return 1;
}