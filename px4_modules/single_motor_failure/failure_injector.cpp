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
 * @file failure_injector.cpp
 * Implementation of Injector Class
 * Injects failure in desired motor
 *
 */

#include "failure_injector.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>

px4::AppState Injector::appState;   /* track requests to terminate app */

int Injector::main(int motor_index)
{
    appState.setRunning(true);
    int actuator_outputs_fd = orb_subscribe(ORB_ID(actuator_motors));      // Subscribing to the actuator_motors topic so that we do not change the values on other motors
    actuator_motors_s controller;
    actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act); // Advertising to the actuator_motors topic for publishing to the motor to be failed

    printf("Injecting failure at instance %d...\n", motor_index);

    // In each iteration of loop, fail the desired motor by publishing NaN on it

    while (!appState.exitRequested() ) {
        px4_sleep(0.1);
        updateController(actuator_outputs_fd, controller);                 // Subscribing thrust values for other motors
        failMotorIndex(controller, act, motor_index);                      // Failing the desired motor
        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);            // Publishing on the actuator_motors
    }
    return 0;
}

void Injector::updateController(int actuator_outputs_fd, actuator_motors_s &controller) {
    orb_copy(ORB_ID(actuator_motors), actuator_outputs_fd, &controller);
}

void Injector::failMotorIndex(const actuator_motors_s &controller, actuator_motors_s &act, int motor_index) {
    int num_controls = controller.NUM_CONTROLS;

    for (int i = 0; i < num_controls; i++) {
        act.control[i] = (motor_index == 0 || i == (motor_index - 1)) ? (float)nan("1") : controller.control[i];  // Publish NaN to fail the motor
    }
}
