#!/bin/bash

loading_bar() {
    duration=$1
    bar=""

    for ((i = 1; i <= duration; i++)); do
        bar="${bar}#"
        printf "\r[%-${duration}s]" "$bar"
        sleep 1
    done
    printf "\n"
}

for ((run = 1; run<=3; run++)); do
    echo "Starting run for height 10m with motor failure 1"

    tmux new-session -d -s drone_automation
    echo "Created new tmux session 'drone_automation'."

    tmux split-window -v -t drone_automation

    tmux send-keys -t drone_automation "cd ~/ros2/PX4-Autopilot && make px4_sitl gz_x500" C-m
    echo "Starting PX4 SITL with Gazebo Harmonic..."
    loading_bar 15

    tmux send-keys -t drone_automation "param set MIS_TAKEOFF_ALT 10" C-m
    echo "Set takeoff altitude to 10 m..."
    loading_bar 1

    if((run==1)); then
        echo "Starting Hovering Session"
        tmux send-keys -t drone_automation "commander takeoff" C-m
        echo "Sent 'commander takeoff' to SITL..."
        loading_bar 12
    fi
    if ((run==2)); then
        echo "Starting Takeoff Session"
        tmux send-keys -t drone_automation "commander takeoff" C-m
        echo "Sent 'commander takeoff' to SITL..."
        loading_bar 8
    fi
    if((run==3)); then
        echo "Starting Landing Session"
        tmux send-keys -t drone_automation "commander takeoff" C-m
        echo "Sent 'commander takeoff' to SITL..."
        loading_bar 12

        tmux send-keys -t drone_automation "commander land" C-m
            echo "Sent 'commander land' to SITL..."
            loading_bar 4
    fi
    tmux send-keys -t drone_automation.0 "python3 ~/ros2/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/detection_tests/scripts/odometry_subscriber.py" C-m
    echo "Running Odometry Subscriber"

    loading_bar 1
    tmux send-keys -t drone_automation "smf detect" C-m
    echo "Sent 'smf detect' to SITL... Starting Detection Deamon"
    loading_bar 1

    tmux send-keys -t drone_automation "smf start 1" C-m
    echo "Sent 'smf start 1' to SITL... Starting Injection Deamon"
    loading_bar 2

    tmux send-keys -t drone_automation C-c
    echo "Terminating SITL session..."

    tmux kill-session -t drone_automation.0
    echo "Terminating Odometry session..."
    echo "Completed run for height 10 m with motor failure 1."
    sleep 5
done
echo "Automation script completed for all runs."