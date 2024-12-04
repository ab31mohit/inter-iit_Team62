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

    tmux send-keys -t drone_automation "cd ~/PX4-Autopilot && make px4_sitl gz_x500" C-m
    echo "Starting PX4 SITL with Gazebo Harmonic..."
    loading_bar 16

    tmux send-keys -t drone_automation "param set MIS_TAKEOFF_ALT 10" C-m
    echo "Set takeoff altitude to 10 m..."
    loading_bar 1

    echo "Starting Hovering Session"
        tmux send-keys -t drone_automation "commander takeoff" C-m
        echo "Sent 'commander takeoff' to SITL..."
        loading_bar 12

    tmux send-keys -t drone_automation.0 "python3 inter-iit_Team62/Flight_Analysis/scripts/odometry_subscriber.py 
y" C-m
    echo "Running Odometry Subscriber"

    loading_bar 1
    tmux send-keys -t drone_automation "smf detect" C-m
    echo "Sent 'smf detect' to SITL... Starting Detection Deamon"
    loading_bar 1

    tmux send-keys -t drone_automation "smf sub" C-m
    echo "Sent 'smf sub' to initiate Subscriber Deamon"
    loading_bar 1

    tmux send-keys -t drone_automation "smf start 1" C-m
    echo "Sent 'smf start' to SITL... Starting Injection Deamon"
    loading_bar 6

    tmux send-keys -t drone_automation C-c
    echo "Terminating SITL session..."

    tmux kill-session -t drone_automation.0
    echo "Terminating Odometry session..."
    echo "Completed run for height 10 m with motor failure 1."
    sleep 3
done
echo "Automation script completed for all runs."