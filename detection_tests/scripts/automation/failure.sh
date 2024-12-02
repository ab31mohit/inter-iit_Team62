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

height=10
failure=1
echo "Starting run for height ${height}m with motor failure ${failure}..."

# Start a new tmux session named "drone_automation" and keep it detached initially
tmux new-session -d -s drone_automation
echo "Created new tmux session 'drone_automation'."

# Split tmux window and start the odometry subscriber Python script in a new pane
tmux split-window -v -t drone_automation

# Pane 1: Launch PX4 SITL with Gazebo in the main pane
tmux send-keys -t drone_automation "cd ~/ros2/PX4-Autopilot && make px4_sitl gz_x500" C-m
echo "Starting PX4 SITL..."
loading_bar 15

# Set the takeoff altitude parameter
tmux send-keys -t drone_automation "param set MIS_TAKEOFF_ALT ${height}" C-m
echo "Set takeoff altitude to ${height}m..."
loading_bar 1

# Send 'commander takeoff' command
tmux send-keys -t drone_automation "commander takeoff" C-m
echo "Sent 'commander takeoff' to SITL..."

if (( $(echo "$height <= 10.0" | bc -l) )); then
    loading_bar 15
elif (( $(echo "$height > 10.0" | bc -l) )); then
    loading_bar 20
fi

# Send 'smf detect' command to the SITL terminal
tmux send-keys -t drone_automation "smf detect" C-m
echo "Sent 'smf detect' to SITL..."
loading_bar 1

# Simulate motor failure
tmux send-keys -t drone_automation "smf sub" C-m
echo "Sent 'smf sub' to SITL..."
tmux send-keys -t drone_automation "smf start ${failure}" C-m
echo "Simulated motor failure for motor ${failure}..."
loading_bar 8

# Send Ctrl+C to terminate the SITL session gracefully
tmux send-keys -t drone_automation C-c
echo "Terminating SITL session..."

# Send Ctrl+C to terminate the Odometry session gracefully
tmux kill-session -t drone_automation.0
echo "Terminating Odometry session..."

echo "Completed run for height ${height}m with motor failure ${failure}."
