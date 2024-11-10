#!/bin/bash

# Function to display a loading bar for specified seconds
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

# Array of heights to be used in the sessions
heights=(10.0 15.0 20.0)
failures=(1 2 3 4) # Motor failure indices (assuming motors are 1-indexed)

# Total runs (4 heights * 4 failure scenarios)
total_runs=$(( ${#heights[@]} * ${#failures[@]} ))

# Run the entire process for each height and failure scenario
for height in "${heights[@]}"; do
    for failure in "${failures[@]}"; do
        for((run =1; run<=5; run++)); do
            echo "Starting run ${run}"
            echo "Starting run for height ${height}m with motor failure ${failure}..."

            # Start a new tmux session named "drone_automation" and keep it detached initially
            tmux new-session -d -s drone_automation
            echo "Created new tmux session 'drone_automation'."

            # Split tmux window and start the odometry subscriber Python script in a new pane
            tmux split-window -v -t drone_automation

            # Pane 1: Launch PX4 SITL with Gazebo in the main pane
            tmux send-keys -t drone_automation "cd ~/PX4-Autopilot && make px4_sitl gz_x500" C-m
            echo "Starting PX4 SITL..."
            loading_bar 15

            # Set the takeoff altitude parameter
            tmux send-keys -t drone_automation "param set MIS_TAKEOFF_ALT ${height}" C-m
            echo "Set takeoff altitude to ${height}m..."
            loading_bar 1

            # Send 'commander takeoff' command
            tmux send-keys -t drone_automation "commander takeoff" C-m
            echo "Sent 'commander takeoff' to SITL..."

            if (( $(echo "$height == 10.0" | bc -l) )); then
                loading_bar 12
            elif (( $(echo "$height == 15.0" | bc -l) )); then
                loading_bar 17
            elif (( $(echo "$height == 20.0" | bc -l) )); then
                loading_bar 22
            fi

            # Send 'commander takeoff' command
            tmux send-keys -t drone_automation "commander land" C-m
            echo "Sent 'commander land' to SITL..."
            loading_bar 4

            # Pane 0: Start the odometry subscriber Python script
            tmux send-keys -t drone_automation.0 "python3 inter-iit_Team62/detection_tests/scripts/odometry_subscriber.py" C-m
            echo "Running Odometry Subscriber"
            loading_bar 1

            # Send 'smf detect' command to the SITL terminal
            tmux send-keys -t drone_automation "smf detect" C-m
            echo "Sent 'smf detect' to SITL..."
            loading_bar 1

            # Simulate motor failure
            tmux send-keys -t drone_automation "smf start ${failure}" C-m
            echo "Simulated motor failure for motor ${failure}..."
            loading_bar 2

            # Send Ctrl+C to terminate the SITL session gracefully
            tmux send-keys -t drone_automation C-c
            echo "Terminating SITL session..."

            # Send Ctrl+C to terminate the Odometry session gracefully
            tmux kill-session -t drone_automation.0
            echo "Terminating Odometry session..."

            # Optionally attach to the tmux session for manual monitoring
            # Uncomment if you want to monitor each session
            # tmux attach-session -t drone_automation

            echo "Completed run for height ${height}m with motor failure ${failure}."
            
            # Sleep before the next run (you can adjust the sleep time as needed)
            sleep 10
        done
    done
done

echo "Automation script completed for all runs."