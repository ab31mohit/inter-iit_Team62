#!/bin/bash

# Function to display a loading bar for specified seconds
loading_bar() {
    duration=$1
    bar=""

    for ((i = 1; i <= $(echo "$duration" | bc); i++)); do
        bar="${bar}#"
        printf "\r[%-${duration}s]" "$bar"
        sleep 1
    done
    printf "\n"
}

# Array of times (use whole numbers or decimals)
# times=(17.0 20.0 25.0 30.0 35.0 40.0)
times=(17.0 20.0 25.0 30.0 35.0)
failures=(1 2 3 4) # Motor failure indices

# Run the entire process for each time and failure scenario
for time in "${times[@]}"; do
    int_time=$(echo "$time" | bc) # Convert to integer for wait times

    for failure in "${failures[@]}"; do
        for((run =1; run<=3; run++)); do
            echo "Starting run ${run} for time ${time}s with motor failure ${failure}."

            # Start a new tmux session named "drone_automation" and keep it detached initially
            tmux new-session -d -s drone_automation
            echo "Created new tmux session 'drone_automation'."

            # Pane 0: Launch PX4 SITL with Gazebo
            tmux send-keys -t drone_automation.0 "cd ~/PX4-Autopilot && make px4_sitl gz_x500" C-m
            echo "Starting PX4 SITL..."
            loading_bar 15

            # Split window vertically and run `mission.py` in Pane 1
            tmux split-window -v -t drone_automation.0
            tmux send-keys -t drone_automation.1 "python3 inter-iit_ws/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/scripts/mission.py" C-m
            echo "Running mission.py..."
            if (( $(echo "$time == 17.0" | bc -l) )); then
                loading_bar 19
            elif (( $(echo "$time == 20.0" | bc -l) )); then
                loading_bar 20
            elif (( $(echo "$time == 25.0" | bc -l) )); then
                loading_bar 25
            elif (( $(echo "$time == 30.0" | bc -l) )); then
                loading_bar 30
            elif (( $(echo "$time == 35.0" | bc -l) )); then
                loading_bar 35
            elif (( $(echo "$time == 40.0" | bc -l) )); then
                loading_bar 40
            fi

            # Split again for odometry subscriber in Pane 2
            tmux split-window -v -t drone_automation.1
            tmux send-keys -t drone_automation.2 "python3 inter-iit_Team62/detection_tests/scripts/odometry_subscriber.py" C-m
            echo "Running Odometry Subscriber"
            loading_bar 1

            # Send 'smf detect' command to the SITL terminal
            tmux send-keys -t drone_automation.0 "smf detect" C-m
            echo "Sent 'smf detect' to SITL..."
            loading_bar 1

            # Simulate motor failure in Pane 0 (PX4 SITL)
            tmux send-keys -t drone_automation.0 "smf start ${failure}" C-m
            echo "Simulated motor failure for motor ${failure}..."
            loading_bar 5

            # Terminate the Odometry and mission.py sessions
            tmux send-keys -t drone_automation.2 C-c
            echo "Terminated Odometry Subscriber."

            tmux send-keys -t drone_automation.1 C-c
            echo "Terminated mission.py session."

            # Send Ctrl+C to terminate the SITL session gracefully
            tmux send-keys -t drone_automation.0 C-c
            echo "Terminated SITL session."

            # Kill the tmux session
            tmux kill-session -t drone_automation
            echo "Completed run for time ${time}s with motor failure ${failure}."

            # Sleep before the next run (you can adjust the sleep time as needed)
            sleep 8
        done
    done
done

echo "Automation script completed for all runs."
