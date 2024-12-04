File Structure
inter-iit_Team62  
├── Flight_Analysis                      # Contains all flight analysis data and scripts  
│   ├── detection_logs                   # vehicle_odometry data for detection purpose
│   │   ├── csv                          # Odometry CSV files
│   │   │   └── odometry_data_0.csv         
│   │   ├── plots                        # Odometry plots  
│   │   │   └── odometry_plot_0.png          
│   │   └── px4_logs                     # px4 detection logs  
│   │       └── px4_logs_0.csv
|   |           
│   ├── flight_logs                      # Flight state logs from drone_state.h  
│   │   └── flight_log_0.csv
|   |            # Flight log details  
│   ├── scripts                  
│   │   ├── automation                   # Scripts for automated drone tasks  
│   │   │   ├── demo.sh                  # Script for demonstrating smf functionality  
│   │   │   ├── hovering.sh              # Script for automating drone hovering sessions  
│   │   │   ├── takeoff.sh               # Script for automating drone takeoff sessions
│   │   │   └── manuevring.sh            # Script for automating drone maneuvers sessions 
│   │   ├── launch_drone_env.py          # launched drone env (QGC + DDS Agent)  
│   │   ├── mission.py                   # Script for running a MAVSDK mission  
│   │   ├── odometry_subscriber.py       # Script for subscribing to odometry data  
│   │   └── plotting_automation.py       # Script for automating data plotting  
|   |
│   ├── LQR_Optimizer.ipynb              # Notebook to optimize LQR and tuning penalty and control matrices.  
│   └── px4_data_plotter.ipynb           # Notebook for plotting flight_logs data 
|  
└── px4_modules
    ├── gazebo_classic_fault_tolerant_control  #Classic fault tolerant control module
    ├── iris_with_standoffs              # IRIS quadcopter meshes
    ├── single_motor_failure             # smf module 
    ├── DroneState.msg                   # custom uORB message
    ├── model.sdf                        # iris sdf file for harmonic
    