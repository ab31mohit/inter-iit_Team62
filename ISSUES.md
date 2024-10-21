# Information about the issues during the installation:

## changes in .bashrc file:
##ROS2
source /opt/ros/humble/setup.bash


##PX4
export PATH=$PATH:$HOME/.local/bin
export GZ_VERSION=harmonic

#### Note: the command below doesn't make much difference 
export GZ_SIM_RESOURCE_PATH=$HOME/ros2/PX4-Autopilot/Tools/simulation/gz/models:$HOME/ros2/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH





## issues

1. Running bash script for px4 installation:

    WARNING: The scripts pyserial-miniterm and pyserial-ports are installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The script isympy is installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The scripts f2py and numpy-config are installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The scripts alldefconfig, allmodconfig, allnoconfig, allyesconfig, defconfig, genconfig, guiconfig, listnewconfig, menuconfig, oldconfig, olddefconfig, savedefconfig and setconfig are installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The scripts ulog2csv, ulog2kml, ulog2rosbag, ulog_extract_gps_dump, ulog_info, ulog_messages, ulog_migratedb and ulog_params are installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The script nnvg is installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.
    WARNING: The script jsonschema is installed in '/home/rajeev-gupta/.local/bin' which is not on PATH.
    Consider adding this directory to PATH or, if you prefer to suppress this warning, use --no-warn-script-location.

    sol: 
    export PATH=$PATH:/home/rajeev-gupta/.local/bin

2. Micro DDS installation

    <!-- CMake Warning (dev) at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:438 (message):
      The package name passed to `find_package_handle_standard_args` (tinyxml2)
      does not match the name of the calling package (TinyXML2).  This can lead
      to problems in calling code that expects `find_package` result variables
      (e.g., `_FOUND`) to follow a certain pattern.
    Call Stack (most recent call first):
      cmake/modules/FindTinyXML2.cmake:40 (find_package_handle_standard_args)
      build/temp_install/fastdds-3.1/share/fastdds/cmake/fastdds-config.cmake:51 (find_package)
      CMakeLists.txt:153 (find_package)
    This warning is for project developers.  Use -Wno-dev to suppress it. -->


3. make command for PX4:
    submodules aren't installed correctly/recursively. 

    try: 
        reclone PX4 and make
        check there's no fatal error in cloning 

4. colcon build: px4_msgs
    --- stderr: px4_msgs                               
    Traceback (most recent call last):
      File "/home/rajeev-gupta/ros2/ws_ros2/build/px4_msgs/ament_cmake_python/px4_msgs/setup.py", line 4, in <module>
        setup(


    sol: 
        ref: https://github.com/PX4/px4_msgs/issues/42
        First please try to check your current version of packaging

        '''pip list | grep packaging'''
        if the version of packaging is <22.0, please upgrade it to 22.0

        '''pip install packaging==22.0'''



        

