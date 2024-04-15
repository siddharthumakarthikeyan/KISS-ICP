#!/bin/bash

# Terminal 1
gnome-terminal -- bash -c "python3 remap.py"

# Terminal 2
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link velodyne"

# Terminal 3
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch kiss_icp odometry.launch.py topic:=velodyne_points"

# Terminal 4
gnome-terminal -- bash -c "cd my_scripts/; python3 task_1.py"

# Terminal 5
gnome-terminal -- bash -c "cd my_scripts/; python3 task_2.py"

# Terminal 6
gnome-terminal -- bash -c "cd odom_logger/; python3 odom_log.py"
