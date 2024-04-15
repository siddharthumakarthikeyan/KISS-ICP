#!/bin/bash

# Terminal 1
gnome-terminal -- bash -c "python3 remap.py"

# Terminal 2
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link velodyne"

# Terminal 3
gnome-terminal -- bash -c "cd my_scripts/; python3 task_1.py"

