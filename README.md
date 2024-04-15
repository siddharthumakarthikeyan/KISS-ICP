# KISS-ICP

KISS-ICP (Keep It Simple, Stupid - Iterative Closest Point) is a robotics project focusing on odometry estimation using the ICP algorithm. This project captures local maps at poses of interest and uses these maps along with a query scan to estimate odometry.

## Prerequisites

Before you begin, ensure you have met the following requirements:
* You have a `Linux` machine with ROS installed.
* You have installed `git`.

## Dataset

## Cloning KISS-ICP

## Installation
Install pykitti
```bash
pip install pykitti
```

Workspace Configuration
```bash
cd <workspace>/
git clone https://github.com/siddharthumakarthikeyan/KISS-ICP
colcon build
source install/setup.bash
```
## Task 1
```bash
cd <workspace>/
./task_1.sh
```
You can open RVIZ2 to visualize the 3D Point Cloud data getting streamed under the topic /raw_point.

## Task 2
```bash
cd <workspace>/
./task_2.sh
```
A local map is generated and published under the topic /local_map and upon reaching the pose of interest the map saver is triggered and the map is been saved in the maps subfolder.

## Task 3
```bash
cd <workspace>/
./task_3.sh
```
A local map is published under the topic /local_map and query scan is given input for the node. Upon reaching the pose of interest the odom logger is triggered and the odometry estimated by the KISS-ICP algorithm is captured. The code is designed to easily change the deisred point of interest.
