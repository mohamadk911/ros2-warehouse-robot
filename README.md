# ROS2 Warehouse Robot Navigation

## Description
This project implements autonomous navigation of a TurtleBot3 robot in a custom warehouse environment using ROS2 and Gazebo.  
The robot follows a predefined route to simulate transport of goods between locations inside a warehouse.

---

## Project Structure

```text
ros2_ws/
├── src/
│   └── warehouse_delivery/   # ROS2 package control logic
├── models/                   # Robot and environment models
├── worlds/                   # Gazebo warehouse world
└── routes/                   # Recorded navigation paths CSV
```


---

## Requirements
- ROS2 Humble
- Gazebo
- TurtleBot3 packages

---
## Build Instructions
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

Run Simulation (4 Terminals)
Terminal 1 — Gazebo Server
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH=~/ros2_ws/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
gzserver ~/ros2_ws/worlds/warehouse.world \
-s libgazebo_ros_init.so \
-s libgazebo_ros_factory.so

Terminal 2 - Gazebo GUI
export DISPLAY=:0
export GAZEBO_MODEL_PATH=~/ros2_ws/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
gzclient

Terminal 3 — Spawn Robot
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=~/ros2_ws/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
ros2 run gazebo_ros spawn_entity.py \
-entity tb3 \
-file ~/ros2_ws/models/turtlebot3_burger/model.sdf \
-x -16 -y -10 -z 0.1

Terminal 4 — Run Navigation
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run warehouse_delivery play_route \
--ros-args -p route_file:=~/ros2_ws/routes/test1.csv





























s/worlds/warehouse.world \
-s libgazebo_ros_init.so \
-s libgazebo_ros_factory.so
