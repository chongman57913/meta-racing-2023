# MoCAM-Competition
This is a repo of a autonomous driving beaseline in CARLA for the competition. 

## System Dependency
- **System requirement**: Ubuntu 20.04

- **ROS (Noetic)**: http://wiki.ros.org/noetic/Installation/Ubuntu

- **CARLA**:https://carla.readthedocs.io/en/latest/start_quickstart/

- **Pytorch (version 1.13 or later)**: https://pytorch.org/get-started/locally/

- **MoCAM CARLA map**: https://google.com (update later)

## Install python and ROS package dependency
```bash
pip install carla, transforms3d, cvxpy

sudo apt install ros-noetic-tf2-sensor-msgs
```

## Build ROS projects for MoCAM demo
```bash
# Build all the projects in the repo

cd /path/to/repo_folder/MoCAM_carla-ros-bridge/catkin_ws
catkin_make
source devel/setup.bash

cd /path/to/repo_folder/MoCAM_pt2laserscan_ws
catkin_make
source devel/setup.bash

cd /path/to/repo_folder/MoCAM_yolodetect_ws
catkin_make
source devel/setup.bash

cd /path/to/repo_folder/MoCAM_MPC_ROS_ws
catkin_make
source devel/setup.bash
```

## Load variable for ROS projects 
### Add ROS path via modifying .bashrc
```bash
vim ~/.bashrc
```

### Running the setup script by appending the following commands in .bashrc
```bash
# For ROS & Carla-ROS-Bridge
source /opt/ros/noetic/setup.bash
source /path/to/ros_bridge/carla-ros-bridge/catkin_ws/devel/setup.bash

# For MoCAM demo
source /path/to/repo_folder/MoCAM_pt2laserscan_ws/devel/setup.bash
source /path/to/repo_folder/MoCAM_yolodetect_ws/devel/setup.bash
source /path/to/repo_folder/MoCAM_MPC_ROS_ws/devel/setup.bash
```
### Modify the model checkpoint path for Yolo object detector
```xml
<!-- Modify the setting in /path/to/repo_folder/MoCAM_yolodetect_ws/src/yolov5_ros/launch/yolov5.launch -->

<arg name="weights" default="/path/to/yolo_checkpoint_folder/checkpoint_name.pt"/>
```

## Run Carla map
``` bash
cd /path/to/carla_map_folder
./CarlaUE4.sh
```

## Start ROS core service in a new terminal window
```bash
roscore
```

## Run a ROS service to convert 3D point cloud into a 2D laser scan
```bash
roslaunch pointcloud_to_laserscan pt2_to_scan.launch
```

## Run a ROS service for object detection using Yolo v5
```bash
roslaunch yolov5_ros yolov5.launch
```

## Add a vehicle entity in CARLA world
```bash
roslaunch carla_ros_bridge run_car_sim_Town04.launch
```

## Run baseline demo (MPC control, Obstacle detector, YOLO v5 object detector)
```bash
rosrun mpc_ros mpc_Town04_launch.py 
```

## Tips for debugging
### Enable RVIZ
RVIZ is a ROS GUI that allows you to visualize a lot of information and helps debugging in your implementation. You can enable it by adding the following code in the ROS launch file.

```xml
<!-- 
    The launch file is located at: 
    /path/to/repo_folder/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_Town04.launch
-->

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find carla_ros_bridge)/rviz/mpc.rviz"/>
```

### Enable vehicle manual control in CARLA
There is a built-in manual control for vehicles in CARLA. You can control your car during the debugging stages. Add the following code in the ROS launch file if you want to enable the manual control module.
```xml
<!-- 
    The launch file is located at: 
    /path/to/repo_folder/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_Town04.launch
-->

  <include file="$(find carla_manual_control)/launch/carla_manual_control.launch" unless="$(arg auto_control)">
  <arg name='role_name' value='agent_0'/>
  </include>
```


## Tools for evulation (update later)
```
python test_pedestrian.py (run the walker)
python mpc_Pub.py (control the traffic_light)
python time_recorder_with_mpc_Town04.py (run the time_recorder with MoCAM Demo together)
python collision_counter_traffic_collision_evnt_Qt.py (run the collision_recorder for the wall collision event)
python collision_recorder_traffic_light_Qt.py (run the collision_recorder for the wrong traffic rule event)
python collision_recorder_walker_hurt_Qt.py (run the collision_recorder for the walker hurt event)
```
