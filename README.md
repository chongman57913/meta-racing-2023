# MoCAM-Competition
This is a repo of a autonomous driving beaseline in CARLA for the competition. 

## System Dependency
- **System requirement**: Ubuntu 20.04

- **Python**: Python 3.8

- **ROS (Noetic)**: http://wiki.ros.org/noetic/Installation/Ubuntu

- **CARLA**:https://carla.readthedocs.io/en/latest/start_quickstart/

- **Pytorch (version 1.13 or later)**: https://pytorch.org/get-started/locally/

- **MoCAM CARLA map**: https://go.um.edu.mo/1b0q464z (password: Um@853)

## Install python and ROS package dependency
```bash
pip install carla==0.9.13

pip install transforms3d cvxpy opencv-python pandas Pillow requests torch torchvision seaborn matplotlib

sudo apt install ros-noetic-tf2-sensor-msgs ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs  ros-noetic-vision-msgs
```

## Build ROS projects for MoCAM demo
```bash
# Build all the projects in the repo

cd ~/MoCAM_carla-ros-bridge/catkin_ws
catkin_make && source devel/setup.bash

cd ~/MoCAM_pt2laserscan_ws
catkin_make && source devel/setup.bash

cd ~/MoCAM_yolodetect_ws
catkin_make && source devel/setup.bash

cd ~/MoCAM_MPC_ROS_ws
catkin_make && source devel/setup.bash
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
source ~/MoCAM_carla-ros-bridge/catkin_ws/devel/setup.bash

# For MoCAM demo
source ~/MoCAM_pt2laserscan_ws/devel/setup.bash
source ~/MoCAM_yolodetect_ws/devel/setup.bash
source ~/MoCAM_MPC_ROS_ws/devel/setup.bash

# Configure the path of the CARLA map folder
export CARLA_ROOT="/path/to/CARLA map/folder"
```

### Reload .bashrc 
```bash
source ~/.bashrc
```

### Modify the model checkpoint path for Yolo object detector
To obtain a pre-train checkpoint of YOLOv5, please check on [the official Github](https://github.com/ultralytics/yolov5).

If you would like to train a YOLOv5 model by yourself, please refer to the [section](#train-a-yolo-v5-object-detection-model) below.
```xml
<!-- Modify the setting in ~/MoCAM_yolodetect_ws/src/yolov5_ros/launch/yolov5.launch -->

<arg name="weights" default="/path/to/yolo_checkpoint_folder/checkpoint_name.pt"/>
```

## Run Carla map
``` bash
cd $CARLA_ROOT
./CarlaUE4.sh
```
CARLA_ROOT is the root folder for CARLA

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

## Train a YOLO v5 object detection model
to be updated

## Tips for debugging
### Enable RVIZ
RVIZ is a ROS GUI that allows you to visualize a lot of information and helps debugging in your implementation. You can enable it by adding the following code in the ROS launch file.

```xml
<!-- 
    The launch file is located at: 
    ~/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_Town04.launch
-->

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find carla_ros_bridge)/rviz/mpc.rviz"/>
```

### Enable vehicle manual control in CARLA
There is a built-in manual control for vehicles in CARLA. You can control your car during the debugging stages. Add the following code in the ROS launch file if you want to enable the manual control module.
```xml
<!-- 
    The launch file is located at: 
    ~/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_Town04.launch
-->

  <include file="$(find carla_manual_control)/launch/carla_manual_control.launch" unless="$(arg auto_control)">
  <arg name='role_name' value='agent_0'/>
  </include>
```

### Generate pedestrian and vehicle in CARLA world
to be updated


## Tools for evulation (update later)
```
python test_pedestrian.py (run the walker)
python mpc_Pub.py (control the traffic_light)
python time_recorder_with_mpc_Town04.py (run the time_recorder with MoCAM Demo together)
python collision_counter_traffic_collision_evnt_Qt.py (run the collision_recorder for the wall collision event)
python collision_recorder_traffic_light_Qt.py (run the collision_recorder for the wrong traffic rule event)
python collision_recorder_walker_hurt_Qt.py (run the collision_recorder for the walker hurt event)
```
