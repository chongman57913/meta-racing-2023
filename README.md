# MoCAM-Competition
This is a repo of a autonomous driving beaseline in CARLA for the competition. 

## System Dependency
- **System requirement**: Ubuntu 20.04

- **Python**: Python 3.8(Currently does not support python3.9 or above)

- **ROS (Noetic)**: http://wiki.ros.org/noetic/Installation/Ubuntu

- **CARLA**:https://carla.readthedocs.io/en/latest/start_quickstart/

- **Pytorch (version 1.13 or later)**: https://pytorch.org/get-started/locally/

- **MoCAM CARLA map**: https://go.um.edu.mo/1b0q464z (password: Um@853)

## Install python and ROS package dependency
```bash
pip install carla==0.9.13

pip install transforms3d cvxpy opencv-python pandas Pillow requests torch torchvision seaborn matplotlib pyqt5

sudo apt install ros-noetic-tf2-sensor-msgs ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs  ros-noetic-vision-msgs
```

## Build ROS projects for MoCAM demo
```bash
# Build all the projects in the repo

cd ~/meta-racing-2023/MoCAM_carla-ros-bridge/catkin_ws
catkin_make && source devel/setup.bash
or
catkin_make && source devel/setup.zsh

cd ~/meta-racing-2023/MoCAM_pt2laserscan_ws
catkin_make && source devel/setup.bash
or
catkin_make && source devel/setup.zsh

cd ~/meta-racing-2023/MoCAM_yolodetect_ws
catkin_make && source devel/setup.bash
or
catkin_make && source devel/setup.zsh

cd ~/meta-racing-2023/MoCAM_MPC_ROS_ws
catkin_make && source devel/setup.bash
or
catkin_make && source devel/setup.zsh
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
source ~/meta-racing-2023/MoCAM_carla-ros-bridge/catkin_ws/devel/setup.bash

# For MoCAM demo
source ~/meta-racing-2023/MoCAM_pt2laserscan_ws/devel/setup.bash
source ~/meta-racing-2023/MoCAM_yolodetect_ws/devel/setup.bash
source ~/meta-racing-2023/MoCAM_MPC_ROS_ws/devel/setup.bash

# Configure the path of the CARLA map folder
export CARLA_ROOT="ROOT_PATH"
```

### Reload .bashrc 
```bash
source ~/.bashrc
```

### Modify the model checkpoint path for Yolo object detector
To obtain a pre-train checkpoint of YOLOv5, please check on [the official Github](https://github.com/ultralytics/yolov5).

If you would like to train a YOLOv5 model by yourself, please refer to [this section](/MoCAM_yolodetect_ws/src/yolov5_ros/README.md#train-a-yolov5-model-in-mocam).

```xml
<!-- Modify the setting in ~/meta-racing-2023/MoCAM_yolodetect_ws/src/yolov5_ros/launch/yolov5.launch -->

<arg name="weights" default="/your_yolov5_rospackage_folder/checkpoint_name.pt"/>
```

## Run Carla map
``` bash
cd $CARLA_ROOT
./CarlaUE4.sh
```
CARLA_ROOT is the root folder for CARLA, write the 'CARLA_ROOT = $ROOT_PATH' in ~/.bashrc.

## Start ROS core service in a new terminal window
roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate.

```bash
roscore
```

## Run a ROS service to convert 3D point cloud into a 2D laser scan
Converts a 3D Point Cloud into a 2D laser scan. This is useful for making devices like the Kinect appear like a laser scanner for 2D-based algorithms

```bash
roslaunch pointcloud_to_laserscan pt2_to_scan.launch
```

## Run a ROS service for object detection using Yolo v5
Run the YOLOv5 object detection service in ROS. It will use the image from the vehicle and publish the results.

```bash
roslaunch yolov5_ros yolov5.launch
```

## Add a vehicle entity in CARLA world
Run the ROS service to add a vehicle into a CARLA world.  

```bash
roslaunch carla_ros_bridge run_car_sim_mocam.launch
```

## Run baseline demo (MPC control, Obstacle detector, YOLO v5 object detector)
```bash
rosrun mpc_ros mocam_racing_launch.py 
```

## Tips for debugging
### Enable RVIZ
RVIZ is a ROS GUI that allows you to visualize a lot of information and helps debugging in your implementation. You can enable it by adding the following code in the ROS launch file.

```xml
<!-- 
    The launch file is located at: 
    ~/meta-racing-2023/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_mocam.launch
-->

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find carla_ros_bridge)/rviz/mpc.rviz"/>
```

### Enable vehicle manual control in CARLA
There is a built-in manual control for vehicles in CARLA. You can control your car during the debugging stages. Add the following code in the ROS launch file if you want to enable the manual control module.
```xml
<!-- 
    The launch file is located at: 
    ~/meta-racing-2023/MoCAM_carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ros_bridge/launch/run_car_sim_mocam.launch
-->

  <include file="$(find carla_manual_control)/launch/carla_manual_control.launch" unless="$(arg auto_control)">
  <arg name='role_name' value='agent_0'/>
  </include>
```

### Change of road conditions in CARLA world
To test the algorithm in different road conditions, we implement a script to add pedestrian, vehicle, other blueprint object and traffic light control in the CARLA world.
```bash
cd MoCAM_evaluation_tools

python test_pedestrian.py # add pedestrain and blueprint object
python mpc_Pub.py #control the traffic light signal
```

### Collision detector for the competition
We also implement the collision counter to review and evaluate the autonomous driving performance. 
```bash
python time_recorder_with_mocam_racing.py # timer
python collision_counter_traffic_collision_evnt_Qt.py # wall collision
python collision_recorder_traffic_light_Qt.py # run a red light
python collision_recorder_walker_hurt_Qt.py # crash with pedestrian
```