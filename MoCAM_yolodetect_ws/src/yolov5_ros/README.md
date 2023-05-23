# YOLOv5 ROS
This is a ROS interface for using YOLOv5 for real time object detection on a ROS image topic. It supports inference on multiple deep learning frameworks used in the [official YOLOv5 repository](https://github.com/ultralytics/yolov5).

## Installation

### Dependencies
This package is built and tested on Ubuntu 20.04 LTS and ROS Noetic with Python 3.8.

* Clone the packages to ROS workspace and install requirement for YOLOv5 submodule:
```bash
cd <ros_workspace>/src
git clone https://github.com/mats-robotics/detection_msgs.git
git clone --recurse-submodules https://github.com/mats-robotics/yolov5_ros.git 
cd yolov5_ros/src/yolov5
pip install -r requirements.txt # install the requirements for yolov5
```
* Build the ROS package:
```bash
cd <ros_workspace>
catkin build yolov5_ros # build the ROS package
```
* Make the Python script executable 
```bash
cd <ros_workspace>/src/yolov5_ros/src
chmod +x detect.py
```

## Basic usage
Change the parameter for `input_image_topic` in launch/yolov5.launch to any ROS topic with message type of `sensor_msgs/Image` or `sensor_msgs/CompressedImage`. Other parameters can be modified or used as is.

* Launch the node:
```bash
roslaunch yolov5_ros yolov5.launch
```

## Using custom weights and dataset (Working)
* Put your weights into `yolov5_ros/src/yolov5`
* Put the yaml file for your dataset classes into `yolov5_ros/src/yolov5/data`
* Change related ROS parameters in yolov5.launch: `weights`,  `data`

## Train a YOLOv5 model in MoCAM

### Data collection
To train an object detection model, first we need to prepare an annotated dataset. You can run the script to collect image from the camera from the vehicle. 

```bash
# Please ensure the vehicle entity is added in the CARLA world

python image_collector.py /path/to/the/image/folder/
```

### Data annotation
After preparing the image set, we have to annotate the object in the image. [https://github.com/heartexlabs/labelImg]() is a tool for data annotation, and you can find how to use the tool by watching the [demo video](https://www.youtube.com/watch?v=zSda1AoUTkc).
```bash
git clone https://github.com/heartexlabs/labelImg

# Run the labelImg tools and choose YOLO as annotation format
./labelImg/labelImg
```

### Train the model using the annotated dataset
To define the details of the training dataset, we have to create a yaml file to declears 1) the dataset root directory path and relative paths to train / val / test image directories, and 2) a class names dictionary.

```yaml
path: /path/to/image_dataset/root  # dataset root dir
train: images/train  # train images (relative to 'path') 
val: images/val # val images (relative to 'path')
test:  # test images (optional)

# Classes
names:
  0: red
  1: yellow
  2: green
  3: pedestrian
  #......
```

Then we can train a YOLOv5s model with a specific training dataset.

```bash
cd ~/yolov5
python train.py --data image_test_YOLO.yaml --weights yolov5s.pt --img 640
```

## Reference
* YOLOv5 official repository: https://github.com/ultralytics/yolov5
* YOLOv3 ROS PyTorch: https://github.com/eriklindernoren/PyTorch-YOLOv3
* Darknet ROS: https://github.com/leggedrobotics/darknet_ros
