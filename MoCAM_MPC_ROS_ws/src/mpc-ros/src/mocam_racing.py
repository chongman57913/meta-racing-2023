#! /usr/bin/env python3

from ast import Pass
from gc import callbacks
from socket import timeout
import rospy
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes,BoundingBox
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
from collections import namedtuple
from utils.curve_generator import curve_generator
from nav_msgs.msg import Odometry, Path
from math import atan2, sin, cos, pi
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import MarkerArray, Marker
import time
import carla
import random
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path 
from carla_msgs.msg import CarlaCollisionEvent
from collections import namedtuple
from math import atan2
from mpc.mpc_path_tracking import mpc_path_tracking
import yaml
import os
import math
# from time_recorder_1 import *

car = namedtuple('car', 'G g cone_type wheelbase abs_speed abs_steer abs_acce abs_acce_steer')
obstacle = namedtuple('obstacle', 'A b cone_type ')
#scan=LaserScan()
#scan.ranges=[1.0,2.0,3.0,4.0]
#threshold=2.0
scaling = 12
obstacle_check_x_range = 1.3
obstacle_check_y_range = 0.3
obstacle_null = 0.0

BASE_DIR = os.path.abspath(os.path.join( os.path.dirname( __file__ ), '..' ))
#obstacle_detected=obstacle_detection(scan,threshold)
with open(f"{BASE_DIR}/src/config.yaml", 'r') as f:
    try:
        cfg = yaml.safe_load(f, Loader=yaml.FullLoader)
    except:
        cfg = yaml.safe_load(f)

odom_topic= cfg["odom_topic"]
ctl_topic= cfg["ctl_topic"]

class mpc_core:
    def __init__(self):
        
        # ros parameter
        receding = rospy.get_param('receding', 10)
        max_speed = rospy.get_param('max_speed', 8)
        ref_speed = rospy.get_param('ref_speed', 4)
        max_acce = rospy.get_param('max_acce', 1)
        max_acce_steer = rospy.get_param('max_acce_steer', 0.02)
        sample_time = rospy.get_param('sample_time', 0.2)
        max_steer = rospy.get_param('max_steer', 0.5)
        wheelbase = rospy.get_param('wheelbase', 2.87)
        self.shape = rospy.get_param('shape', [4.69, 1.85, 2.87, 1.75])   # [length, width, wheelbase, wheelbase_w] limo
        cs = rospy.get_param('cs', [1, 1, 1])
        cs = np.diag(cs)
        cu = rospy.get_param('cu', 1)
        cst = rospy.get_param('cst', [1, 1, 1])
        cst = np.diag(cst)
        cut = rospy.get_param('cut', 1)

        # goals
        start_position = rospy.get_param('start_position', [-10.4, -13.1, pi/2]) 
        goal1_position = rospy.get_param('goal_position', [-10.4, 9.8, pi/2])
        goal2_position = rospy.get_param('goal_position', [13.2, 9.8, 0])
        goal3_position = rospy.get_param('goal_position', [13.2, -13.9, -pi/2]) 
        goal4_position = rospy.get_param('finish_position', [-10.4, -13.1, pi/2]) 
        goal5_position = rospy.get_param('finish_position', [-10.4, 9.8, pi/2])
        goal6_position = rospy.get_param('finish_position', [13.2, 9.8, 0]) 
        goal7_position = rospy.get_param('goal_position', [13.2, -13.9, -pi/2])
        goal8_position = rospy.get_param('finish_position', [-10.4, -13.1, pi/2]) 
        goal9_position = rospy.get_param('finish_position', [-10.4, 9.8, pi/2])
        goal10_position = rospy.get_param('finish_position', [13.2, 9.8, 0]) 
        goal11_position = rospy.get_param('goal_position', [13.2, -13.9, -pi/2])  
        goal12_position = rospy.get_param('finish_position', [-0.8, -13.2, pi/2]) 
        goal13_position = rospy.get_param('finish_position', [-0.0, -9.3, pi/2]) 
        finish_position = rospy.get_param('finish_position', [-3.2, 0.0, 5*pi/12])
        
        self.name = 'Town04'
        self.vehicle_list = []
        self.vel_linear = []
        self.vel_steer = []
        self.time_record = []
        self.marker_id = 0
        self.marker_array = MarkerArray()
        self.marker_array_car = MarkerArray()
        self.obstacle_distance_= float('nan')

        # mpc
        self.mpc_track = mpc_path_tracking(receding=receding, max_speed=max_speed, ref_speed=ref_speed, max_acce=max_acce, max_acce_steer=max_acce_steer, sample_time=sample_time, max_steer=max_steer, wheelbase=wheelbase, cs=np.diag([1, 1, 1]), cu=cu, cst=cst, cut=cut)
        rospy.init_node('mpc_node')
        self.start_time = rospy.get_time()
        self.generated_time = None

        # app = QApplication([])
        # timer = Timer()
        # timer.show()
        # app.exec_()
        # print("00000")
        
        # # 定时器
        # self.timer = QTimer(timer)
        # self.timer.timeout.connect(timer.updateTime)
        # self.timer.start(1000) 

        # # 监听特定脚本启动
        # self.process = QProcess(timer)
        # self.process.started.connect(timer.startTimer)
        # self.process.finished.connect(timer.processFinished) 
        # # self.process.start()
        # # self.process.start("python", ["mpc_Town04_launch.py"])  # 替换成你的特定脚本路径

        # timer.show()
        # # 监听特定脚本启动
        # self.process = QProcess(timer)
        # self.process.started.connect(timer.startTimer)
        # self.process.finished.connect(timer.processFinished) 
        # #轮询定时器
        # self.pollTimer = QTimer(timer)
        # self.pollTimer.timeout.connect(timer.checkVariable)
        # self.pollTimer.start(1000)  # 每秒轮询一次

        # print("1111")
        start_point = np.c_[start_position]
        goal1_point = np.c_[goal1_position]
        goal2_point = np.c_[goal2_position]
        # timer.show()
        goal3_point = np.c_[goal3_position]
        goal4_point = np.c_[goal4_position]
        goal5_point = np.c_[goal5_position]
        goal6_point = np.c_[goal6_position]
        goal7_point = np.c_[goal7_position]
        goal8_point = np.c_[goal8_position]
        goal9_point = np.c_[goal9_position]
        goal10_point = np.c_[goal10_position]
        goal11_point = np.c_[goal11_position]
        goal12_point = np.c_[goal12_position]
        goal13_point = np.c_[goal13_position]
        finish_point = np.c_[finish_position]

        point_list = [start_point, goal1_point, goal2_point, goal3_point, goal4_point, goal5_point, goal6_point, goal7_point, goal8_point, goal9_point, goal10_point, goal11_point, goal12_point, goal13_point, finish_point]

        cg = curve_generator(point_list=point_list, curve_style='dubins', min_radius=0.5)

        self.ref_path_list = cg.generate_curve(step_size=0.1)
        self.vel = Twist()
        self.output = CarlaEgoVehicleControl()
        self.robot_state = start_point.copy()
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0
        
        # rviz show marker of car
        self.pub_marker_car = rospy.Publisher('car_marker', MarkerArray, queue_size=10)
        # ros topic
        #rospy.init_node('obstacle_detector')
        rospy.Subscriber(odom_topic, Odometry, self.robot_state_callback)
        #rospy.Subscriber('/carla/agent_0/scan',LaserScan, self.obstacle_detector)

        #rospy.Subscriber("/scan",LaserScan,self.obstacle_detection)
        self.pub_vel = rospy.Publisher(ctl_topic, CarlaEgoVehicleControl, queue_size=10)
        self.pub_path = rospy.Publisher('dubin_path', Path, queue_size=10)
        self.pub_opt_path = rospy.Publisher('opt_path', Path, queue_size=10)
        self.path = self.generate_path(self.ref_path_list)

    def cal_vel(self, freq=10, **kwargs):

        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            opt_vel, info, flag, _ = self.mpc_track.controller(self.robot_state, self.ref_path_list, iter_num=5)
            self.time_recorder()
            rospy.Subscriber('/carla/agent_0/scan',LaserScan, self.obstacle_detector)
            detections=rospy.wait_for_message('/yolov5/detections',BoundingBoxes)
            Odometrys=rospy.wait_for_message('/carla/agent_0/odometry',Odometry)
            # collision_detector=rospy.wait_for_message('/carla/agent_0/collision_sensor',CarlaCollisionEvent)
            bouding_detector=detections.bounding_boxes
            Odometry_position_x=Odometrys.pose.pose.position.x
            Odometry_position_y=Odometrys.pose.pose.position.y
            # print(type(Odometry_position_y))
            # print("x:{},y:{}".format(Odometry_position_x,Odometry_position_y))
            first_element = None
            # print(bouding_detector)
            has_green = False
            has_red = False
            for i in range(len(bouding_detector)):
                 ele = bouding_detector[i]
                 if ele.Class == "red":
                      has_red = True
                 if ele.Class == "green":
                      has_green = True
            sign_stop = has_red and not has_green


            # if len(bouding_detector) > 0:
            #     first_element = bouding_detector[0]
            #     print(type(first_element), first_element)
            #     cls = first_element.Class
            # print(bouding_detector)

            # rospy.loginfo("bouding_detector:{}".format(bouding_detector))
            print("self.obstacle_distance_",self.obstacle_distance_)
            # print("generated_time",time_recorder)
            # scan_msg = rospy.wait_for_message('/carla/agent_0/scan', LaserScan, timeout=5)
            # scan_data=scan_msg.ranges
            # scan_degree=len(scan_data)
            # min_angle=int(scan_degree/2)-50
            # max_angle=int(scan_degree/2)+50
            # min_distance=min(scan_data[min_angle:max_angle])
            # curr_scan=np.array(min_distance)
            # if min_range < 2.0:
            # print("min_distance",min_distance)
            # if prev_scan is not None:
                # diff=curr_scan-prev_scan
                # displacement=np.linalg.norm(diff)
                # speed=displacement / 0.1
                # print("speed:",speed)

                # if speed > 1.0:
                    # obstacle_detected = True
                    # print("obstacle detected!")
                    # self.vel.linear.x = 0
                    # self.vel.angular.z = 0
                    # self.output.throttle = 0
                    # self.output.steer = 0
            if self.obstacle_distance_ < obstacle_check_x_range or (sign_stop and 1.0 < Odometry_position_y <3.0):
                # if self.obstacle_distance_ > obstacle_null:
                    obstacle_detected = True
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.output.throttle = 0
                    self.output.steer = 0
                    self.output.brake        = 1
                    print("obstacle detected!")
            #    self.vel.linear.x = 0
            #    self.vel.angular.z = 0
            #    self.output.throttle = 0
            #    self.output.steer = 0
            else:
                # if self.obstacle_distance_ > obstacle_null:
                    obstacle_detected=False
                    self.output.brake = 0
            # prev_scan=curr_scan
            # if not obstacle_detected:
                #print('******')
                #print('states:\n', self.robot_state)
            if flag == True:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.output.throttle = 0
                    self.output.steer = 0
                    # self.output.brake    = 0

            else:
                    self.vel.linear.x = round(opt_vel[0, 0], 2)
                    self.vel.angular.z = round(opt_vel[1, 0], 2)
                    self.output.throttle = round(opt_vel[0, 0], 2) / scaling
                    self.output.steer = - round(opt_vel[1, 0], 2)

            #print('actions:')
            #print('linear', round(opt_vel[0, 0], 2), 'steer', round(opt_vel[1, 0], 2))
            #self.output.brake = 0
            self.pub_vel.publish(self.output)
            self.pub_path.publish(self.path)
            self.pub_marker_car.publish(self.marker_array_car)

            rate.sleep()

    def robot_state_callback(self, data):
        
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        quat = data.pose.pose.orientation

        raw = mpc_core.quat_to_yaw(quat)

        raw_degree = mpc_core.quat_to_yaw(quat)*180/pi

        self.x = x
        self.y = y
        self.z = z
        self.angle = raw_degree

        offset = self.shape[2] / 2
        
        self.robot_state[0] = x - offset * cos(raw)
        self.robot_state[1] = y - offset * sin(raw)
        self.robot_state[2] = raw
    

    @staticmethod
    def generate_path(ref_path_list):
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'

        for i in range(len(ref_path_list)):
            ps = PoseStamped()
            ps.pose.position.x = ref_path_list[i][0, 0]
            ps.pose.position.y = ref_path_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def yaw_to_quat(yaw):

         
        w = cos(yaw/2)
        x = 0
        y = 0
        z = sin(yaw/2)

        quat = Quaternion(w=w, x=x, y=y, z=z)

        return quat
    
    def obstacle_detector(self, scan):
        # 将障碍物距离重置为正无穷大
        obstacle_distance_ = float('inf')
        # 获取激光扫描数据中的数据点数
        ranges = len(scan.ranges)
        scan.angle_increment = math.pi/180*20
        # 循环遍历所有数据点
        for i in range(ranges):
            # 如果数据点距离小于最小距离，则跳过该点
            if scan.ranges[i] < scan.range_min:
                continue
            # 计算当前数据点对应的角度
            angle = scan.angle_min + i * scan.angle_increment
            x = scan.ranges[i] * math.cos(angle)
            y = scan.ranges[i] * math.sin(angle)
            # 如果数据点在车辆前方并且在y轴范围内
            if x > 0 and abs(y) < obstacle_check_y_range:
                # 如果当前数据点距离比之前的障碍物距离更近，则更新障碍物距离
                if x < obstacle_distance_:
                    obstacle_distance_ = x
        self.obstacle_distance_=obstacle_distance_
        #print("self.obstacle_distance_",self.obstacle_distance_)

    def time_recorder(self):
        # mpc_path_tracking=mpc_path_tracking()
        # self.mpc_track.controller()
        if self.mpc_track.flag and self.generated_time is None:
            finished_time=rospy.get_time()
            self.generated_time=finished_time-self.start_time
        if self.mpc_track.flag:
            print("generated_time", self.generated_time)         
        # return generated_time

#    def obstacle_detection(scan):
#        obstacle_range = 0.5
#        for i in range(len(scan.ranges)):
#                if scan.ranges[i] < obstacle_range:
#                    cmd_publisher(True)
#                    return
#        cmd_publisher.publish(False)
        
#        cmd_publisher = rospy.Publisher(
#            "/carla/ego_vehicle/vehicle_control_cmd",
#            CarlaEgoVehicleControl,
#            queue_size=10)
#        command = CarlaEgoVehicleControl()
#        command.throttle     = 0
#        command.steer        = 0
        # command.reverse      = reverse
#        command.brake        = 1
#        cmd_publisher.publish(command)

#    def obstacle_detector(self, msg):
#        obstacle_range = 5
#        for i in range(len(msg.ranges)):
#                if msg.ranges[i] < obstacle_range:
                       #pub=rospy.Publisher('obstacle_detected',Bool,queue_size=10)
                       #pub.publish(True)
                       #return
                #pub.publish(False)
#                    pub=rospy.Publisher("/carla/agent_0/vehicle_control_cmd",CarlaEgoVehicleControl,queue_size=10)
#                    command = CarlaEgoVehicleControl()
#                    command.throttle     = 0
#                    command.steer        = 0
#                    command.brake        = 1#                    pub.publish(command)
#                    return

                   
         