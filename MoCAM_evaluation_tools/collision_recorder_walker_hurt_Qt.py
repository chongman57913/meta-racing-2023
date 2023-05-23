#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from detection_msgs.msg import BoundingBoxes,BoundingBox
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
import sys


class DetectionSubscriber(QThread):
    counter_signal = pyqtSignal(int)  # Define a new Signal

    def __init__(self):
        QThread.__init__(self)
        rospy.init_node('detection_subscriber')
        self.flag1 = False
        self.processed1 = False
        self.counter = 0
        self.new_detection = False
        rospy.Subscriber('/carla/agent_0/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.detection_callback)

    def run(self):
        rospy.spin()

    def odometry_callback(self, odometry):
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        if x > -13.0 and x< -7.0 and y > -4.1 and y <-3.0:
            self.flag1 = True
        else:
            self.flag1 = False
            self.processed1 = False

    def detection_callback(self, bounding):
        detection=bounding.bounding_boxes
        for i in range(len(detection)):
            ele = detection[i]
            if ele.Class == "red":
                if self.flag1 and not self.processed1:
                    self.counter += 1
                    self.processed1 = True
                    self.new_detection = True

        if self.new_detection:
            rospy.loginfo("Counter: {}".format(self.counter))
            self.counter_signal.emit(self.counter)  # Emit the signal
            self.new_detection = False


class MainWindow(QWidget):
    def __init__(self, detection_subscriber):
        super(MainWindow, self).__init__()

        self.counter_label = QLabel()
        detection_subscriber.counter_signal.connect(self.update_counter)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("闯红灯罚秒"))
        layout.addWidget(self.counter_label)
        self.setLayout(layout)

    def update_counter(self, counter):
        self.counter_label.setText(str(counter))


if __name__ == '__main__':
    app = QApplication(sys.argv)

    detection_subscriber = DetectionSubscriber()
    main_window = MainWindow(detection_subscriber)

    main_window.show()

    detection_subscriber.start()

    sys.exit(app.exec_())
