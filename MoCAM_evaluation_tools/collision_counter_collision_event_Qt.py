#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from carla_msgs.msg import CarlaCollisionEvent
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
import sys


class CollisionSensorSubscriber(QThread):
    counter_signal = pyqtSignal(int)  # Define a new Signal

    def __init__(self):
        QThread.__init__(self)
        rospy.init_node('collision_sensor_subscriber')
        self.counter = 0
        self.last_collision_time = rospy.Time.now()
        self.last_other_id = None
        rospy.Subscriber('/carla/agent_0/collision_sensor', CarlaCollisionEvent, self.callback)

    def run(self):
        rospy.spin()

    def callback(self, data):
        other_id = data.other_actor_id
        current_time = rospy.Time.now()

        if other_id == 0 and (self.last_other_id is None or self.last_other_id != 0):
            self.last_collision_time = current_time
            self.last_other_id = other_id
        elif other_id == 0 and self.last_other_id == 0:
            if (current_time - self.last_collision_time).to_sec() >= 3:
                self.counter += 1
                rospy.loginfo("Counter: {}".format(self.counter))
                self.last_collision_time = current_time
        elif other_id == 12:
            self.counter += 5
            rospy.loginfo("Counter: {}".format(self.counter))
        self.last_other_id = other_id
        self.counter_signal.emit(self.counter)  # Emit the signal


class MainWindow(QWidget):
    def __init__(self, collision_sensor_subscriber):
        super(MainWindow, self).__init__()

        self.counter_label = QLabel()
        collision_sensor_subscriber.counter_signal.connect(self.update_counter)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("碰撞事件罚秒"))
        layout.addWidget(self.counter_label)
        self.setLayout(layout)

    def update_counter(self, counter):
        self.counter_label.setText(str(counter))


if __name__ == '__main__':
    app = QApplication(sys.argv)

    collision_sensor_subscriber = CollisionSensorSubscriber()
    main_window = MainWindow(collision_sensor_subscriber)

    main_window.show()

    collision_sensor_subscriber.start()

    sys.exit(app.exec_())
