#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def publish_msg():
    pub = rospy.Publisher('/carla/traffic_light_sensor/control', Float32, queue_size=10)
    rospy.init_node('my_node', anonymous=True)

    # 等待120秒
    rospy.sleep(160)

    # 按照指定的频率发布消息，持续40秒
    rate = rospy.Rate(1000) # 1000 Hz
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < 40.0:
        pub.publish(0.0)
        rate.sleep()

    # 停止发布
    pub.unregister()

if __name__ == '__main__':
    try:
        publish_msg()
    except rospy.ROSInterruptException:
        pass
