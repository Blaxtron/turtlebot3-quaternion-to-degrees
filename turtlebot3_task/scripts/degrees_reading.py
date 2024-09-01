#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from collections import deque
from turtlebot3_task.msg import degrees_msg





def callback(data):
    rospy.loginfo("Roll: %.2f, Pitch: %.2f, Yaw: %.2f",data.roll,data.pitch,data.yaw)





def read():
    rospy.init_node("degree_reading",anonymous=False)
    rospy.Subscriber("conversion",degrees_msg,callback,queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    read()
