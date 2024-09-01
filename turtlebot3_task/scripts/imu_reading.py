#!/usr/bin/env python3
import rospy
import math
import tf.transformations
from sensor_msgs.msg import Imu
from collections import deque
from turtlebot3_task.msg import degrees_msg

#I'm going to apply a double ended queue for each rotation axis, this is better than the normal queue in that it has the "maxlen" parameter which 
#specifies the max number of items in queue, and starts removing items when the limit is reached in a FIFO manner.
roll_queue = deque(maxlen=10)
pitch_queue = deque(maxlen=10)
yaw_queue = deque(maxlen=10)
pub = rospy.Publisher("rotation",degrees_msg,queue_size=10)

def callback(data):
    

    quaternion = data.orientation
    quaternion_tuple = (quaternion.x,quaternion.y,quaternion.z,quaternion.w) 
    
    degrees = tf.transformations.euler_from_quaternion(quaternion_tuple)

    roll = math.degrees(degrees[0])
    pitch = math.degrees(degrees[1])
    yaw = math.degrees(degrees[2]) 

    #applying a moving average filter for the rotation axes, because there was a lot of noise.
    roll_queue.append(roll)
    roll_average = sum(roll_queue)/len(roll_queue)
    pitch_queue.append(pitch)
    pitch_average = sum(pitch_queue)/len(pitch_queue)
    yaw_queue.append(yaw)
    yaw_average = sum(yaw_queue)/len(yaw_queue)
    
    message = degrees_msg()
    message.roll = roll_average
    message.pitch = pitch_average
    message.yaw = yaw_average
    pub.publish(message)
    
    
    rospy.loginfo("roll: %.2f, pitch: %.2f, yaw: %.2f", roll_average,pitch_average,yaw_average)
    

    


def read():
    ##initializing q2d topic (quaternion to degrees) and subscribing to imu topic
    rospy.init_node('q2d',anonymous=False)
    rospy.Subscriber('imu',Imu,callback,queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    read()
