#!/usr/bin/env python3
import rospy
from turtlebot3_task.msg import degrees_msg  
from std_msgs.msg import Float32

class KalmanFilter:
    def __init__(self):
        self.process_variance = 0.01  # Process variance
        self.measurement_variance = 0.1   # Measurement variance
        self.estimate = 0.0   # Initial estimate
        self.estimate_covariance = 1.0   # Initial estimation covariance

        rospy.init_node('kalman_filter')

        rospy.Subscriber('conversion', degrees_msg, self.callback)

        self.filtered_pub = rospy.Publisher('filtered_yaw', Float32, queue_size=10)

    def callback(self, data):
        # Extract the noisy YAW angle
        z = data.yaw

        # Prediction step
        self.estimate_covariance += self.process_variance

        # Update step
        K = self.estimate_covariance / (self.estimate_covariance + self.measurement_variance)
        self.estimate += K * (z - self.estimate)
        self.estimate_covariance *= (1 - K)

        # Publish the filtered YAW angle
        filtered_yaw = Float32()
        filtered_yaw.data = self.estimate
        self.filtered_pub.publish(filtered_yaw)

        rospy.loginfo("Noisy Yaw: %.2f, Filtered Yaw: %.2f", z, self.estimate)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        kf = KalmanFilter()
        kf.run()
    except rospy.ROSInterruptException:
        pass
