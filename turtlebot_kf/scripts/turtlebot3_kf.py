import rospy
import numpy as np
import tf.transformations 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Point, Twist

class Kalman_filter:
    def __init__(self):
        self.sigma_yaw=0.25
        self.sigma_yaw_dot=0.3
        self.dt=0.1
        self.A = np.array([[1, self.dt],[0, 1]])
        self.Q = np.diag([self.sigma_yaw**2, self.sigma_yaw_dot**2])
        self.R = np.diag([self.sigma_yaw_dot**2, self.sigma_yaw_dot**2])
        self.C = np.array([[1, 0],[0,1]])
        self.I = np.eye(2)
        self.x0 = np.array([[0,0]])
        self.P0 = np.diag([200,200])

    def kalman_node(self):
        rospy.init_node("kalman_node",anonymous=True)
        rospy.Subscriber("/imu",Imu,   self.imu_callback)
        rospy.Subscriber("/noisy_state",Float32MultiArray,   self.noisy_state_callback)
        self.filtered_heading_pub=rospy.Publisher('/filtered_heading', Float32MultiArray,queue_size=10)
        rospy.spin()

    def imu_callback(self,imu_data_msg):
        self.imu_quat = [
            imu_data_msg.orientation.x,
            imu_data_msg.orientation.y,
            imu_data_msg.orientation.z,
            imu_data_msg.orientation.w,
        ]
        self.roll_imu,  self.pitch_imu,  self.yaw_imu = tf.transformations.euler_from_quaternion( self.imu_quat)
        self.yaw_dot = imu_data_msg.angular_velocity.z
        self.imu_measurement = np.array([[self.yaw_imu],[self.yaw_dot]])
        # self.z = []
        # self.z.append(self.yaw_imu)
        # self.z.append(self.yaw_dot)
        # self.filtered_heading_pub.publish(Float32MultiArray(data = self.z))
        # print(self.z)

    def noisy_state_callback(self,noisy_state_data_msg):
        self.x_hat = np.array([[noisy_state_data_msg.data[0],noisy_state_data_msg.data[1]]]).T
        print("x_hat 111",self.x_hat)
        self.p = (self.A.dot(self.P0).dot(self.A.T)) + self.Q
        # print("p",self.p)
        self.k = self.p.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.p).dot(self.C.T)+self.R))
        # print("K",self.k)
        self.x_hat = self.x_hat + self.k.dot((self.imu_measurement-self.C.dot(self.x_hat)))
        print("x_hat",self.x_hat)
        self.filtered_heading = []
        self.filtered_heading.append(float(self.x_hat[0]))
        self.filtered_heading.append(float(self.x_hat[1]))
        self.filtered_heading_pub.publish(Float32MultiArray(data = self.filtered_heading))
        print(self.filtered_heading)
        self.x0 = self.x_hat
        self.P0 = self.p

if __name__=='__main__':
    kalman_filter=Kalman_filter()
    kalman_filter.kalman_node()