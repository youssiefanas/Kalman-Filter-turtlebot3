#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Point, Twist



def node_init():
    rospy.init_node("turtlebot3_noisy",anonymous=True)
    rospy.Subscriber("/odom",Odometry,  odometry_callback)
    global turtlebot3_vel
    global noisy_heading_pub, imu_yaww
    turtlebot3_vel=rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    noisy_heading_pub=rospy.Publisher('/noisy_state', Float32MultiArray,queue_size=10)

    rospy.spin()




def odometry_callback(odom_data):
    explicit_quat = [odom_data.pose.pose.orientation.x,
    odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z,
    odom_data.pose.pose.orientation.w]
    global yaw, yaw_robot,yaw_dot
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    yaw_robot= np.random.normal(yaw,0.25,1)[0]
    ang_z=odom_data.twist.twist.angular.z
    yaw_dot=np.random.normal(ang_z,0.3,1)[0]
    
    global noisy_state 
    noisy_state= []
    noisy_state.append(yaw_robot)
    noisy_state.append(yaw_dot)
    noisy_heading_pub.publish(Float32MultiArray(data = noisy_state))

    velocity_bot=Twist()
    velocity_bot.angular.z=3
    velocity_bot.linear.x=0.7
    turtlebot3_vel.publish(velocity_bot)

    rospy.sleep(0.01)

if __name__=='__main__':
    node_init()
    
