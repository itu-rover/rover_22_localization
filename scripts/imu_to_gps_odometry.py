#!/usr/bin/env python

import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


transformed_imu_pub = rospy.Publisher("/imu/odometry/transformed", Odometry, queue_size=10)

counter = 1


def get_rotation_matrix(roll, pitch, yaw):
    return np.array(
        [
            [np.cos(pitch)*np.cos(yaw), (np.sin(roll)*np.sin(pitch)*np.cos(yaw))-(np.cos(roll)*np.sin(yaw)), (np.cos(roll)*np.sin(pitch)*np.cos(yaw))+(np.sin(roll)*np.sin(yaw)],
            [np.cos(pitch)*np.sin(yaw), (np.sin(roll)*np.sin(pitch)*np.sin(yaw))+(np.cos(roll)*np.cos(yaw)), (np.cos(roll)*np.sin(pitch)*np.sin(yaw))-(np.sin(roll)*np.cos(yaw)],
            [np.sin(pitch)*(-1),         np.sin(roll)*np.cos(pitch),                                          np.cos(roll)*np.cos(pitch)]
        ]
    )


def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if counter==1:
        global rotation_matrix = get_rotation_matrix(roll, pitch, yaw)
        global l_roll, l_pitch, l_yaw = roll, pitch, yaw
        counter++


def orientation_transformer(data):
    transformed_imu_odom_msg = Odometry()
    imu_orientation = np.array(
        [l_roll],
        [l_pitch],
        [l_roll]
    )
    imu_transformed_orientation = np.matmul(imu_orientation, rotation_matrix)  # returns a np.array
    imu_transformed_quaternion = quaternion_from_euler(imu_transformed_orientation)
    
    transformed_imu_odom_msg.pose.pose.orientation.x = imu_transformed_orientation[0,0,0]
    transformed_imu_odom_msg.pose.pose.orientation.y = imu_transformed_orientation[0,0,1]
    transformed_imu_odom_msg.pose.pose.orientation.z = imu_transformed_orientation[0,0,2]
    transformed_imu_odom_msg.pose.pose.orientation.w = imu_transformed_orientation[0,0,3]
    
    transformed_imu_pub.publish(transformed_imu_odom_msg)



if __name__ == "__main__":
    
    try:
        # Initialize Node
        rospy.init_node("geodesy_business")
        
        # Time
        current_time = rospy.Time.now()
        
        # Object registrations
        # reference_station = ReferenceStation()
        # rover_station = RoverStation()
        
        # rospy.Subscriber("/ublox_gps/fix", NavSatFix, reference_station.mean_update_and_calculate_error)  
        # rospy.Subscriber("/fix", NavSatFix, rover_station.rover_instant_position)
        # rospy.Subscriber("/gnss/ref/error", Float64MultiArray, rover_station.coordinate_corrector)
        
        rospy.Subscriber("/imu/data", Imu, get_rotation)
        rospy.Subscriber("/odometry/imu", Odometry, orientation_transformer)
        # rospy.Subscriber("/odometry/gps", Odometry, rover_station.coordinate_corrector)

        
        rospy.spin()
            
            
    except rospy.ROSInterruptException:
        pass
