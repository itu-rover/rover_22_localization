#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
import rospara


class ReferenceStation():
    
    def __init__(self):
        """ 
            Updates the mean of lat, long, alt
        """
        rospy.init_node("gnss_publisher")

        # Mean values
        self.received_number = 1
        self.mean_coords = [
            0.0, # self.mean_lat
            0.0, # self.mean_long
            0.0  # self.mean_alt
        ]

        self.current_time = rospy.Time.now()

        # Publisher part
        self.ref_pub = rospy.Publisher("/gnss/ref/fix", NavSatFix, queue_size=10)
        self.ref_err_pub = rospy.Publisher("/gnss/ref/error", Float64MultiArray)

        # Subscriber part
        rospy.Subscriber("/fix", NavSatFix, self.mean_updater)
        rospy.Subscriber("/fix", NavSatFix, self.difference_calculator)

    
    def mean_updater(self, data):
        """ 
            Updates the mean of lat, long, alt

            Publishes mean of lat, long, alt of reference station
        """
        counter = 0
        self.instant_coords = [
            data.latitude, # in degrees
            data.longitude, # in degrees
            data.altitude # in meters
            ]

        for coord in self.instant_coords:
            self.mean_coords[counter] = (self.mean_coords[counter] + coord) / self.received_number
            counter+=1
        
        # Create NavSatFix message
        self.ref_nav = NavSatFix()
        self.ref_nav.header.stamp = self.current_time
        self.ref_nav.header.frame_id = "base_link"
        self.ref_nav.latitude = self.mean_coords[0]
        self.ref_nav.longitude = self.mean_coords[1]
        self.ref_nav.altitude = self.mean_coords[2]
        self.ref_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        
        # Publish the message
        self.ref_pub.publish(self.nav)



    def difference_calculator(self):
        """ 
            Calculates the differences between each data
            and mean of coordinates.

            Publishes lat, long, alt differences at the reference station
        """
        self.difference_list = [
            # reference mean lat - reference instant lat,
            # reference mean long - reference instant long,
            # reference mean alt - reference instant alt
        ]
        counter = 0

        for coord in self.instant_coords:
            self.difference_list.append(self.mean_coords[counter] - coord)
            counter+=1
        
        self.ref_dif = Float64MultiArray()
        self.ref_dif.data = self.difference_list
        self.ref_err_pub.publish(self.ref_dif)



class RoverStation(self):

    def __init__(self):
        """ 
            
        """
        pass
    

    def coordinate_corrector(self):
        """ 
            Makes the coordinate corrections comes from
            the reference stations.
            
            returns corrected lat, long, alt
        """
        pass



if __name__ == "__main__":
    try:
        pass
    except rospy.ROSInterruptException:
        pass

