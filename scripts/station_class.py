#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix


class ReferenceStation():
    
    def __init__(self):
        """ 
            Initialize the node, subscribers and publishers.
            Set the initial values of mean coordinates and 
            coordinate's number.
            Set the current time.
        """
        rospy.init_node("ref_coord_publisher")

        # Mean values
        self.received_number = 1
        self.mean_coords = [
            0.0, # self.mean_lat
            0.0, # self.mean_long
            0.0  # self.mean_alt
        ]

        self.current_time = rospy.Time.now()

        # Publisher part
        self.ref_coord_pub = rospy.Publisher("/gnss/ref/fix_mean", NavSatFix, queue_size=10)
        self.ref_coord_err_pub = rospy.Publisher("/gnss/ref/error", Float64MultiArray, queue_size=1)

        # Subscriber part
        rospy.Subscriber("/fix", NavSatFix, self.mean_updater)  # /gnss/ref/fix
        rospy.Subscriber("/fix", NavSatFix, self.difference_calculator)  # /gnss/ref/fix

    
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
        self.ref_nav.latitude = self.mean_coords[0]
        self.ref_nav.longitude = self.mean_coords[1]
        self.ref_nav.altitude = self.mean_coords[2]
        self.ref_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        
        # Publish the message
        self.ref_coord_pub.publish(self.ref_nav)



    def difference_calculator(self, data):
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
        
        self.ref_coord_dif = Float64MultiArray()
        self.ref_coord_dif.data = self.difference_list
        self.ref_coord_err_pub.publish(self.ref_coord_dif)

        # Return to use in RoverStation class
        return self.ref_coord_dif



class RoverStation():

    def __init__(self):
        """ 
            
        """
        self.reference = ReferenceStation()

        #rospy.init_node("rover_coord_publisher")
        self.current_time = rospy.Time.now()

        self.rover_coord_pub = rospy.Publisher("/gnss/rover/fix_corrected", NavSatFix, queue_size=10)

        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.coordinate_corrector) # /gnss/rover/fix

    

    def coordinate_corrector(self, data):
        """ 
            Makes the coordinate corrections comes from
            the reference stations.
            
            Publishes corrected lat, long, alt
        """

        # Take the correction values from Reference Station class
        # [lat, long, alt]
        correction_values = reference.difference_calculator()
        self.rover_nav = NavSatFix()
        self.rover_nav.header.stamp = self.current_time
        self.rover_nav.header.frame_id = "base_link"
        self.rover_nav.latitude = data.latitude - correction_values[0]
        self.rover_nav.longitude = data.longitude - correction_values[1]
        self.rover_nav.altitude = data.altitude - correction_values[2]
        self.rover_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        self.rover_coord_pub.publish(self.rover_nav)


if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            ReferenceStation()
            RoverStation()

    except rospy.ROSInterruptException:
        pass
