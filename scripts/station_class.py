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
        
        
        # Publisher initialization part for ReferenceStation
        self.ref_coord_pub = rospy.Publisher("/gnss/ref/fix_mean", NavSatFix, queue_size=10)
        self.ref_coord_err_pub = rospy.Publisher("/gnss/ref/error", Float64MultiArray, queue_size=10)
        

        # Mean values/gnss/ref/fix_mean/gnss/ref/fix_mean/gnss/ref/fix_mean
        self.received_message = 1
        self.mean_coords = [
            0.0, # self.mean_lat
            0.0, # self.mean_long
            0.0  # self.mean_alt
        ]
        
        
    def mean_update_and_calculate_error(self, data):
        """
            Gets the instant coordinates of reference station.
            
            
        """
        
        self.ref_instant_coords = [
            data.latitude, # in degrees
            data.longitude, # in degrees
            data.altitude # in meters
            ]
        print("reference instant: " + str(self.ref_instant_coords))

        self.mean_coords = [
            (self.mean_coords[0]*(self.received_message-1) + self.ref_instant_coords[0]) / self.received_message,
            (self.mean_coords[1]*(self.received_message-1) + self.ref_instant_coords[1]) / self.received_message,
            (self.mean_coords[2]*(self.received_message-1) + self.ref_instant_coords[2]) / self.received_message
        ]
        print("reference mean: " + str(self.mean_coords))
        print("received_message: " + str(self.received_message))
        
        self.error_list = [
            self.mean_coords[0] - self.ref_instant_coords[0], # reference mean lat - reference instant lat,
            self.mean_coords[1] - self.ref_instant_coords[1], # reference mean long - reference instant long,
            self.mean_coords[2] - self.ref_instant_coords[2], # reference mean alt - reference instant alt
        ]
        print("reference error: " + str(self.error_list))
        print("----------------")
        print("----------------")
        
        # counter = 0
        # for coord in self.instant_coords:
        #     #self.mean_coords[counter] = (self.mean_coords[counter] + coord) / self.received_number
        #     self.error_list.append(self.mean_coords[counter] - coord)
        #     counter+=1
        
        self.received_message+=1
        
        # Create NavSatFix message
        self.ref_nav = NavSatFix()
        self.ref_nav.header.stamp = current_time
        self.ref_nav.latitude = self.mean_coords[0]
        self.ref_nav.longitude = self.mean_coords[1]
        self.ref_nav.altitude = self.mean_coords[2]
        self.ref_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        
        # Create Float64MultiArray message
        self.ref_coord_err = Float64MultiArray()
        self.ref_coord_err.data = self.error_list
        
        # Publish
        self.ref_coord_err_pub.publish(self.ref_coord_err)
        self.ref_coord_pub.publish(self.ref_nav)







class RoverStation():

    def __init__(self):
        """ 
            
        """

        self.corrected_rover_nav = NavSatFix()
        self.instant_rover_nav = NavSatFix()
        

        self.rover_coord_pub = rospy.Publisher("/gnss/rover/fix_corrected", NavSatFix, queue_size=10)
        
    def rover_instant_position(self, data):
        """ 
            Gets the instant position data of rover.
        
            Updates the created NavSatFix msg for the
            instant coordinates.
        """
        self.instant_rover_nav.header.stamp = current_time
        self.instant_rover_nav.header.frame_id = ""
        self.instant_rover_nav.latitude = data.latitude
        self.instant_rover_nav.longitude = data.longitude
        self.instant_rover_nav.altitude = data.altitude
        self.instant_rover_nav.position_covariance = data.position_covariance
    

    def coordinate_corrector(self, data):
        """ 
            Gets the error values comes from reference station.
        
            Makes the coordinate corrections comes from
            the reference stations.
            
            Publishes corrected lat, long, alt
        """

        # Take the correction values from Reference Station class
        # [lat, long, alt]
        self.corrected_rover_nav.header.stamp = current_time
        self.corrected_rover_nav.header.frame_id = "base_link"
        self.corrected_rover_nav.latitude = self.instant_rover_nav.latitude - data.data[0]
        self.corrected_rover_nav.longitude = self.instant_rover_nav.longitude - data.data[1]
        self.corrected_rover_nav.altitude = self.instant_rover_nav.altitude - data.data[2]
        self.corrected_rover_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        
        self.rover_coord_pub.publish(self.corrected_rover_nav)






if __name__ == "__main__":
    
    try:
        # Initialize Node
        rospy.init_node("gnss_business")
        
        # Time
        current_time = rospy.Time.now()
        
        # Object registrations
        reference_station = ReferenceStation()
        rover_station = RoverStation()
        
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, reference_station.mean_update_and_calculate_error)  
        rospy.Subscriber("/fix", NavSatFix, rover_station.rover_instant_position)
        rospy.Subscriber("/gnss/ref/error", Float64MultiArray, rover_station.coordinate_corrector)
        
        rospy.spin()
            
            
    except rospy.ROSInterruptException:
        pass
