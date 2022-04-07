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
        
        rospy.init_node("gnss_business")
        
        # Publisher initialization part for ReferenceStation
        self.ref_coord_pub = rospy.Publisher("/gnss/ref/fix_mean", NavSatFix, queue_size=10)
        self.ref_coord_err_pub = rospy.Publisher("/gnss/ref/error", Float64MultiArray, queue_size=1)
        

        # Mean values
        self.received_number = 1
        self.init_coords = [
            0.0, # self.mean_lat
            0.0, # self.mean_long
            0.0  # self.mean_alt
        ]
        
        
    def mean_update_and_calculate_error(self, data):
        
        
        self.instant_coords = [
            data.latitude, # in degrees
            data.longitude, # in degrees
            data.altitude # in meters
            ]
        print("instant: " + str(self.instant_coords))

        self.mean_coords = [
            (self.mean_coords[0] + self.instant_coords[0]) / self.received_number,
            (self.mean_coords[1] + self.instant_coords[1]) / self.received_number,
            (self.mean_coords[2] + self.instant_coords[2]) / self.received_number
        ]
        print("mean: " + str(self.mean_coords))
        
        self.error_list = [
            self.mean_coords[0] - self.instant_coords[0], # reference mean lat - reference instant lat,
            self.mean_coords[1] - self.instant_coords[1], # reference mean long - reference instant long,
            self.mean_coords[2] - self.instant_coords[2], # reference mean alt - reference instant alt
        ]
        print("error: " + str(self.error_list))
        print("----------------")
        print("----------------")
        
        # counter = 0
        # for coord in self.instant_coords:
        #     #self.mean_coords[counter] = (self.mean_coords[counter] + coord) / self.received_number
        #     self.error_list.append(self.mean_coords[counter] - coord)
        #     counter+=1
        
        self.received_number+=1
        
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

        #rospy.init_node("rover_coord_publisher")
        

        self.rover_coord_pub = rospy.Publisher("/gnss/rover/fix_corrected", NavSatFix, queue_size=10)
    

    def coordinate_corrector(self, data):
        """ 
            Makes the coordinate corrections comes from
            the reference stations.
            
            Publishes corrected lat, long, alt
        """

        # Take the correction values from Reference Station class
        # [lat, long, alt]
        self.rover_nav = NavSatFix()
        self.current_time = current_time
        self.rover_nav.header.stamp = self.current_time
        self.rover_nav.header.frame_id = "base_link"
        self.rover_nav.latitude = data.latitude - data.data[0]
        self.rover_nav.longitude = data.longitude - data.data[1]
        self.rover_nav.altitude = data.altitude - data.data[2]
        self.rover_nav.position_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        
        self.rover_coord_pub.publish(rover_nav)






if __name__ == "__main__":
    try:
        
        # Object registrations
        reference_station = ReferenceStation()
        rover_station = RoverStation()
        
        
        while not rospy.is_shutdown():
            
            # Time
            current_time = rospy.Time.now()
            
            # Subscriber part for RoverStation
            rospy.Subscriber("/ublox_gps/fix", NavSatFix, reference_station.mean_update_and_calculate_error)  
            rospy.Subscriber("/gnss/ref/error", NavSatFix, rover_station.coordinate_corrector)
            
            
    except rospy.ROSInterruptException:
        pass
