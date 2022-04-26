
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>


GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

GeographicLib::LocalCartesian locart;

double x, y, z;

int counter = 1;

ros::Publisher gps_odometry_pub;
ros::Publisher gps_odometry_vel_pub;

nav_msgs::Odometry gps_odom_msg;


void ref_mean_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    GeographicLib::LocalCartesian locart(0, 0, 0, earth);

    locart.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // std::cout << " Mean :  -- X : " << x
    //         << " -- Y : " << y <<
    //         " -- Z : " << z << 
    //         std::endl;

};

void ref_error_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    GeographicLib::LocalCartesian locart(0, 0, 0, earth);

    locart.Forward(msg->data[0], msg->data[1], msg->data[2], x, y, z);

    // double displacement = std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
    double eucledian_error = std::sqrt(std::pow(x,2) + std::pow(y,2));

    std::cout << " Displacement in XY: " << eucledian_error << " m " << std::endl;

    std::cout << " Error :  -- X : " << x
            << " -- Y : " << y <<
            " -- Z : " << z << 
            std::endl;

    //std::cout << "----------" << std::endl;

};

void rover_corrected_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    GeographicLib::LocalCartesian locart(0, 0, 0, earth);

    locart.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // std::cout << " Rover Corrected :  -- X : " << x
    //         << " -- Y : " << y <<
    //         " -- Z : " << z << 
    //         std::endl;

    //std::cout << "----------" << std::endl;

};


void gps_odometry_publisher_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (counter==1)
    {
        locart=GeographicLib::LocalCartesian(msg->latitude, msg->longitude, msg->altitude, earth);
    }
    counter++;
    locart.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    gps_odom_msg.header.stamp = ros::Time::now();
    gps_odom_msg.header.frame_id = "base_link";
    gps_odom_msg.pose.pose.position.x = x;
    gps_odom_msg.pose.pose.position.y = y;
    gps_odom_msg.pose.pose.position.z = z;
    gps_odom_msg.pose.pose.orientation.x = 0;  // take this from IMU
    gps_odom_msg.pose.pose.orientation.y = 0;  // take this from IMU
    gps_odom_msg.pose.pose.orientation.z = 0;  // take this from IMU
    gps_odom_msg.pose.pose.orientation.w = 1;  // take this from IMU
    // gps_odom_msg.pose.covariance = msg->position_covariance;
    // {
    //     0, 0, 0,
    //     0, 0, 0,
    //     0, 0, 0
    // };
    // gps_odometry_pub.publish(gps_odom_msg);

}



void gps_odometry_vel_publisher_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
    gps_odom_msg.twist.twist.angular.x = msg->twist.twist.angular.x;
    gps_odom_msg.twist.twist.angular.y = msg->twist.twist.angular.y;
    gps_odom_msg.twist.twist.angular.z = msg->twist.twist.angular.z;
    gps_odom_msg.twist.twist.linear.x = msg->twist.twist.linear.x;
    gps_odom_msg.twist.twist.linear.y = msg->twist.twist.linear.y;
    gps_odom_msg.twist.twist.linear.z = msg->twist.twist.linear.z;
    gps_odom_msg.twist.covariance = msg->twist.covariance;
    gps_odometry_pub.publish(gps_odom_msg);

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "local_cartesian");
    ros::NodeHandle n;
    
    ros::Publisher local_cartesian_pub = n.advertise<sensor_msgs::NavSatFix>("local_cartesian_publisher", 10);
    gps_odometry_pub = n.advertise<nav_msgs::Odometry>("/odometry/gps", 10);

    ros::Rate loop_rate(10);

    
    // ros::Subscriber ref_mean_sub = n.subscribe("/gnss/ref/fix_mean", 1000, ref_mean_callback);
    // ros::Subscriber ref_error_sub = n.subscribe("/gnss/ref/error", 1000, ref_error_callback);
    // ros::Subscriber rover_corrected_sub = n.subscribe("/gnss/rover/fix_corrected", 1000, rover_corrected_callback);
    ros::Subscriber gps_odometry_sub = n.subscribe("/ublox_gps/fix", 1000, gps_odometry_publisher_callback);
    ros::Subscriber gps_odometry_vel_sub = n.subscribe("/ublox_gps/fix_velocity", 1000, gps_odometry_vel_publisher_callback);

    ros::spin();


    return 0;


}