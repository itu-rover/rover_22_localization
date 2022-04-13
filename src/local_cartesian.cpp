
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>


GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

double x, y, z;


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




int main(int argc, char **argv)
{

    ros::init(argc, argv, "local_cartesian");
    ros::NodeHandle n;
    
    ros::Publisher local_cartesian_pub = n.advertise<sensor_msgs::NavSatFix>("local_cartesian_publisher", 10);

    ros::Rate loop_rate(10);

    
    ros::Subscriber ref_mean_sub = n.subscribe("/gnss/ref/fix_mean", 1000, ref_mean_callback);
    ros::Subscriber ref_error_sub = n.subscribe("/gnss/ref/error", 1000, ref_error_callback);
    ros::Subscriber rover_corrected_sub = n.subscribe("/gnss/rover/fix_corrected", 1000, rover_corrected_callback);

    ros::spin();


    return 0;


}