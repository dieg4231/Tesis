// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>



double getYawFromQuaternion(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw; 
	tf2::Quaternion quat_tf;
	tf2::fromMsg(quat, quat_tf);
	tf2::Matrix3x3 m( quat_tf );
	m.getRPY(roll, pitch, yaw);
	//yaw = atan2( (float)yaw );
	return yaw;
}

void OdomCallback(const nav_msgs::Odometry& msg)
{
    std::cout <<  msg.pose.pose.position << "\n";
    std::cout <<  "theta:" << getYawFromQuaternion(msg.pose.pose.orientation )<< "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_readable");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 0, OdomCallback);
    ros::Rate r(10.0);
    
    ros::spin();

    ROS_INFO_STREAM("Shutting down.");

    return 0;
}




