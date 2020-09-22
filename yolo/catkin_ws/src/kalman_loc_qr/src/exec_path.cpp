#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


nav_msgs::Path path;

bool active = true;
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(active)
    {
        path = *msg;
        active = false;
    }
}


nav_msgs::Odometry robot_odom;

void odomCallback(const nav_msgs::Odometry& msg)
{
	robot_odom = msg ;
}

double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "excec_path");
  ros::NodeHandle n;
  ros::Publisher turtle_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber map_sub = n.subscribe("path",10,pathCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

  
  geometry_msgs::PoseStamped sub_goal;

  ros::Rate rate(10.0);

  geometry_msgs::Twist vel_msg;


  int index = 0;
  double distance_to_path;
  
  while (n.ok())
  {  
    if(!active)
    {
        if(path.poses.size() != index)
        {
            distance_to_path = sqrt(   pow(path.poses[index].pose.position.x - robot_odom.pose.pose.position.x,2) + pow(path.poses[index].pose.position.y - robot_odom.pose.pose.position.y,2)    );
            if(  distance_to_path > 0.1 )
            {
                
                vel_msg.angular.z = 1.5 * normalize( atan2(path.poses[index].pose.position.y - robot_odom.pose.pose.position.y,
                                                path.poses[index].pose.position.x - robot_odom.pose.pose.position.x) - getYawFromQuaternion( robot_odom.pose.pose.orientation)   ) ;
                vel_msg.linear.x = 1* distance_to_path;
                std::cout << "Aun no llego a:\n" << path.poses[index].pose.position  << std::endl;
                std::cout << "angular:\n" << atan2(path.poses[index].pose.position.y - robot_odom.pose.pose.position.y,
                                                path.poses[index].pose.position.x - robot_odom.pose.pose.position.x)  << std::endl;
                std::cout << "Lineal:\n" << distance_to_path << std::endl;
                
            }else
            {  
                index++;
            }
        }
        else 
        {
            vel_msg.angular.z = 0.0;
            vel_msg.linear.x = 0.0;
        }
        


        turtle_vel.publish(vel_msg);
        
    }  
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}