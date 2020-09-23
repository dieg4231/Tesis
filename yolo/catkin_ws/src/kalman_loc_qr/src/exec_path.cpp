#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>


nav_msgs::Path path;
geometry_msgs::TransformStamped TRANSFORM_STAMPED_MAP_2_BASE ;

bool active = true;


void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    std::cout << "Recividooo " << std::endl;
    if(active)
    {
        path = *msg;
        active = false;
    }
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
  ros::Subscriber map_sub = n.subscribe("path",0,pathCallback);

  
  geometry_msgs::PoseStamped sub_goal;

  ros::Rate rate(10.0);

  geometry_msgs::Twist vel_msg;


  int index = 0;
  double distance_to_path;
  
  tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

  while (n.ok())
  { 
    /*
		Get map to base_link trasformation
	*/

	
        

        	
    try{
        TRANSFORM_STAMPED_MAP_2_BASE = tfBuffer.lookupTransform( "map","base_link",
                                ros::Time(0));
        //ROS_INFO("TF_base_to_device: \n X: %f \n Y: %f \n Z: %f \n -----\n ",TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x,
        //																	TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y,
        //																	TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.z);	
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }  
    //
    if(!active)
    {
        if(path.poses.size() != index)
        {
            distance_to_path = sqrt(   pow(path.poses[index].pose.position.x - TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x,2) + pow(path.poses[index].pose.position.y - TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y,2)    );
            if(  distance_to_path > 0.1 )
            {
                
                vel_msg.angular.z = 1.3 * normalize( atan2(path.poses[index].pose.position.y - TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y,
                                                path.poses[index].pose.position.x - TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x) - getYawFromQuaternion( TRANSFORM_STAMPED_MAP_2_BASE.transform.rotation)   ) ;
                
                vel_msg.linear.x = 1* distance_to_path;
                
                std::cout << "Aun no llego a:\n" << path.poses[index].pose.position  << std::endl;
                std::cout << "angular:\n" << vel_msg.angular.z  << std::endl;
                std::cout << "Lineal:\n" << vel_msg.linear.x << std::endl;
                
            }else
            {  
                index++;
            }
        }
        else 
        {
            index = 0;
            active = true;
            vel_msg.angular.z = 0.0;
            vel_msg.linear.x = 0.0;
        }
        


        turtle_vel.publish(vel_msg);
        
    }  
    //std::cout << "Activ : " << active << std::endl;
   

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}