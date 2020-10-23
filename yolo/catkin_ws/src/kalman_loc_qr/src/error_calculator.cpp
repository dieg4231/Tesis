#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <std_srvs/SetBool.h>

#include <fstream>


nav_msgs::Path path_ekf;
nav_msgs::Path path_odom;

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

bool dataCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{

    std::cout << "BBBBBBBBBBBBBBBBBBBBBBB NUmero de datos: " << path_ekf.poses.size() << "\n";
    std::ofstream myfile;
    myfile.open ( "data.txt");
    std::cout << "Writing to: \n ROS_HOME/data.txt \n";

  for(int i = 0 ; i < path_ekf.poses.size();i++)
  {
      myfile <<   path_ekf.poses[i].pose.position.x << "\t" << path_odom.poses[i].pose.position.x << 
             "\t" << path_ekf.poses[i].pose.position.y << "\t" << path_odom.poses[i].pose.position.y << 
             "\t" << getYawFromQuaternion(path_ekf.poses[i].pose.orientation) << "\t" << getYawFromQuaternion(path_odom.poses[i].pose.orientation) << std::endl;
  }
  res.message = "Enterado";


  
  myfile.close();

  return true;
}

nav_msgs::Odometry robot_odom;

void odomCallback(const nav_msgs::Odometry& msg)
{
	robot_odom = msg ;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "excec_path");
    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("odom", 0, odomCallback);
    ros::Publisher path_ekf_pub = n.advertise<nav_msgs::Path>("path_ekf",10);
    ros::Publisher path_odom_pub = n.advertise<nav_msgs::Path>("path_odom",10);
    ros::ServiceServer get_data = n.advertiseService("error_get_data",dataCallBack);  

    ros::Rate rate(0.5);

    geometry_msgs::TransformStamped TRANSFORM_STAMPED_MAP_2_BASE ;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //nav_msgs::Path path_ekf;
    //nav_msgs::Path path_odom;

    path_odom.header.stamp    = path_ekf.header.stamp    = ros::Time::now();
    path_odom.header.frame_id = path_ekf.header.frame_id = "map";

    while (n.ok())
    { 

        std::cout << "Esperando odom topic...\n";
        while(odom_sub.getNumPublishers() == 0 || robot_odom.pose.pose.orientation.w == 0)
        {
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        };
        
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
        geometry_msgs::PoseStamped pose_ekf;
    
        pose_ekf.header.stamp = ros::Time::now();
        pose_ekf.header.frame_id = "map";

        pose_ekf.pose.position.x = TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x;
        pose_ekf.pose.position.y = TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y;
        pose_ekf.pose.position.z = TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.z;
        pose_ekf.pose.orientation = TRANSFORM_STAMPED_MAP_2_BASE.transform.rotation;

        path_ekf.poses.push_back(pose_ekf);
        //

        geometry_msgs::PoseStamped pose_odom;
    
        pose_odom.header.stamp = ros::Time::now();
        pose_odom.header.frame_id = "map";

        pose_odom.pose = robot_odom.pose.pose; 

        path_odom.poses.push_back(pose_odom);

        path_ekf_pub.publish(path_ekf);
        path_odom_pub.publish(path_odom);
        
    
        ros::spinOnce();
        rate.sleep();
    }


  return 0;
}