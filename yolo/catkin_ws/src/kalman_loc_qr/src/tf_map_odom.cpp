#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


nav_msgs::Odometry ROBOT_ODOM;
geometry_msgs::PoseWithCovariance  LOCALIZATION_POSE;

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
	return normalize(yaw);
}

void OdomCallback(const nav_msgs::Odometry& msg)
{
	ROBOT_ODOM = msg ;
}

void LocalizationEkfCallback(const geometry_msgs::PoseWithCovariance& msg)
{
    LOCALIZATION_POSE = msg;
}




int main(int argc, char *argv[])
{

	ros::init(argc, argv, "Nodo_ekf_tf");
	ros::NodeHandle nh;
    ros::Subscriber localization_ekf_sub = nh.subscribe("localization_ekf", 0, LocalizationEkfCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 0, OdomCallback);

    int32_t publish_rate_ = 100;
    tf2_ros::TransformBroadcaster tf_br_;
    geometry_msgs::TransformStamped tf_map_to_odom_;


    // set up parent and child frames
    tf_map_to_odom_.header.frame_id = std::string("map");
    tf_map_to_odom_.child_frame_id = std::string("odom");

    // set up publish rate
    ros::Rate loop_rate(publish_rate_);

    while(localization_ekf_sub.getNumPublishers() == 0)
	{
		loop_rate.sleep();
        ros::spinOnce();
	}

    while(odom_sub.getNumPublishers() == 0)
	{
		loop_rate.sleep();
        ros::spinOnce();
	}
	

    double incremento = 0;
    // main loop
    while (ros::ok())
    {
        // time stamp
        tf_map_to_odom_.header.stamp = ros::Time::now();

        // specify actual transformation vectors from odometry
        // NOTE: zeros have to be substituted with actual variable data
        tf_map_to_odom_.transform.translation.x = LOCALIZATION_POSE.pose.position.x - ROBOT_ODOM.pose.pose.position.x;
        tf_map_to_odom_.transform.translation.y = LOCALIZATION_POSE.pose.position.y - ROBOT_ODOM.pose.pose.position.y;
        tf_map_to_odom_.transform.translation.z = 0;
   

        //std::cout << getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation  )<< "\n";
        tf2::Quaternion q;
        q.setRPY(0, 0, normalize( getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation ) - getYawFromQuaternion( ROBOT_ODOM.pose.pose.orientation ) ) );
        q.normalize();
        tf_map_to_odom_.transform.rotation.x = q.x();
        tf_map_to_odom_.transform.rotation.y = q.y();
        tf_map_to_odom_.transform.rotation.z = q.z();
        tf_map_to_odom_.transform.rotation.w = q.w();

        incremento += .01;
        
        
        // broadcast transform
        tf_br_.sendTransform(tf_map_to_odom_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
