// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <kalman_loc_qr/Landmarks.h>

// tf

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// Viz message
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++
#include <string>
#include <vector>

geometry_msgs::TransformStamped TRANSFORM_STAMPED_BASE_2_DEVICE ;

ros::Publisher vis_qr_map ;
ros::Publisher vis_qr_map_titles;

visualization_msgs::Marker landmarks;
visualization_msgs::MarkerArray landmarks_titles;
visualization_msgs::Marker title;
geometry_msgs::Point point;
std_msgs::ColorRGBA color; 


void 
landmarksPointCallBack (const kalman_loc_qr::LandmarksConstPtr& msg)
{ 
    landmarks.points.clear();
    landmarks.colors.clear();
    landmarks_titles.markers.clear();

    for(int i = 0; i < msg->ids.size(); i++) // Todos los ids recibidos
    {
     
        point.x = msg->pointLandmarks[i].point.x;
        point.y = msg->pointLandmarks[i].point.y;
        point.z = msg->pointLandmarks[i].point.z;

        point.x += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.x;
		point.y += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.y;
		point.z += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.z;

        title.text = "id: "+ std::to_string(msg->ids[i]);
        title.pose.position.x = msg->pointLandmarks[i].point.x;
        title.pose.position.y = msg->pointLandmarks[i].point.y;
        title.pose.position.z = msg->pointLandmarks[i].point.z + .3;
        
        //ROS_INFO(" x : %f y: %f z: %f", point.x, point.y, point.z );
        
        title.id = msg->ids[i];
        landmarks_titles.markers.push_back(title);

        landmarks.points.push_back(point);
        landmarks.colors.push_back(color);
    }
    vis_qr_map.publish( landmarks );
    vis_qr_map_titles.publish( landmarks_titles );

}

int main(int argc, char** argv)
{
   
    color.r = 155;
    color.g = 200;
    color.b = 12;
    color.a = 1;
    landmarks.color = color;


    landmarks.header.frame_id = "base_link";// "camera_link"; //
    landmarks.header.stamp = ros::Time();
    landmarks.ns = "my_namespace2";
    landmarks.id = 0;
    landmarks.type = visualization_msgs::Marker::SPHERE_LIST;
    landmarks.action = visualization_msgs::Marker::ADD;
    landmarks.lifetime = ros::Duration(.1);
    landmarks.pose.orientation.x = 0.0;
    landmarks.pose.orientation.y = 0.0;
    landmarks.pose.orientation.z = 0.0;
    landmarks.pose.orientation.w = 1.0;
    landmarks.scale.x = .15;
    landmarks.scale.y = .15;
    landmarks.scale.z = .15;


    title.header.frame_id = "base_link"; // "camera_link"; //
    title.header.stamp = ros::Time();
    //title.ns = "my_namespace3";
    title.id = 0;
    title.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    title.action = visualization_msgs::Marker::ADD;
    title.lifetime = ros::Duration(.1);
    
    title.scale.z = .15;
    
    title.color.r = 0;
    title.color.g = 0;
    title.color.b = 0;
    title.color.a = 1;

    ros::init(argc, argv, "landmarks_found_node");
    ros::NodeHandle nh;

    ros::Subscriber landmarks_sub = nh.subscribe ("landmarksPoint", 1, landmarksPointCallBack);
    vis_qr_map = nh.advertise<visualization_msgs::Marker>( "marker_recognized", 0 );
    vis_qr_map_titles = nh.advertise<visualization_msgs::MarkerArray>( "marker_recognized_titles", 0 );
  
    ros::Rate r(10.0);

    tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);

	
	while (nh.ok()){
		
		try{
		TRANSFORM_STAMPED_BASE_2_DEVICE = tfBuffer.lookupTransform( "base_link","kinect_link",
								ros::Time(0));
		ROS_INFO("TF_base_to_device: \n X: %f \n Y: %f \n Z: %f \n -----\n ",TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.x,
																			TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.y,
																			TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.z);	
		break;

		}
		catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
		}    
		r.sleep();
	}


    ros::spin(); 


  return 0;
}
