// ROS
#include <ros/package.h>
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++
#include <string>
#include <vector>
#include <fstream>

std::string mapfile;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_rviz_node");
  ros::NodeHandle n;
    
    if(ros::param::has("~mapfile"))
  {
    
    ros::param::get("~mapfile", mapfile);
    
  }
  else
  {
    std::cout << "Sorry U_U give a device name_file. Try: _mapfile:=soyElMapa.txt"  << std::endl;
    return 0;
  }
    std::string path2pack = ros::package::getPath("kalman_loc_qr");
    std::ifstream infile(mapfile);

    double x,y,z;
    int id;

    visualization_msgs::Marker landmarks;
    visualization_msgs::MarkerArray landmarks_titles;
    visualization_msgs::Marker title;
    geometry_msgs::Point point;

    std_msgs::ColorRGBA color;  

    color.r = 55;
    color.g = 20;
    color.b = 172;
    color.a = 1;
    landmarks.color = color;


    landmarks.header.frame_id = "map";
    landmarks.header.stamp = ros::Time();
    landmarks.ns = "my_namespace2";
    landmarks.id = 0;
    landmarks.type = visualization_msgs::Marker::SPHERE_LIST;
    landmarks.action = visualization_msgs::Marker::ADD;
    landmarks.lifetime = ros::Duration(1);
    landmarks.pose.orientation.x = 0.0;
    landmarks.pose.orientation.y = 0.0;
    landmarks.pose.orientation.z = 0.0;
    landmarks.pose.orientation.w = 1.0;
    landmarks.scale.x = .15;
    landmarks.scale.y = .15;
    landmarks.scale.z = .15;


    title.header.frame_id = "map";
    title.header.stamp = ros::Time();
    //title.ns = "my_namespace3";
    title.id = 0;
    title.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    title.action = visualization_msgs::Marker::ADD;
    title.lifetime = ros::Duration(1);
    
    title.scale.z = .15;
    
    title.color.r = 0;
    title.color.g = 0;
    title.color.b = 0;
    title.color.a = 1;

    while (infile >> id >> x >> y >> z)
    {   ROS_INFO("Id: %d",id );
    
        point.x = x;
        point.y = y;
        point.z = z;

        title.text = "id: "+ std::to_string(id);
        title.pose.position.x = x;
        title.pose.position.y = y;
        title.pose.position.z = z + .3;
        title.id = id;
        landmarks_titles.markers.push_back(title);

        landmarks.points.push_back(point);
        landmarks.colors.push_back(color);

    }

  ros::Publisher vis_qr_map = n.advertise<visualization_msgs::Marker>( "map_marker", 0 );
  ros::Publisher vis_qr_map_titles = n.advertise<visualization_msgs::MarkerArray>( "map_marker_titles", 0 );
  

  ros::Rate r(10.0);
  
  while(n.ok())
  {
    

    vis_qr_map.publish( landmarks );
    vis_qr_map_titles.publish( landmarks_titles );
 
    ros::spinOnce(); 
    r.sleep();

  }

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}




