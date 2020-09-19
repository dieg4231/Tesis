#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/surface/convex_hull.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <string>
#include <kalman_loc_qr/Landmarks.h>
#include <kalman_loc_qr/ArucoMasks.h>
#include <kalman_loc_qr/SaveQRMap.h>

#include <ros/package.h>
#include <tf/transform_listener.h>

double max_dist = 100;
std::string max_dist_string;
std::string::size_type sz;     // alias of size_t

std::vector<std::vector<geometry_msgs::PointStamped>> points_acumulado;
std::vector<int> ids_acumulado;

geometry_msgs::PointStamped from_device2map(geometry_msgs::PointStamped device)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::PointStamped map_point;
  tf::Vector3 v(device.point.x, device.point.y, device.point.z);
  
  try{
    listener.waitForTransform("/map", "kinect_link", ros::Time(0), ros::Duration(3.0));
    //listener.transformPoint("/map", ros::Time(0), device,"/camera_depth_optical_frame",map_point );
    listener.lookupTransform("/map", "kinect_link",ros::Time(0), transform);
    
    v = transform * v;
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  map_point.point.x = v.x();//device.x + transform.getOrigin().x();
  map_point.point.y = v.y();//device.y + transform.getOrigin().y();
  map_point.point.z = v.z();//device.z + transform.getOrigin().z();

  ROS_INFO("X: %f Y: %f Z: %f ",map_point.point.x,map_point.point.y,map_point.point.z);

  return map_point;

}

double distance2landmak(geometry_msgs::PointStamped landmark)
{
  return sqrt( pow(landmark.point.x,2) + pow(landmark.point.y,2) + pow(landmark.point.z,2) );
}

void 
add_points (const kalman_loc_qr::LandmarksConstPtr& msg)
{ 
  
  for(int i = 0; i < msg->ids.size(); i++) // Todos los ids recibidos
  { 
    if( distance2landmak( msg->pointLandmarks[i] ) > max_dist )
        continue;


    std::vector<int>::iterator it =  std::find(ids_acumulado.begin(), ids_acumulado.end(), msg->ids[i]);
    if( it != ids_acumulado.end() ) //Si se encuentra el id i en la lista 
    {   // se agregan sus puntos al acumulado
        
        int index = it - ids_acumulado.begin();
       
        points_acumulado[index].push_back( from_device2map( msg->pointLandmarks[i]) );
    }
    else  // se grega el nuevo id
    {
      ids_acumulado.push_back(msg->ids[i]);
      std::vector<geometry_msgs::PointStamped> v_aux;
      v_aux.push_back(from_device2map(msg->pointLandmarks[i]));
      points_acumulado.push_back( v_aux );
      std::cout << "nuevo: " << msg->ids[i] << " \n";  
    }
    
  }
    std::cout << "----------- \n";
}

bool 
save_map(kalman_loc_qr::SaveQRMap::Request &req,
         kalman_loc_qr::SaveQRMap::Response &res)
{
  long  double x,y,z;
  int j;

  std::string path2pack = ros::package::getPath("kalman_loc_qr");
  std::ofstream myfile;
  myfile.open ( req.fileName+".txt");
  std::cout << "Writing to: \n ROS_HOME/" << req.fileName+".txt \n";
  
  for(int i = 0; i < ids_acumulado.size(); i++) // Todos los ids recibidos
  {
    j = x = y = z = 0;
    for(j = 0; j < points_acumulado[i].size(); j++)
    {
      x += points_acumulado[i][j].point.x;
      y += points_acumulado[i][j].point.y;
      z += points_acumulado[i][j].point.z;
    }

    x /= j;
    y /= j;
    z /= j;
    myfile <<  ids_acumulado[i] << " " << x << " " << y << " " << z <<"\n";
  }

  myfile.close();
  res.success = true;
  return true;
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "aruco_mapping_node");
  ros::NodeHandle nh;

  if(ros::param::has("~max_dist"))
  {
    
    ros::param::get("~max_dist", max_dist );
    //max_dist = atof(max_dist_string.c_str());
    if( max_dist < 0)
    {
      return 0;
    }
    std::cout << "max_dist: " <<  max_dist  << std::endl;
   
  }
  else
  {
    std::cout << "Sorry U_U give a  max_dist. Try: _max_dist:=5.0 "  << max_dist << std::endl;
    return 0;
  }

  ros::Subscriber landmarks_sub = nh.subscribe ("/landmarksPoint", 1, add_points);
  ros::ServiceServer save_service = nh.advertiseService("get_qr_map", save_map);    

  ros::spin ();
}
