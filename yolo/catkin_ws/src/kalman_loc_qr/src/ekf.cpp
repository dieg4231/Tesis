#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
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



std::vector<std::vector<geometry_msgs::Point>> points_acumulado;
std::vector<int> ids_acumulado;

geometry_msgs::Point from_device2map(geometry_msgs::Point device)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  try{
    listener.lookupTransform("/map", "/kinect",
                             ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  geometry_msgs::Point map_point;

  map_point.x = device.x + transform.getOrigin().x();
  map_point.y = device.y + transform.getOrigin().y();
  map_point.z = device.z + transform.getOrigin().z();
  
  return map_point;

}

void 
add_points (const kalman_loc_qr::LandmarksConstPtr& msg)
{ 
  
  for(int i = 0; i < msg->ids.size(); i++) // Todos los ids recibidos
  {
    std::vector<int>::iterator it =  std::find(ids_acumulado.begin(), ids_acumulado.end(), ids_acumulado[i]);
    if( it != ids_acumulado.end() ) //Si se encuentra el id i en la lista 
    {   // se agregan sus puntos al acumulado
        
        int index = it - ids_acumulado.begin();
       
        points_acumulado[index].push_back( msg->pointLandmarks[i] );
    }
    else  // se grega el nuevo id
    {
      ids_acumulado.push_back(msg->ids[i]);
      std::vector<geometry_msgs::Point> v_aux;
      v_aux.push_back(msg->pointLandmarks[i]);
      points_acumulado.push_back( v_aux );
    }
  }
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
  std::cout << "Writing to: \n" << path2pack+"/data/landmarks_"+req.fileName+".txt \n";
  
  for(int i = 0; i < ids_acumulado.size(); i++) // Todos los ids recibidos
  {
    j = x = y = z = 0;
    for(j = 0; j < points_acumulado[i].size(); j++)
    {
      x += points_acumulado[i][j].x;
      y += points_acumulado[i][j].y;
      z += points_acumulado[i][j].z;
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
  ros::init (argc, argv, "ekf");
  ros::NodeHandle nh;


  ros::Subscriber landmarks_sub = nh.subscribe ("/landmarksPoint", 1, add_points);
  ros::ServiceServer save_service = nh.advertiseService("get_qr_map", save_map);    

  ros::spin ();
}