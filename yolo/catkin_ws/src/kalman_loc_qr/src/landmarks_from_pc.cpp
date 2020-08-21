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


ros::Publisher pub_cloud;
ros::Publisher pub_landmarks;

pcl::PointIndices::Ptr indx(new pcl::PointIndices ());;
std::string device;

std::vector<unsigned int> separators;
std::vector<unsigned int> ids;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 
  pcl::PointIndices::Ptr indx2(new pcl::PointIndices ());
  std::vector<unsigned int> separators2;
  std::vector<unsigned int> ids2;

  *indx2 = *indx;
  separators2 = separators;
  ids2 = ids;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_pcl_filtered(new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_plane_polygon (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_roi (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::fromROSMsg(*cloud_msg,*cloud_pcl);

  pcl::ExtractIndices<pcl::PointXYZ> filter;
  filter.setInputCloud (cloud_pcl);
  filter.setIndices (indx2);

  // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
  filter.setKeepOrganized (true);
  filter.filter (*cloud_roi);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  
  pcl::toROSMsg(*cloud_roi,output);
  
  std_msgs::Header header;
 
  if(device.compare("kinect") == 0 )
  {

    header.frame_id = "camera_depth_optical_frame"; //Kinect
   
  }
  else if( device.compare("zed_stereo") == 0)
  {

    header.frame_id = "zed_left_camera_frame"; // ZED

  }
  else
  {
    std::cout << "You mustn't be here" << device << std::endl;
  }

  output.header = header;
  header.stamp = ros::Time::now();

  //Publish the data
  pub_cloud.publish (output);


  kalman_loc_qr::Landmarks landmarks;
  landmarks.header = header;

  for(int i = 0; i < ids2.size() ; i++)
  {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for( int j =  i == 0 ? 0 : separators2[i-1]; j < separators2[i] ; j++)//   cloud_roi->points.begin() ; it < cloud_roi->points.end(); it++)
    {
     if( !isnan(cloud_roi->points[indx2->indices[j]].x) && !isnan(cloud_roi->points[indx2->indices[j]].y) && !isnan(cloud_roi->points[indx2->indices[j]].z)  )
      {
                        //de la nube de puntos (cloud_roi) se obtiene los puntos (cloud_roi->point) que se encuentran en las posiciones indicadas en (indx2->indices) que es la lista de todos los indices y en este caso "j" va recorriendo los indices que pertencen a cada marca.
                        //separatos es un vector con los indices separadores del arreglo de indises jaja en serio.
        //ORIGINAL centroid.add (pcl::PointXYZ ( cloud_roi->points[indx2->indices[j]].x, cloud_roi->points[indx2->indices[j]].y, cloud_roi->points[indx2->indices[j]].z) );
        //Transformado  
        centroid.add (pcl::PointXYZ ( cloud_roi->points[indx2->indices[j]].z, -cloud_roi->points[indx2->indices[j]].x, -cloud_roi->points[indx2->indices[j]].y) );
      }
    } 

    pcl::PointXYZ c1;
    centroid.get (c1);

    //Posicion del kinect  <origin rpy="0 0 0" xyz="0.17 0.0 0.78"/>
    // (Cambiar por la transformacion automaticamente)
    
    

    double aux;
    if( device.compare("zed_stereo") == 0)
    {
      aux = c1.x;
      c1.x = -c1.y ;
      c1.y = -c1.z;
      c1.z = aux;
    }

    c1.x += 0.17;
    c1.z += 0.78;


    std::cout << "ARUCO: " << ids2[i] << " X: " << c1.x << ", Y: " << c1.y << ", Z: " << c1.z << " DIST: " <<  sqrt( pow(c1.x,2)+pow(c1.y,2)+pow(c1.z,2) ) << " Ang: " << atan2(c1.z,c1.x)-(M_PI/2) << std::endl;
    
    

    geometry_msgs::PointStamped point_msgs;
    header.stamp = ros::Time::now();
    point_msgs.header = header;
    point_msgs.point.x =c1.x;
    point_msgs.point.y =c1.y;
    point_msgs.point.z =c1.z;

    landmarks.ids.push_back(ids2[i]);
    landmarks.pointLandmarks.push_back(point_msgs);

    //angle =  atan2(c1.x,c1.y)-M_PI/2 ;
    //distance = sqrt( pow(c1.x,2)+pow(c1.y,2);
    //geometry_msgs::PoseArray
  }

  pub_landmarks.publish(landmarks);

}


void 
indicesCallback (const kalman_loc_qr::ArucoMasksPtr& indices)
{

  indx->indices.clear();
  separators.clear();
  ids.clear();

  for(std::vector<unsigned int>::const_iterator it = indices->masks_index.begin(); it != indices->masks_index.end(); ++it)
    indx->indices.push_back(*it);
  
  for(std::vector<unsigned int>::const_iterator it = indices->separators.begin(); it != indices->separators.end(); ++it)
    separators.push_back(*it);
  
  for(std::vector<unsigned int>::const_iterator it = indices->ids.begin(); it != indices->ids.end(); ++it)
    ids.push_back(*it);

}



int
main (int argc, char** argv)
{

  ros::init (argc, argv, "landmarks_from_pc_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_cloud;

  if(ros::param::has("~device"))
  {
    
    ros::param::get("~device", device);
    
    // Create a ROS subscriber for the input point cloud
    
    if( device.compare("kinect") == 0)
    {

      sub_cloud = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
    
    }
    else if(device.compare("zed_stereo") == 0)
    {
     
      sub_cloud = nh.subscribe ("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
    
    }
    else
    {
    
      std::cout << "Sorry device " << device << "not found. Try: _device:=kinect/zed_stereo" << std::endl;
      return 0;
    
    }
  }
  else
  {
    std::cout << "Sorry U_U give a device name. Try: _device:=kinect/zed_stereo "  << device << std::endl;
    return 0;
  }

  
  ros::Subscriber sub_indices = nh.subscribe ("/indices", 1, indicesCallback);

  // Create a ROS publisher for the output point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("landmarksPointCloud", 1);
  pub_landmarks = nh.advertise<kalman_loc_qr::Landmarks> ("landmarksPoint", 1);

  // Spin xD
  ros::spin ();
}
