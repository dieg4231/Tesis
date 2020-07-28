#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <string>
#include <my_pcl_tutorial/Landmarks.h>

ros::Publisher pub;
pcl::PointIndices::Ptr indx(new pcl::PointIndices ());;
std::string device;

std::vector<unsigned int> separators;
std::vector<unsigned int> ids;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
//std::cout << "atixxxxxx \n";
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
/*  
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_pcl);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*cloud_pcl_filtered);

  cloud_plane_polygon->push_back (pcl::PointXYZ (0.0,1, 1));
  cloud_plane_polygon->push_back (pcl::PointXYZ (0.0,-1, 1));
  cloud_plane_polygon->push_back (pcl::PointXYZ (0.0,-1, -1));
  cloud_plane_polygon->push_back (pcl::PointXYZ (0.0,1, -1)); 

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::ConvexHull<pcl::PointXYZ> hull;

  // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
  hull.setInputCloud(cloud_plane_polygon);
  hull.reconstruct (*hull_points);

  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());

  

  if (hull.getDimension () == 2)
  {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (cloud_pcl_filtered);
    prism.setInputPlanarHull (hull_points);
    prism.setHeightLimits (0, 10);
    prism.segment (*indices);
  }
  else
    PCL_ERROR ("The input cloud does not represent a planar surface.\n");
 */
/*
  pcl::PointIndices::Ptr indx(new pcl::PointIndices ());
  int cta=0;
  for(int y = 374; y < 419;y++ )
    for(int x=690; x < 737; x++,cta++ )   
      indx->indices.push_back(1280*y+x);

/*
  indx->indices.clear();
  for(int i=0; i<153600;i++)
    indx->indices.push_back(i);
*/
  

  pcl::ExtractIndices<pcl::PointXYZ> filter;
  filter.setInputCloud (cloud_pcl);//_filtered);
  filter.setIndices (indx2);//ices);
  // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
  filter.setKeepOrganized (true);
  filter.filter (*cloud_roi);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  
  pcl::toROSMsg(*cloud_roi,output);
  
  std_msgs::Header header;
 // if(device.compare("kinect") == 0 )
  //{
    header.frame_id = "camera_depth_optical_frame"; //Kinect
   // std::cout << "atooo \n";
  //}else if(device.compare("zed_stereo") == 0)
  //{
   // header.frame_id = "base_link"; // ZED
   // std::cout << "ati \n";
 // }
 // else
  //{
    //std::cout << "You mustn't be here" << device << std::endl;
  //}
  
  output.header = header;
  // Publish the data
  pub.publish (output);

  

  //for(std::vector<unsigned int>::const_iterator it_ids = ids2.begin(); it_ids != ids2.end(); ++it_ids)
  for(int i = 0; i < ids2.size() ; i++)
  {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for( int j =  i == 0 ? 0 : separators2[i-1]; j < separators2[i] ; j++)//   cloud_roi->points.begin() ; it < cloud_roi->points.end(); it++)
    {
      //std::cout << "sii22" << indx2->indices[j] << std::endl;
      if( !isnan(cloud_roi->points[indx2->indices[j]].x) && !isnan(cloud_roi->points[indx2->indices[j]].y) && !isnan(cloud_roi->points[indx2->indices[j]].z)  )
      {
                        //de la numbe de puntos (cloud_roi) se obtiene los puntos (cloud_roi->point) que se encuentran en las posiciones indicadas en (indx2->indices) que es la lista de todos los indices y en este caso "j" va recorriendo los indices que pertencen a cada marca.
                        //separatos es un vector con los indices separadores del arreglo de indises jaja en serio.
        centroid.add (pcl::PointXYZ ( cloud_roi->points[indx2->indices[j]].x, cloud_roi->points[indx2->indices[j]].y, cloud_roi->points[indx2->indices[j]].z) );
        //std::cout << "sii" << cloud_roi->points[indx2->indices[j]].x<< std::endl;
      }
      //std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
    } 

    pcl::PointXYZ c1;
    centroid.get (c1);

    std::cout << "ARUCO: " << ids2[i] << " X: " << c1.x << ", Y: " << c1.y << ", Z: " << c1.z << " DIST: " <<  sqrt( pow(c1.x,2)+pow(c1.y,2)+pow(c1.z,2) ) << " Ang: " << atan2(c1.z,c1.x)-(M_PI/2) << std::endl;
  
  }

}


void 
indicesCallback (const my_pcl_tutorial::LandmarksPtr& indices)
{
  //std::cout << "atiwwwwwwwww \n";
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
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  if(ros::param::has("~device"))
  {
    ros::param::get("~device", device);
    std::cout << "Device: " << device << std::endl;
  }
  else
  {
    std::cout << "Sorry device not found. Try: kinect/zed_stereo" << device << std::endl;
    return 0;
  }


  // Create a ROS subscriber for the input point cloud
  //if(device.compare("kinect") == 0)
    
    ros::Subscriber sub_cloud = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
    //ros::Subscriber sub_cloud = nh.subscribe ("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  /*else if(device.compare("zed_stereo") == 0)
  {
    std::cout << "aquuuoio \n";
    ros::Subscriber sub_cloud = nh.subscribe ("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  }
  else
  {
    std::cout << "Sorry U_U" << device << std::endl;
    return 0;
  }*/
  
  ros::Subscriber sub_indices = nh.subscribe ("/indices", 1, indicesCallback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
