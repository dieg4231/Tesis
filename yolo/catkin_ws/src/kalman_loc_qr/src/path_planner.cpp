#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include "quadtree.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovariance.h>


ros::Publisher map_pub;
ros::Publisher path_pub;
std_msgs::Header header ;
nav_msgs::MapMetaData info ;
geometry_msgs::TransformStamped TRANSFORM_STAMPED_MAP_2_BASE ;

typedef struct _conection conection;
typedef struct _nodo nodo;


typedef struct _conection
{
   nodo *node;
   double cost;
}  conection;

typedef struct _nodo
{
   char flag;
   int num_node;
   double x;
   double y;
   //int num_conections;
   std::vector<conection> conections;
   int parent;
   double acumulado;
}  nodo;

std::vector<nodo> nodes;



void dijkstra_algorithm(int D ,int L)
{
   /*
      D = Nodo Inicial
      L = Nodo Final
      Y = Totalmente expandido
      N = Nuevo (Nodo sin padre ni acumulado)
      P = Nodo que no es ni Y ni N   (Parcialmente expandido)

      Video explicativo https://www.youtube.com/watch?v=LLx0QVMZVkk
   */

    /*Clean variables*/
    for (auto &ptr_nodo : nodes) // c++ 11
    {
      ptr_nodo.flag = 'N';
      ptr_nodo.acumulado = 0;
      ptr_nodo.parent = -1;
    }

   int menor,flagOnce;
   int contador = 0;
   int j;

   nodes[D].acumulado = 0;

   while( nodes[L].flag != 'Y')
   {  
     std::cout << "Empieza \n";
      for(j = 0 ; j < nodes[D].conections.size(); j++)
         {
            if( nodes[D].conections[j].node->flag == 'N')
            {
              nodes[D].conections[j].node->acumulado = nodes[D].acumulado + nodes[D].conections[j].cost;
              nodes[D].conections[j].node->parent = D;
              nodes[D].conections[j].node->flag = 'P';
              std::cout << "n \n";
            }  
            else if( nodes[D].conections[j].node->flag == 'P' )
            {
              std::cout << "p \n";
               if( nodes[D].conections[j].node->acumulado > nodes[D].acumulado + nodes[D].conections[j].cost)
               {
                  nodes[D].conections[j].node->acumulado = nodes[D].acumulado + nodes[D].conections[j].cost;
                  nodes[D].conections[j].node->parent = D;
               }
            }
            else
            {
              std::cout << "??? \n";
            }
         }

      nodes[D].flag = 'Y';
         menor = 0;
         flagOnce = 1;
         for(int j = 0; j < nodes.size() ; j++)
         {
            if(nodes[j].flag == 'P')
            {
               if(flagOnce)
               {
                  menor = j;
                  flagOnce = 0;
               }
               else if( nodes[menor].acumulado > nodes[j].acumulado )
               {
                  menor = j;
               }
               std::cout << "q " << nodes.size()  << "  : " << j<< "meno :" << menor <<"\n";
            }  
            //std::cout << "q " << nodes.size()  << "  : " << j<< "\n";
         }
         //std::cout << "-----------------------------menor: "<< menor  <<" \n";
         D = menor;
         std::cout << "D: "<< D  <<" \n";
   }
}





void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  int step =1;
  header = msg->header;
  info = msg->info;
  ROS_INFO("Got map %d %d", info.width, info.height);

  //matrix del tamaño del mapa y cada elemento es un char(byte con signo)
  cv::Mat binary_map(info.width, info.height, CV_8S);


  //Asigna los valores del mapa a la matriz de opencv
  for (unsigned int x = 0; x < info.width; x+=step)
    for (unsigned int y = 0; y < info.height; y+=step)
      binary_map.at<char>(y,x) = msg->data[x+ info.width * y];
  
  //Se hace un clone de la matriz
  cv::Mat clone = binary_map.clone();

  //Por cada celda ocupada se dibuja un circulo de radio igual al radio del robot 
  for (unsigned int x = 0; x < info.width; x+=step)
    for (unsigned int y = 0; y < info.height; y+=step)
      if(clone.at<char>(y,x) == 100)
        circle(binary_map, cv::Point(x,y),5, 100,CV_FILLED, -1);
  
  //Se le vuelve a aplicar la mascara de las celdas desconocidar para que los bordes del mapa no queden gorditos
  for (unsigned int x = 0; x < info.width; x+=step)
    for (unsigned int y = 0; y < info.height; y+=step)
      if(clone.at<char>(y,x) == -1)
        binary_map.at<char>(y,x) = -1;
      

  //Los valores de la matrix de opnecv son asignados a un tipo MAPA
  Map map(info.width, info.height);
  for (unsigned int x = 0; x < info.width; x+=step)
    for (unsigned int y = 0; y < info.height; y+=step)
      map.Insert(Cell(x,y,info.width,binary_map.at<uchar>(y,x)));
  
  
  //Se publica el mapa
  nav_msgs::OccupancyGrid* newGrid = map.Grid();
  newGrid->header = header;
  newGrid->info = info;
  map_pub.publish(*newGrid);

      //Show the results
    
    //cv::namedWindow("Output", cv::WINDOW_NORMAL);
    //cv::imshow("Output", binary_map);
    
    //cv::imwrite("/home/diego/Desktop/mapa.png", binary_map);
    //cv::waitKey(0);

    double distancia_diagonal =   sqrt( pow(info.resolution,2) + pow(info.resolution,2)  ); 
double distancia_recto = info.resolution;

  
  nodo nodo_aux;

  for (unsigned int x = 0; x < info.width; x+=step)
    for (unsigned int y = 0; y < info.height; y+=step)
    {
      if(binary_map.at<char>(y,x) == 0)
      {
        nodo_aux.num_node = x+ info.width * y;
        nodo_aux.x = x;
        nodo_aux.y = y;
        nodo_aux.flag = 'N';
        nodo_aux.acumulado = 0;
        nodo_aux.parent = -1;
        nodes.push_back(nodo_aux);
      }
    }

  
  conection connection_aux;

 
    //for (vector<nodo>::iterator ptr = nodes.begin(); ptr < nodes.end(); ptr++) other c++ version 
    for (auto &ptr_nodo : nodes) // c++ 11
    {
      
      int x = ptr_nodo.x;
      int y = ptr_nodo.y;

      for(int i = x - 1; i < x + 2; i++)
        for(int j = y - 1; j < y + 2; j++)
        {
          if( i == x && j == y)
            continue;
          if( binary_map.at<char>(j,i) == 0)
          {
            double id = i + info.width * j;
            auto pred = [id](const nodo & item) {return item.num_node == id;};

            connection_aux.node = nodes.empty() ? NULL : &nodes[0] + (std::find_if(std::begin(nodes), std::end(nodes), pred) - nodes.begin());
            connection_aux.cost = ( i == x - 1 && j == y - 1 ) || ( i == x + 1 && j == y - 1 ) || ( i == x + 1 && j == y + 1 ) || ( i == x - 1 && j == y + 1 ) ? distancia_diagonal : distancia_recto;
            ptr_nodo.conections.push_back(connection_aux);
          }
        }
    }


  /*
    int g=0;
    for (auto &ptr_nodo : nodes) // c++ 11
    {
      if(g++>30)
        break;

      std::cout << "ptr_nodo.num_node" << ptr_nodo.num_node << std::endl;

      std::cout << "ptr_nodo.x" << ptr_nodo.x << std::endl;
      std::cout << "ptr_nodo.y" << ptr_nodo.y << std::endl;
      std::cout << "Num of connections " << ptr_nodo.conections.size() << std::endl;

     for (auto &ptr_conection : ptr_nodo.conections ) 
        {
          std::cout << "  ptr_conection.num_node" << ptr_conection.node->num_node << std::endl;
          std::cout << "  ptr_conection.x" << ptr_conection.node->x << std::endl;
          std::cout << "  ptr_conection.y" << ptr_conection.node->y << std::endl;
        }
        
    }
  */
  

}


 int find_nearest_to(double x,double y)
 {
   double x_nodo;
   double y_nodo;

    double distancia;
    double min_distancia = 1000;
    int index;

  for (int i = 0; i < nodes.size(); i++ ) // c++ 11
  {
      x_nodo = (nodes[i].x  * info.resolution) +  (   info.origin.position.x );
      y_nodo = (nodes[i].y  * info.resolution) +  (   info.origin.position.y );
      
      distancia = sqrt(  pow(x - x_nodo,2) + pow(y - y_nodo,2)   );
      if(distancia < min_distancia)
      { 
        min_distancia = distancia;
        index = i;
      }
  }
  return index;
 }


int goal;

void pathCalculator(int start, int goal1)
{
  
  dijkstra_algorithm(goal1 ,start);


  nav_msgs::Path path;

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  geometry_msgs::PoseStamped pose;
  pose.pose.position.z = 0;
  pose.pose.orientation.w = 0;
  


  int padre = start;
  while( padre != -1)
  { 	 
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

   	pose.pose.position.x = (nodes[padre].x  * info.resolution) +  (   info.origin.position.x ) ;
   	pose.pose.position.y = (nodes[padre].y  * info.resolution) +  (   info.origin.position.y ) ;


   	padre = nodes[padre].parent;
    path.poses.push_back(pose);
  }
	
  path_pub.publish(path);
  
  std::cout << "Goal recived"  << std::endl;
}

void goalCallback(const geometry_msgs::PoseStamped::Ptr& msgs)
{
  
  //Find nearest node :D

  goal = find_nearest_to(msgs->pose.position.x,msgs->pose.position.y);

  int start = find_nearest_to(TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x,TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y);
  pathCalculator(start, goal);

  std::cout << "Goal recived from  2D nav goal"  << std::endl;
}


void LocalizationEkfCallback(const geometry_msgs::PoseWithCovariance& msg)
{

  //Find nearest node :D
  //int start = find_nearest_to(TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.x,TRANSFORM_STAMPED_MAP_2_BASE.transform.translation.y);
  //pathCalculator(start, goal);

  //std::cout << "Goal recived from new localization"  << std::endl;
}



int main(int argc, char **argv){



  ros::init(argc, argv, "path_planner");
  ros::NodeHandle n;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
  path_pub = n.advertise<nav_msgs::Path>("path",10);
  ros::Subscriber map_sub = n.subscribe("map",10,mapCallback);
  ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal",10,goalCallback);
  ros::Subscriber localization_ekf_sub = n.subscribe("localization_ekf", 0, LocalizationEkfCallback);
  /*
		Get map to base_link trasformation
	*/

	tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
	
  ros::Rate rate(10);
	while (n.ok())
	{	/*
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
		}    */
    ros::spinOnce();
		rate.sleep();
	}
  
  ros::spin();
  return 0;
}