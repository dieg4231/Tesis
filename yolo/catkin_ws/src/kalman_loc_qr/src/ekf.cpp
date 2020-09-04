#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//kalman ?
#include <Eigen/Dense>


nav_msgs::Odometry robot_odom;
nav_msgs::Odometry robot_odom_1;

double alfa1 = 0.00001;
double alfa2 = 0.00001;
double alfa3 = 0.00001;
double alfa4 = 0.00001;



/*
Kalaman filter as "Probabilistic robotics page: 204"
*/

/*
#include "ros/ros.h"
#include <ros/package.h>
#include <random>
#include <iostream>
#include <Eigen/Dense>
#include "simulator/Parameters.h"
#include "simulator/simulator_parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_base.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovariance.h"
using namespace Eigen;




geometry_msgs::PoseWithCovariance  prediccion;
geometry_msgs::PoseWithCovariance  actualizacion;


parameters params;

*/
Eigen::Matrix3d q;            // Error del proceso 
Eigen::Matrix3d r;            // Error de medicion  
Eigen::Matrix3d p;            // Matriz de covarianza
Eigen::Matrix3d H;            // Jacobiano de la funcion H
Eigen::Vector3d x_;           // Vector de estado
Eigen::Matrix3d s;            // s xD
Eigen::Matrix3d k;            // ganancia de Kalman
Eigen::Matrix3d I;           // Identity Matrix

/*

typedef struct LandMark_ {
        float x; //Coordenada del robot x
        float y; //Coordenada del robot y
        float r; //Distancia del landmark al robot
        float t; //Angulo del landmark respecto al robot
} LandMark;



std::vector<LandMark> objs;

std::vector<Vector2d> position_ideal;
std::vector<Vector2d> position_real;
std::vector<Vector2d> position_kalman;

int swi;
double theta_track;
bool debug=0;

*/
double stddev_distance = 0.03;
double stddev_theta = 0.174533;


double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
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


double sample_motion_model_odometry(Eigen::Vector3d &x_vector)
{
	double d_rot1, d_rot1_hat;
	double d_rot2, d_rot2_hat;
	double d_trans1, d_trans1_hat;
	
	double x_bar_p = robot_odom.pose.pose.position.x;
	double y_bar_p = robot_odom.pose.pose.position.y;
	double theta_bar_p;

	double x_bar = robot_odom_1.pose.pose.position.x;
	double y_bar = robot_odom_1.pose.pose.position.y;
	double theta_bar;
	double roll, pitch;

	theta_bar =  getYawFromQuaternion(robot_odom_1.pose.pose.orientation);
	theta_bar_p =  getYawFromQuaternion(robot_odom.pose.pose.orientation);

	std::cout << "theta_bar_p: " << theta_bar_p << "\n";
	std::cout << "theta_bar: " <<  theta_bar  << "\n";

	if( sqrt(  pow(x_bar - x_bar_p, 2)  +  pow(y_bar - y_bar_p, 2) < .01) ) // 
		d_rot1 = 0; // Si solo gira  y este valor no es cero entonces  d_rot2 = - d_rot1 y el angulo final es practicamente el mismo  que el inicial :o alv
	else
		d_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar ) - theta_bar;


	d_trans1 = sqrt(  pow(x_bar - x_bar_p, 2)  +  pow(y_bar - y_bar_p, 2)  );
	d_rot2 = theta_bar_p - theta_bar - d_rot1;

	d_rot1_hat =  d_rot1 ;//- pf_ran_gaussian( alfa1 * pow(d_rot1,2) + alfa2 * pow(d_trans1,2) );
	d_trans1_hat = d_trans1 ;//- pf_ran_gaussian( alfa3 * pow(d_trans1,2) + alfa4 * pow(d_rot1,2) + alfa4 * pow(d_rot2,2));
	d_rot2_hat = d_rot2 ;//- pf_ran_gaussian(alfa1 * pow(d_rot2,2) + alfa2 * pow(d_trans1,2));

	x_vector(0) = x_vector(0) + d_trans1_hat * cos( x_vector(2) + d_rot1_hat );
	x_vector(1) = x_vector(1) + d_trans1_hat * sin( x_vector(2) + d_rot1_hat );
	

	std::cout << "Angulo original: " << x_vector(2)<< "\n";
	std::cout << "Angulo rot1: " <<  d_rot1_hat   << "\n";
	std::cout << "Angulo rot2: " <<  d_rot2_hat   << "\n";

	x_vector(2) = x_vector(2) + d_rot1_hat + d_rot2_hat;

	std::cout << "Angulo final: " <<  x_vector(2)  << "\n";

	return  d_trans1_hat;

}

int debug = 1, ns =1;

bool ekf ()
{
	float theta;
	float dist;
	float angle_z_i_t_hat;
	double translation;

	Eigen::Vector3d v; // Diference of measurment and predicted measurment
	Eigen::Matrix3d vf;
	Eigen::Vector3d z_i_t;
	Eigen::Vector3d z_i_t_hat;

	
	if(ns) // Restart variables for new estimation 
	{   ns = 0;
		std::cout << "New simulator: " << "\n";
		r << 0.00001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001;
	    //q << 0.00001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001;
	    q << pow(stddev_distance,2), 0, 0,0, pow(stddev_distance,2), 0, 0, 0, pow(stddev_theta,2);
		p << 0,0,0,0,0,0,0,0,0;
		x_ << robot_odom_1.pose.pose.position.x ,robot_odom_1.pose.pose.position.y, getYawFromQuaternion(robot_odom_1.pose.pose.orientation);
		if(x_(2) > M_PI) x_(2) -= 2 * M_PI;
		if(x_(2) < M_PI) x_(2) += 2 * M_PI;
		
		

	}

	 // Prediction step
		
		
	
		if(debug)std::cout << "Inicio: \n" << x_ << "\n ------\n";
	    

		translation = sample_motion_model_odometry(x_);
		x_(2) = atan2(sin(x_(2)),cos(x_(2)));
		/*
		theta = x_(2) + req.theta;
		x_(0) = x_(0) + req.distance * cos(theta) ;
		x_(1) = x_(1) + req.distance * sin(theta) ;
		x_(2) = theta;
	    */

	    vf <<  	1, 0, -translation * sin(x_(2)), 0, 1,  translation * cos(x_(2)), 0, 0,  1;
		p = vf * p * vf.transpose() + q;
		if(debug)std::cout << "Fin prediccion x: \n" << x_ << "\n ------\n";
		if(debug)std::cout << "Fin prediccion p: \n" << p << "\n ------\n";

		/*
		theta_track += req.theta;
		position_ideal.push_back(Vector2d( position_ideal.back()(0) + req.distance * cos(theta_track) ,position_ideal.back()(1) + req.distance * sin(theta_track) ) );
		

		prediccion.pose.position.x = x_(0);
		prediccion.pose.position.y = x_(1);
		prediccion.pose.position.z = 0;

		prediccion.pose.orientation.x = 0;
		prediccion.pose.orientation.y = 0;
		prediccion.pose.orientation.z = x_(2);

		prediccion.covariance[0] = p(0,0);
		prediccion.covariance[1] = p(0,1);
		prediccion.covariance[5] = p(0,2);
		prediccion.covariance[6] = p(1,0);
		prediccion.covariance[7] = p(1,1);
		prediccion.covariance[11] = p(1,2);
		prediccion.covariance[30] = p(2,0);
		prediccion.covariance[31] = p(2,1);
		prediccion.covariance[35] = p(2,2);

		return true;
	}	
	else // Actualizacion
	{
		swi = 0;
	
		for(std::vector<LandMark>::const_iterator mj = objs.begin(); mj != objs.end(); ++mj)
		{	
			if(debug)std::cout << "Centroide x y r t : \n" << mj->x << " , " << mj->y << " , " << mj->r << " , " << mj->t   << "\n ------\n";
			
			z_i_t << mj->r , mj->t ,0; // The real measurement
			
			if(debug)std::cout << "z_i_t: \n" << z_i_t << "\n ------\n";
			
			// Calculo de las lecturas esperadas respecto a la posicion del robot obtenida en el paso de prediccion
				
				dist = pow( mj->x - x_(0),2 ) + pow( mj->y - x_(1) ,2); // Radicando de la de la distancia entre un landmark y el robot 

				// Angulo del landmark respecto al robot
				angle_z_i_t_hat  = atan2(mj->y - x_(1), mj->x - x_(0)) - x_(2);
			
				// Validacion para que el angulo obtenido siempre este entre pi y -pi
				if (angle_z_i_t_hat > M_PI)
					angle_z_i_t_hat -= 2*M_PI;
				if (angle_z_i_t_hat < -M_PI)
					angle_z_i_t_hat += 2*M_PI;

				// Vector de lecturas esperadas
				z_i_t_hat  << sqrt(dist), angle_z_i_t_hat, 0 ;

				if(debug)std::cout << "z_i_t_hat: \n" << z_i_t_hat << "\n ------\n";
			
			// fin calculo de lecturas esperadas
			
			// El jacobiano de la funcion h() con respecto al robot	
			H << -( mj->x - x_(0) ) / sqrt(dist) , -( mj->y - x_(1) ) / sqrt(dist),0,
			 	  ( mj->y - x_(1) ) / dist, -( mj->x - x_(0) ) / dist,-1,
			 	  0,0,0; // this row is zero por que  la etiqueta del landmark no depende de la posicion del robot
			
			if(debug)std::cout << "H: \n" << H << "\n ------\n";
			
			s = ( H * p * H.transpose() ) + r; // S xD
			
			if(debug)std::cout << "S: \n" << s << "\n ------\n";

			k = p * H.transpose() * s.inverse(); // Ganancia de kalman
			
			if(debug)std::cout << "K: \n"<<  k << "\n ------\n";

			v = z_i_t - z_i_t_hat;  // original menos lo esperadp_
			
			if(debug)std::cout << "V: \n"<< v << "\n ------\n";
			
			x_ = x_ + (k*v); // Actualizacion vector de estado
			
			if(debug)std::cout << "X: \n" << x_ << "\n ------\n";
			
			p = ( I - k * H ) * p; // Actualizacion matriz de covarianza
			
			if(debug)std::cout << "P: \n" << p << "\n ------\n";

		}
		
		// Validacion para que el angulo obtenido siempre este entre pi y -pi
		if (x_(2) > M_PI)
			x_(2) -= 2*M_PI;
		if (x_(2) < -M_PI)
			x_(2) += 2*M_PI;

		// Arrow for visualization in rviz
		position_real.push_back(Vector2d(params.robot_x,params.robot_y));
		position_kalman.push_back(Vector2d(x_(0),x_(1)));
		
		arrow.pose.position.x = x_(0);
		arrow.pose.position.y = x_(1);
		arrow.pose.position.z = 0;
		arrow.pose.orientation = tf::createQuaternionMsgFromYaw(x_(2));


		actualizacion.pose.position.x = x_(0);
		actualizacion.pose.position.y = x_(1);
		actualizacion.pose.position.z = 0;

		actualizacion.pose.orientation.x = 0;
		actualizacion.pose.orientation.y = 0;
		actualizacion.pose.orientation.z = x_(2);

		actualizacion.covariance[0] = p(0,0);
		actualizacion.covariance[1] = p(0,1);
		actualizacion.covariance[5] = p(0,2);
		actualizacion.covariance[6] = p(1,0);
		actualizacion.covariance[7] = p(1,1);
		actualizacion.covariance[11] = p(1,2);
		actualizacion.covariance[30] = p(2,0);
		actualizacion.covariance[31] = p(2,1);
		actualizacion.covariance[35] = p(2,2);


		if(debug)std::cout << "Prediccion Final: \n" << x_ << "\n ------\n";

		return true;
	}

}


*/
}

void landmarksPointCallBack (const kalman_loc_qr::LandmarksConstPtr& msg)
{ 
  
 
    
}




void odomCallback(const nav_msgs::Odometry& msg)
{
	robot_odom = msg ;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Nodo_ekf");
	ros::NodeHandle nh;
  	ros::Subscriber landmarks_sub = nh.subscribe ("landmarksPoint", 0, landmarksPointCallBack);
  	ros::Subscriber odom_sub = nh.subscribe("odom", 0, odomCallback);
	
	ros::Rate rate(10);
	while(odom_sub.getNumPublishers() == 0)
	{
		rate.sleep();
	};
	ros::spinOnce();

	robot_odom_1 = robot_odom;
	
	//double roll_1, pitch_1, yaw_1;
	//tf::Matrix3x3 mtx_rot_1(robot_odom_1.pose.orientation);
	//mtx_rot_1.getRPY(roll_1, pitch_1, yaw)_1;

	double roll,pitch,yaw;

	//tf::Matrix3x3 mtx_rot;
	
	tf2::Quaternion quat_tf_1, quat_tf,quat_r;
	
	double min_dist_update = .02;

	double min_angle_update = 0.03490658503;// 2grados
	while(ros::ok())
	{
		//std::cout << "odom_1 x" << robot_odom_1.pose.pose.orientation.x  << " \n";
		//std::cout << "odom_1 y" << robot_odom_1.pose.pose.orientation.y  << " \n";
		//std::cout << "odom_1 z" << robot_odom_1.pose.pose.orientation.z  << " \n";
		//std::cout << "odom_1 w" << robot_odom_1.pose.pose.orientation.w  << " \n";	


		//std::cout << "q-1" << quat_tf_1.x() <<" : "<< quat_tf_1.y() <<" : " << quat_tf_1.z() <<" : " << quat_tf_1.w() <<" \n";
		//std::cout << "q" << quat_tf.x() <<" : "<< quat_tf.y() <<" : " << quat_tf.z() <<" : " << quat_tf.w() <<" \n"; 
			//ROS_INFO("SUBS: %d",odom_sub.getNumPublishers()); 	

		tf2::fromMsg(robot_odom_1.pose.pose.orientation, quat_tf_1);
		tf2::fromMsg(robot_odom.pose.pose.orientation, quat_tf);


		quat_tf_1[3] = -quat_tf_1[3];
		quat_r=quat_tf*quat_tf_1 ;
		quat_r.normalize();
		tf2::Matrix3x3 mtx_rot( quat_r );
		mtx_rot.getRPY(roll, pitch, yaw);

		



    	if( 
			( sqrt(pow(robot_odom_1.pose.pose.position.x - robot_odom.pose.pose.position.x,2) + pow(robot_odom_1.pose.pose.position.y - robot_odom.pose.pose.position.y,2)) > min_dist_update )
			 
			)
		{
			
			ROS_INFO("Actualizacion position");
			ekf();
			robot_odom_1 = robot_odom;
			
			
		}
		else if (( fabs(yaw) > min_angle_update ))
		{
			ROS_INFO("Actualizacion orientation");
			
			ekf();
			robot_odom_1 = robot_odom;
		}
		
		//std::cout << yaw << std::endl;
		ros::spinOnce();
		rate.sleep();
	}

}


