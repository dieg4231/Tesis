#include <ros/ros.h>
#include <XmlRpcException.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

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


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
//kalman ?
#include <Eigen/Dense>

geometry_msgs::TransformStamped TRANSFORM_STAMPED_BASE_2_DEVICE ;
int debug = 1, ns =1;
bool UPDATE_LANDMARKS_FLAG = true;
struct Landmark {
   geometry_msgs::Point  point;
   int  id;
} ;  

std::vector<Landmark> landmarks_detected;
std::vector<Landmark> landmarks_on_map;

nav_msgs::Odometry robot_odom;
nav_msgs::Odometry robot_odom_1;
double initial_x = 0;
double initial_y = 0;
double initial_a = 0;
double ALPHA[4];

/*
Kalaman filter as "Probabilistic robotics page: 204"
*/

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovariance  localizatio_pose;

Eigen::Matrix3d q;            // Error del proceso 
Eigen::Matrix3d r;            // Error de medicion  
Eigen::Matrix3d p;            // Matriz de covarianza
Eigen::Matrix3d H;            // Jacobiano de la funcion H
Eigen::Vector3d x_;           // Vector de estado
Eigen::Matrix3d s;            // s xD
Eigen::Matrix3d k;            // ganancia de Kalman
Eigen::Matrix3d I;           // Identity Matrix


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
	return yaw;
}


void sample_motion_model_odometry(Eigen::Vector3d &x_vector, double *theta_plus_rotation1, double *translation )
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
		d_rot1 = normalize(normalize(atan2(y_bar_p - y_bar, x_bar_p - x_bar )) - normalize(theta_bar));//atan2(y_bar_p - y_bar, x_bar_p - x_bar ) - theta_bar;


	d_trans1 = sqrt(  pow(x_bar - x_bar_p, 2)  +  pow(y_bar - y_bar_p, 2)  );
	d_rot2 = normalize(normalize(theta_bar_p) - normalize(theta_bar + d_rot1)); //theta_bar_p - theta_bar - d_rot1;


	d_rot1_hat =  d_rot1 - pf_ran_gaussian( ALPHA[0] * pow(d_rot1,2) + ALPHA[1] * pow(d_trans1,2) );
	d_trans1_hat = d_trans1 - pf_ran_gaussian( ALPHA[2] * pow(d_trans1,2) + ALPHA[3] * pow(d_rot1,2) + ALPHA[3] * pow(d_rot2,2));
	d_rot2_hat = d_rot2 - pf_ran_gaussian(ALPHA[0] * pow(d_rot2,2) + ALPHA[1] * pow(d_trans1,2));


	x_vector(0) = x_vector(0) + d_trans1_hat * cos( x_vector(2) + d_rot1_hat );
	x_vector(1) = x_vector(1) + d_trans1_hat * sin( x_vector(2) + d_rot1_hat );
	
	*theta_plus_rotation1 = x_vector(2) + d_rot1_hat;
	*translation = d_trans1_hat;

	std::cout << "Angulo original: " << x_vector(2) << "\n";
	std::cout << "Angulo rot1: " <<  d_rot1_hat   << "\n";
	std::cout << "Traslacion: " <<  d_trans1_hat   << "\n";
	std::cout << "Angulo rot2: " <<  d_rot2_hat   << "\n";

	x_vector(2) = x_vector(2) + d_rot1_hat + d_rot2_hat;

	x_vector(2) = normalize(x_(2));
	std::cout << "Angulo final: " <<  x_vector(2)  << "\n";

}




bool ekf ()
{
	float theta;
	float dist;
	float angle_z_i_t_hat;
	double translation, theta_plus_rotation1;

	Eigen::Vector3d v; // Diference of measurment and predicted measurment
	Eigen::Matrix3d vf;
	Eigen::Vector3d z_i_t;
	Eigen::Vector3d z_i_t_hat;

	
	if(ns) // Restart variables for new estimation 
	{   ns = 0;
		std::cout << "New simulator: " << "\n";
		///r << .09, 0, 0, 0, .90, 0, 0, 0,.16;
	    //q << 0.00001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001;
	    //q << 0.01,0,0,0,0.01,0,0,0,0.01; //, 0, 0,0, pow(stddev_distance,2), 0, 0, 0, pow(stddev_theta,2);
		p << 0,0,0,0,0,0,0,0,0;
		std::cout << "El odom antes: " <<  robot_odom_1.pose.pose.orientation  << "\n";
		x_ << initial_x ,initial_y, initial_a;
		//if(x_(2) > M_PI) x_(2) -= 2 * M_PI;
		//if(x_(2) < M_PI) x_(2) += 2 * M_PI;
		x_(2) = normalize(x_(2));

	}

	 // Prediction step
	
		if(debug)std::cout << "Inicio prediccion::::::::::::::::::::::::::::::::::::::::::::: \n" << x_ << "\n ------\n";
	    

		sample_motion_model_odometry(x_, &translation, &theta_plus_rotation1 );
		

		if(debug)std::cout << "X: \n" << x_ << "\n ------\n";
		/*
		theta = x_(2) + req.theta;
		x_(0) = x_(0) + req.distance * cos(theta) ;
		x_(1) = x_(1) + req.distance * sin(theta) ;
		x_(2) = theta;
	    */

	    vf <<  	1, 0, -translation * sin(theta_plus_rotation1), 0, 1,  translation * cos(theta_plus_rotation1), 0, 0,  1;



		if(debug)std::cout << "vf: \n" << vf << "\n ------" << std::endl;		
		p = vf * p * vf.transpose() + q;
		if(debug)std::cout << "q: \n" << q << "\n ------\n";
		if(debug)std::cout << "Fin prediccion x: \n" << x_ << "\n ------\n";
		if(debug)std::cout << "Fin prediccion p: \n" << p << "\n ------\n";

		Eigen::Vector3d ajuste;
		ajuste << 0,0,0;

		Eigen::Vector3d media;
		media << 0,0,0;

		int n_land=0;

		UPDATE_LANDMARKS_FLAG = false;
		int id_index;
		
		for( int i = 0; i < landmarks_detected.size(); ++i)
		{	//p = p/2.0;
			
			for(id_index = 0; id_index < landmarks_on_map.size(); id_index++)
				if( landmarks_detected[i].id == landmarks_on_map[id_index].id )
					break;
			
			if(debug)std::cout << " landmarks_detected[i].point.x " << landmarks_detected[i].point.x << std::endl;
			if(debug)std::cout << " landmarks_detected[i].point.y " << landmarks_detected[i].point.y << std::endl;
			if(debug)std::cout << "	ajuste : " << ajuste << std::endl;

/*
			landmarks_detected[i].point.x += ajuste(0);
			landmarks_detected[i].point.y += ajuste(1);
				
			double aux_angle_x =  landmarks_detected[i].point.x*cos(ajuste(2)) + landmarks_detected[i].point.y*sin(ajuste(2));
			double aux_angle_y = -landmarks_detected[i].point.x*sin(ajuste(2)) + landmarks_detected[i].point.y*cos(ajuste(2));
			landmarks_detected[i].point.x =  aux_angle_x;
			landmarks_detected[i].point.y = aux_angle_y;
			*/
			

			if(debug)std::cout << "		Landmarks_detected[i].id : " << landmarks_detected[i].id << std::endl;
			if(debug)std::cout << "		Landmarks_on_map[id_index].id : " << landmarks_on_map[id_index].id << std::endl;
			if(debug)std::cout << "		Landmarks_on_map[id_index].point.x : " << landmarks_on_map[id_index].point.x << std::endl;
			if(debug)std::cout << "		Landmarks_on_map[id_index].point.y : " << landmarks_on_map[id_index].point.y << std::endl;
			if(debug)std::cout << "		Distancia x esperada : " << landmarks_on_map[id_index].point.x - x_(0) << std::endl;
			if(debug)std::cout << "		Distancia y esperada : " << landmarks_on_map[id_index].point.y - x_(1) << std::endl;
			if(debug)std::cout << "		Landmarks_detected[i].point.x : " << landmarks_detected[i].point.x << std::endl;
			if(debug)std::cout << "		Landmarks_detected[i].point.y : " << landmarks_detected[i].point.y << std::endl;
			if(debug)std::cout << "		Error en x : " << landmarks_detected[i].point.x - fabs(landmarks_on_map[id_index].point.x - x_(0) )<< std::endl;
			if(debug)std::cout << "		Error en y : " << landmarks_detected[i].point.y - fabs(landmarks_on_map[id_index].point.y - x_(1) )<< std::endl;
			
			//if( fabs(fabs(landmarks_detected[i].point.x) - fabs(landmarks_on_map[id_index].point.x - x_(0) ) )  > 0.6 ||
			//fabs(fabs(landmarks_detected[i].point.y) - fabs(landmarks_on_map[id_index].point.y - x_(1) )) > .6)
			//	break;


			// The real measurement
			z_i_t << sqrt( pow(landmarks_detected[i].point.x,2) + pow(landmarks_detected[i].point.y,2) ) , atan2(landmarks_detected[i].point.y,landmarks_detected[i].point.x) ,0; 
			
			if(debug)std::cout << "		z_i_t (Mesurment): \n" << z_i_t << "\n ------" << std::endl;
			
			// Calculo de las lecturas esperadas respecto a la posicion del robot obtenida en el paso de prediccion
				
			dist = pow( landmarks_on_map[id_index].point.x - x_(0),2 ) + pow( landmarks_on_map[id_index].point.y - x_(1) ,2); // Radicando de la de la distancia entre un landmark y el robot 

			// Angulo del landmark respecto al robot
			angle_z_i_t_hat  = atan2(landmarks_on_map[id_index].point.y - x_(1), landmarks_on_map[id_index].point.x - x_(0)) - x_(2);
			
			// Validacion para que el angulo obtenido siempre este entre pi y -pi
			/*if (angle_z_i_t_hat > M_PI)
				angle_z_i_t_hat -= 2*M_PI;
			if (angle_z_i_t_hat < -M_PI)
				angle_z_i_t_hat += 2*M_PI;
*/
			angle_z_i_t_hat = normalize(angle_z_i_t_hat);			
			// Vector de lecturas esperadas
			z_i_t_hat  << sqrt(dist), angle_z_i_t_hat, 0 ;

			if(debug)std::cout << "		z_i_t_hat (Esperadas): \n" << z_i_t_hat << "\n ------" << std::endl;

			if(debug)std::cout << "		Error z_i_t_hat (Esperadas) - z_i_t(measurmet): \n" << z_i_t  - z_i_t_hat  << "\n ------" << std::endl;
			
			if( (fabs(z_i_t(0)  - z_i_t_hat(0)) > 1.0) || (fabs(z_i_t(1)  - z_i_t_hat(1)) > 1.0) )
			{
				if(debug)std::cout << "		SALTOOOOOO ....................." << std::endl;
				break;
			}
				

			// fin calculo de lecturas esperadas
			
			// El jacobiano de la funcion h() con respecto al robot	
			H << -( landmarks_on_map[id_index].point.x - x_(0) ) / sqrt(dist) , -( landmarks_on_map[id_index].point.y - x_(1) ) / sqrt(dist),0,
			 	  ( landmarks_on_map[id_index].point.y - x_(1) ) / dist, -( landmarks_on_map[id_index].point.x - x_(0) ) / dist,-1,
			 	  0,0,0; // this row is zero por que  la etiqueta del landmark no depende de la posicion del robot
			
			if(debug)std::cout << "		H: \n" << H << "\n ------\n";
			if(debug)std::cout << "		r: \n" << r << "\n ------\n";
			



			s = ( H * p * H.transpose() ) +  ((landmarks_detected.size()==1)? r/100 :r); // S xD
			
			if(debug)std::cout << "		S: \n" << s << "\n ------\n";

			k = p * H.transpose() * s.inverse(); // Ganancia de kalman
			
			if(debug)std::cout << "		K: \n"<<  k << "\n ------\n";

			v = z_i_t - z_i_t_hat;  // original menos lo esperadp_
			
			if(debug)std::cout << "		V: \n"<< v << "\n ------\n";
			
			
			Eigen::Vector3d ajuste_y;
			ajuste =  1*(k*v);

			if( (fabs(ajuste(0)) > 1.0) || (fabs(ajuste(1)) > 1.0)   )
			{
				if(debug)std::cout << "		SALTOOOOOO ....................." << std::endl;
				break;
			}

			//if(debug)std::cout << "	ajuste : " << ajuste << std::endl;
			if(0)
			for( int j = i; j < landmarks_detected.size(); ++j)
			{	
				if(debug)std::cout << "Antes x: \n"<< landmarks_detected[j].point.x << "\n";
				if(debug)std::cout << "Antes y: \n"<< landmarks_detected[j].point.y << "\n ------\n";
				landmarks_detected[j].point.x += ajuste(0);
				landmarks_detected[j].point.y += ajuste(1);
				
				double aux_angle_x =  landmarks_detected[j].point.x*cos(ajuste(2)) + landmarks_detected[j].point.y*sin(ajuste(2));
				double aux_angle_y = -landmarks_detected[j].point.x*sin(ajuste(2)) + landmarks_detected[j].point.y*cos(ajuste(2));
				landmarks_detected[j].point.x =  aux_angle_x;
				landmarks_detected[j].point.y = aux_angle_y;
				if(debug)std::cout << "Despues x: \n"<< landmarks_detected[j].point.x << "\n";
				if(debug)std::cout << "Despues y: \n"<< landmarks_detected[j].point.y << "\n ------\n";
			}
			x_ = x_ + (k*v); // Actualizacion vector de estado
			
			media += x_ ;
			n_land++;
			if(debug)std::cout << "		X: \n" << x_ << "\n ------\n";
			
			p = ( I - k * H ) * p; // Actualizacion matriz de covarianza
			
			if(debug)std::cout << "		P: \n" << p << "\n ------\n";

			//break;
			
			
		}
		if(n_land>1)
		x_ = media/n_land;
		//p << 0.07,0.07,0,0.07,0,0.07,0,0,0.07;

		UPDATE_LANDMARKS_FLAG = true;
	
		// Validacion para que el angulo obtenido siempre este entre pi y -pi
		//if (x_(2) > M_PI)
		//	x_(2) -= 2*M_PI;
		//if (x_(2) < -M_PI)
		//	x_(2) += 2*M_PI;
		x_(2) = normalize(x_(2));

		
		localizatio_pose.pose.position.x = x_(0);
		localizatio_pose.pose.position.y = x_(1);
		localizatio_pose.pose.position.z = 0;

		localizatio_pose.pose.orientation = tf::createQuaternionMsgFromYaw(x_(2));

		localizatio_pose.covariance[0] = p(0,0);
		localizatio_pose.covariance[1] = p(0,1);
		localizatio_pose.covariance[5] = p(0,2);
		localizatio_pose.covariance[6] = p(1,0);
		localizatio_pose.covariance[7] = p(1,1);
		localizatio_pose.covariance[11] = p(1,2);
		localizatio_pose.covariance[30] = p(2,0);
		localizatio_pose.covariance[31] = p(2,1);
		localizatio_pose.covariance[35] = p(2,2);

		pose_pub.publish(localizatio_pose);
		
		//if(debug)std::cout << "actualizacion Final x: \n" << x_ << "\n ------\n";
		//if(debug)std::cout << "actualizacion Final p: \n" << p << "\n ------\n";
		//f(debug)std::cout << "Final -----\n";

		return true;
	

}




void LandmarksPointCallBack (const kalman_loc_qr::LandmarksConstPtr& msg)
{ 	
	Landmark aux; 

  if(UPDATE_LANDMARKS_FLAG)
  {
	landmarks_detected.clear();

	for(int i = 0 ; i < msg->pointLandmarks.size(); i++)
	{
		
		aux.point = msg->pointLandmarks[i].point;
		aux.point.x += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.x;
		aux.point.y += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.y;
		aux.point.z += TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.z;

		//ROS_INFO(" x : %f y: %f z: %f", aux.point.x, aux.point.y, aux.point.z );

		aux.id = msg->ids[i];
		landmarks_detected.push_back(aux);

	}
  }
}



void odomCallback(const nav_msgs::Odometry& msg)
{
	robot_odom = msg ;
}




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Nodo_ekf");
	ros::NodeHandle nh;
	ros::Rate rate(10);
  	
	/*
		Topicos de entrada de informacion

		landmarksPoint -> Recibe las cordenadas de los puntos (landmarks) asi como su Id
		odom -> La odometria xD
	*/
	ros::Subscriber landmarks_sub = nh.subscribe ("landmarksPoint", 0, LandmarksPointCallBack);
  	ros::Subscriber odom_sub = nh.subscribe("odom", 0, odomCallback);
	
	/*
		Publica la posicion estimada por el EKF  
	*/
	pose_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("localization_ekf", 0);
	
	/* 
		Load parameters such as q_matrix, r_matrix, apha0,apha1,apha2,apha3
	*/
	XmlRpc::XmlRpcValue processMatrixConfig;
	
	q.setZero();
	if(ros::param::has("~q_matrix"))
    {
      try
      {
        ros::param::get("~q_matrix", processMatrixConfig);

        ROS_ASSERT(processMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

        int matSize = q.rows();

        for (int i = 0; i < matSize; i++)
        {
          for (int j = 0; j < matSize; j++)
          {
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << processMatrixConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> q(i, j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
              throw e;
            }
            catch(...)
            {
              throw;
            }
          }
        }

        std::cout << "Q matrix: \n" << q << "\n";
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading q_matrix");
      }

    }

	r.setZero();
	if(ros::param::has("~r_matrix"))
    {
      try
      {
        ros::param::get("~r_matrix", processMatrixConfig);

        ROS_ASSERT(processMatrixConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

        int matSize = r.rows();

        for (int i = 0; i < matSize; i++)
        {
          for (int j = 0; j < matSize; j++)
          {
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << processMatrixConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> r(i, j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
              throw e;
            }
            catch(...)
            {
              throw;
            }
          }
        }

        std::cout << "R matrix: \n" << r << "\n";
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading q_matrix");
      }

    }

	for(int i = 0; i < 4; i++)
	{
		if(ros::param::has("~alpha" + std::to_string(i+1)) )
		{
			ros::param::get("~alpha" + std::to_string(i+1), ALPHA[i] );
			
			if( isnan(ALPHA[i])  )
			{
				ROS_ERROR("Invalid dist_update");
				return 0;
			}
			else
			{
				ROS_INFO("Alpha %d: %f",i+1,ALPHA[i]);
			}
			
		}
		else
		{
			ALPHA[i] = 0.2;
			ROS_INFO("Default ~alpha%d: %f ", i+1 , ALPHA[i]);
			
		}
	}
	

	/*
		Load map of landmarks
	*/
	int id;
	double x,y,z;
	Landmark aux;
	landmarks_on_map.clear();
	std::string path_land_map;
	if(ros::param::has("~path_land_map"))
	{
		ros::param::get("~path_land_map", path_land_map );
		
		std::ifstream infile(path_land_map);
		if( infile.is_open() )
		{	
			std::cout << "Landmarks in (" << path_land_map << ") map file : "  << std::endl;
			while (infile >> id >> x >> y >> z)
			{   
				ROS_INFO("Id: %d",id );
				aux.point.x = x;
				aux.point.y = y;
				aux.point.z = z;
				aux.id = id;
				landmarks_on_map.push_back(aux);
			}
		}
		else
		{
			ROS_ERROR("The path_land_map is invalid U_U");
			return 0;
		}
	}
	else
	{
		std::cout << "Sorry U_U give a path_land_map. Try: _path_land_map:=landmark_map.txt "  << std::endl;
		return 0;
	}

	/*
		Initial Pose
	*/


	if( ros::param::has("~initial_x") ){
		ros::param::get("~initial_x",initial_x);
		ROS_ASSERT_MSG( !isnan(initial_x) , "Invalid initial_x: %f ", initial_x);
		ROS_INFO("initial_x: %f",initial_x);	
	}else
		ROS_INFO("Default initial_x: %f ",initial_x);

	if( ros::param::has("~initial_y") ){
		ros::param::get("~initial_y",initial_y);
		ROS_ASSERT_MSG( !isnan(initial_y) , "Invalid initial_y: %f ", initial_y);
		ROS_INFO("initial_y: %f",initial_y);	
	}else
		ROS_INFO("Default initial_y: %f ",initial_y);

	if( ros::param::has("~initial_a") ){
		ros::param::get("~initial_a",initial_a);
		ROS_ASSERT_MSG( !isnan(initial_a) , "Invalid initial_a: %f ", initial_a);
		ROS_INFO("initial_a: %f",initial_a);	
	}else
		ROS_INFO("Default initial_a: %f ",initial_a);
	
	localizatio_pose.pose.position.x = initial_x;
	localizatio_pose.pose.position.y = initial_y;
	localizatio_pose.pose.position.z = 0;

	localizatio_pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_a);

	localizatio_pose.covariance[0] = 0;
	localizatio_pose.covariance[1] = 0;
	localizatio_pose.covariance[5] = 0;
	localizatio_pose.covariance[6] = 0;
	localizatio_pose.covariance[7] = 0;
	localizatio_pose.covariance[11] = 0;
	localizatio_pose.covariance[30] = 0;
	localizatio_pose.covariance[31] = 0;
	localizatio_pose.covariance[35] = 0;

	pose_pub.publish(localizatio_pose);


	/*
		Variables for ekf trigger
	*/

	double dist_update;
	double angle_update; 

	if(ros::param::has("~dist_update"))
	{
		ros::param::get("~dist_update", dist_update );
		ROS_ASSERT_MSG( !isnan(dist_update), "Invalid dist_update: %f ", dist_update);
		ROS_INFO("dist_update: %f", dist_update);

	}
	else
	{
		dist_update = 0.5;
		ROS_INFO("Using DEFAULT dist_update:  %f ",dist_update);
	}

	if(ros::param::has("~angle_update"))
	{
		ros::param::get("~angle_update", angle_update);
		
		if( isnan(angle_update) )
		{
			ROS_ERROR("Invalid angle_update");
			return 0;
		}
		else
		{
			ROS_INFO("angle_update: %f", angle_update);
		}
		
	}
	else
	{
		angle_update = 0.03490658503;// 2grados
		ROS_INFO("Using DEFAULT angle_update:  %f ",angle_update);
	}

	/*
		Get base_link to device (kinect/stereocam) trasformation
	*/

	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);
	
	while (nh.ok())
	{	
		try{
			TRANSFORM_STAMPED_BASE_2_DEVICE = tfBuffer.lookupTransform( "base_link","kinect_link",
									ros::Time(0));
			ROS_INFO("TF_base_to_device: \n X: %f \n Y: %f \n Z: %f \n -----\n ",TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.x,
																				TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.y,
																				TRANSFORM_STAMPED_BASE_2_DEVICE.transform.translation.z);	
		break;
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}    
		rate.sleep();
	}

	/* 
		Waiting for  odom info
	*/
	std::cout << "Esperando odom topic...\n";
	while(odom_sub.getNumPublishers() == 0 || robot_odom.pose.pose.orientation.w == 0)
	{
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	};
	robot_odom_1 = robot_odom;
	//std::cout << "El odomsoso: " <<  robot_odom.pose.pose.orientation  << "\n";
	


	double roll,pitch,yaw;
	tf2::Quaternion quat_tf_1, quat_tf,quat_r;
	
	std::cout << "Waiting for robot's movements\n";
	while(ros::ok())
	{
		/*
			Get euler from quaternion to compare
		*/
		tf2::fromMsg(robot_odom_1.pose.pose.orientation, quat_tf_1);
		tf2::fromMsg(robot_odom.pose.pose.orientation, quat_tf);

		quat_tf_1[3] = -quat_tf_1[3];
		quat_r=quat_tf*quat_tf_1 ;
		quat_r.normalize();
		tf2::Matrix3x3 mtx_rot( quat_r );
		mtx_rot.getRPY(roll, pitch, yaw);

		/*
			Se revisa si el avance es mayor a min_dist_update o si el giro es mayor a min_angle_update
			si se cumple alguna de las 2 condiciones se ejecuta el algoritmo EKF
		*/

    	if( 
			( sqrt(pow(robot_odom_1.pose.pose.position.x - robot_odom.pose.pose.position.x,2) + pow(robot_odom_1.pose.pose.position.y - robot_odom.pose.pose.position.y,2)) > dist_update )
			||
			( fabs(yaw) > angle_update )
		)
		{
			ROS_INFO("EKF running");
			ekf();
			robot_odom_1 = robot_odom;
		}
		
		//std::cout << yaw << std::endl;
		//pose_pub.publish(localizatio_pose);
		ros::spinOnce();
		rate.sleep();
	}

}


