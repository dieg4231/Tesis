#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "geometry_msgs/PoseWithCovariance.h"
#include "std_msgs/Float32MultiArray.h"

// Viz message
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

ros::Publisher blob_pub;

void localizationEkfCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& data)
{
    int debug = 1;
	
	Eigen::MatrixXcd A(2,2);	
			
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			A(i,j) = std::complex<double>(data->covariance[(6*i)+j],0.0);
			std::cout << A(i,j).real() << " ";
		}
		printf("\n");
	}
		

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> es(A);

	if(debug)
	{
		printf("Eigenvalues \n");
		for(int i = 0; i < 2; i++)
		{
			printf("\t%d: \t", i+1);
			std::cout << es.eigenvalues()[i] << " ";
		}
		printf("\n");
		std::cout << es.eigenvectors() << "\n";
	}

    visualization_msgs::Marker blob;
    blob.header.frame_id = "map";
    blob.header.stamp = ros::Time();
    blob.ns = "my_namespace2";
    blob.id = 0;
    blob.type = visualization_msgs::Marker::SPHERE;
    blob.action = visualization_msgs::Marker::ADD;
    blob.lifetime = ros::Duration(5);
    blob.pose.position.x = data->pose.position.x;
    blob.pose.position.y = data->pose.position.y;
    blob.pose.position.z = 1.0;
    
    blob.color.a = 0.5; // Don't forget to set the alpha!
    blob.color.r = 1.0;
    blob.color.g = 0.8;
    blob.color.b = 0.0;

	if(es.eigenvalues()[0] > es.eigenvalues()[1])
	{
        blob.pose.orientation = tf::createQuaternionMsgFromYaw(atan2( es.eigenvectors().col(0)[1].real() , es.eigenvectors().col(0)[0].real() ));
        blob.scale.x = sqrt( 5.991* fabs(es.eigenvalues()[0])) ;
        blob.scale.y = sqrt( 5.991* fabs(es.eigenvalues()[1]));
        blob.scale.z = 1;

	}
	else
	{
        blob.pose.orientation = tf::createQuaternionMsgFromYaw(atan2( es.eigenvectors().col(1)[1].real() , es.eigenvectors().col(1)[0].real() ));
        blob.scale.x = sqrt( 5.991* fabs(es.eigenvalues()[1])) ;
        blob.scale.y = sqrt( 5.991* fabs(es.eigenvalues()[0]));
        blob.scale.z = 1;
	}

    blob_pub.publish(blob);
	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kalman_to_blobs");
	ros::NodeHandle nh;

	ros::Subscriber localization_ekf_sub = nh.subscribe("localization_ekf", 0, localizationEkfCallback);
    blob_pub = nh.advertise<visualization_msgs::Marker>( "uncertanty_blob", 0 );
	ros::Rate loop_rate(50);
	
	while (ros::ok())
	{
		loop_rate.sleep();
        ros::spinOnce();
	}

}