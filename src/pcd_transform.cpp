//pcd_transform.cpp
//
//引数:変換したいpcd_file roll pitch yaw x y z

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

using namespace std;	

sensor_msgs::PointCloud2 transformed_pc;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());


void pc2transform(float roll, float pitch, float yaw, float x, float y, float z){


	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(roll*(1), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch*(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation (x, y, z);

	Eigen::Matrix4f transform = (rot * init_translation).matrix ();

	pcl::transformPointCloud (*cloud_IN, *transformed_cloud, transform);
	pcl::toROSMsg(*transformed_cloud,transformed_pc);
	
}



int main (int argc, char** argv)
{

	ros::init(argc, argv, "pcd_transform");
	ros::NodeHandle n;
	ros::Rate roop(1);
	
	if (argc != 8)
	{
		cout << "引数確認"<<endl;
		return 0;
	}

	ros::Publisher input_pub = n.advertise<sensor_msgs::PointCloud2>("/transform_pcd_before", 100, true);
	ros::Publisher transform_pub = n.advertise<sensor_msgs::PointCloud2>("/transform_pcd", 100, true);
	

	if( pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}
	
	
	// pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_IN);

	sensor_msgs::PointCloud2 input_pc;
	pcl::toROSMsg(*cloud_IN, input_pc);

	input_pc.header.frame_id  = "map";	
	input_pc.header.stamp  = ros::Time::now();
	input_pub.publish(input_pc);

	pc2transform(atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]));

	transformed_pc.header.frame_id  = "map";	
	transformed_pc.header.stamp  = ros::Time::now();
	transform_pub.publish(transformed_pc);

	cout<<"保存しますか？y or n"<<endl;
	string s;
	cin >> s;
	if(s == "y"){
		cout<<"save now"<<endl;
		pcl::io::savePCDFileASCII ("transform_after_.pcd", *transformed_cloud);
	}
	else cout<<"破棄"<<endl;

	sleep(1.0);
	return (0);
}

