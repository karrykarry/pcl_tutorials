//pcd_transform.cpp で ある程度の位置関係を得た後
//
//引数: target_pcd_file src_pcd_file roll pitch yaw x y z

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

using namespace std;	

sensor_msgs::PointCloud2 transformed_pc;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_IN (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_IN (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

void calc_rpy(Eigen::Matrix4f a,double &roll,double &pitch,double &yaw){
	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

	mat_l.getRPY(roll, pitch, yaw, 1);
}

void pc2transform_icp(float roll, float pitch, float yaw, float x, float y, float z){


	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(roll*(1), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch*(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation (x, y, z);

	Eigen::Matrix4f transform = (rot * init_translation).matrix ();


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(10.0);
	icp.setMaximumIterations(20);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	/*------ Voxel Grid ------*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr map_IN_ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_IN_ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	// vg.setLeafSize(0.3,0.3,0.3);
	vg.setLeafSize(0.3,0.3,0.3);
	vg.setInputCloud(map_IN);
	vg.filter(*map_IN_);
	vg.setInputCloud(local_map_IN);
	vg.filter(*local_map_IN_);

	Eigen::Matrix4f a;
	double l_roll, l_pitch, l_yaw;//角度etc

	icp.setInputTarget(map_IN_);
	icp.setInputSource(local_map_IN_);
	// icp.setInputTarget(map_IN);
	// icp.setInputSource(local_map_IN);
    icp.align (*cloud, transform);
	// icp.align(*cloud);

	pcl::toROSMsg(*cloud,transformed_pc);

	a =  icp.getFinalTransformation();
	calc_rpy(a,l_roll,l_pitch,l_yaw);
	
	cout<<"roll:"<<l_roll<<endl;
	cout<<"pitch:"<<l_pitch<<endl;
	cout<<"yaw:"<<l_yaw<<endl;
	cout<<"x:"<<a(0, 3)<<endl;
	cout<<"y:"<<a(1, 3)<<endl;
	cout<<"z:"<<a(2, 3)<<endl;

}



int main (int argc, char** argv)
{

	ros::init(argc, argv, "pcd_transform");
	ros::NodeHandle n;
	ros::Rate roop(1);
	
	if (argc != 9)
	{
		
		cout << "引数確認"<<endl;
		return 0;
	}

	ros::Publisher input_pub = n.advertise<sensor_msgs::PointCloud2>("/target_pcd", 100, true);
	ros::Publisher transform_pub = n.advertise<sensor_msgs::PointCloud2>("/after_transform_icp", 100, true);

	if( pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *map_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}

	if( pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *local_map_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}
	
	sensor_msgs::PointCloud2 input_pc;
	pcl::toROSMsg(*map_IN, input_pc);

	input_pc.header.frame_id  = "map";	
	input_pc.header.stamp  = ros::Time::now();
	input_pub.publish(input_pc);

	pc2transform_icp(atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]));

	transformed_pc.header.frame_id  = "map";	
	transformed_pc.header.stamp  = ros::Time::now();
	transform_pub.publish(transformed_pc);

	cout<<"保存しますか？y or n"<<endl;
	string s;
	cin >> s;
	if(s == "y"){
		cout<<"save now"<<endl;
		pcl::io::savePCDFileASCII ("after_icp.pcd", *cloud);
	}
	else cout<<"破棄"<<endl;

	sleep(1.0);
	return (0);
}

