
//pcd_loader.cpp
//

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;	


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all (new pcl::PointCloud<pcl::PointXYZ>);


void integrate_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point)
{	
	size_t velodyne_size = input_point->points.size();
	cout<<velodyne_size<<endl;
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_point->points[i].x; 
		temp_point.y = input_point->points[i].y;
		temp_point.z = input_point->points[i].z;

		cloud_all->points.push_back(temp_point);
	}
}





int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_loader_");
	ros::NodeHandle n;
	ros::Rate roop(1);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 100, true);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZ>);

	int end = argc;

	for(int i=1;i<argc;i++){
	if( pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *cloud_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}
	integrate_map(cloud_IN);
	}

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_all, pc);

	pc.header.frame_id  = "map";	
	pc.header.stamp  = ros::Time::now();
	pub.publish(pc);

	sleep(1.0);
	return (0);
}
