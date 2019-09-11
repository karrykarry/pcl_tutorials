#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_map_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);

ros::Publisher pc_pub;
sensor_msgs::PointCloud2 vis_map;


void sq_lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::fromROSMsg (*input, *input_cloud);
	filtered_map_cloud->points.clear();


    pcl::ApproximateVoxelGrid<pcl::PointXYZINormal> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_map_cloud);

    pcl::toROSMsg(*filtered_map_cloud , vis_map);           
    //limit_laserを設定
           
	vis_map.header.frame_id = "/velodyne"; //laserのframe_id
	vis_map.header.stamp = ros::Time::now(); //laserのframe_id

	pc_pub.publish(vis_map);

}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "voxel");
    
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/velodyne_obstacles", 1000, sq_lidarCallback);
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_voxel", 1000);

	ros::spin();

	return (0);
}

