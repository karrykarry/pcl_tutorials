//
// 引数にくっつけたいpcdを取る
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


pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal> cloud_all;



int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_union");
	ros::NodeHandle n;
	ros::Rate roop(1);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 100, true);

	int end = argc;

	for(int i=1;i<argc;i++){
		if( pcl::io::loadPCDFile<pcl::PointXYZINormal>(argv[i], *cloud_IN) == -1 ){
			cout << "load error !!\n";
			exit(1);
		}
		if(argc==1)cloud_all = *cloud_IN;
		else cloud_all += *cloud_IN;
	}
	cout<<"合体完了"<<endl;

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(cloud_all, pc);

	pc.header.frame_id  = "map";	
	pc.header.stamp  = ros::Time::now();
	pub.publish(pc);


	cout<<"保存しますか？y or n"<<endl;
	string s;
	cin >> s;
	if(s == "y"){
		cout<<"save now"<<endl;
		pcl::io::savePCDFileASCII ("pcd_union.pcd", cloud_all);
	}
	else cout<<"破棄"<<endl;


	sleep(1.0);
	return (0);
}
