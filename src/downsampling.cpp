#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

	if( pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.5f, 0.5f, 0.5f);
	sor.filter (*cloud_filtered);

	cout<<"保存しますか？y or n"<<endl;
	string s;
	cin >> s;
	if(s == "y"){
		cout<<"save now"<<endl;
		pcl::io::savePCDFileASCII ("after_downsampling.pcd", *cloud_filtered);
	}
	else cout<<"破棄"<<endl;



	return (0);
}
