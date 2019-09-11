
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <boost/thread.hpp>


#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

int start_index = 0;

float cluster_size[100];

float MIN_SIZE = 0.01;
float MAX_SIZE = 1.6;
float Tolerance = 0.1;
int min_num = 20;
int max_num = 1000;

template <class T>
void getParam(ros::NodeHandle &n, string param, T &val){
    string str;
    if(!n.hasParam(param)){
        cout << param << " don't exist." << endl;
    }   

    if(!n.getParam(param, str)){
        cout << "NG" << endl;
    }   
    std::stringstream ss(str);
    T rsl;
    ss >> rsl;
    val = rsl;
    cout << param << " = " << str << endl;
}


bool getParams(ros::NodeHandle& n)
{
	getParam(n, "MIN_SIZE", MIN_SIZE);
	getParam(n, "MAX_SIZE", MAX_SIZE);
	getParam(n, "Tolerance", Tolerance);
	getParam(n, "Min_num", min_num);
	getParam(n, "Max_num", max_num);
	return true;
}

void pubPC2Msg(ros::Publisher& pub, CloudA& p_in, std::string& frame_id, ros::Time& time)
{
	sensor_msgs::PointCloud2 p_out;
	toROSMsg(p_in, p_out);
	p_out.header.frame_id = frame_id;
	p_out.header.stamp    = time;
	pub.publish(p_out);
}

double getClusterInfo(CloudA pt, PointA &cluster)
{
	////calculate_centroid_section
	// Vector3f centroid=Vector3f::Zero(3);
	Vector3f centroid;
	centroid[0] = pt.points[0].x;
	centroid[1] = pt.points[0].y;
	centroid[2] = pt.points[0].z;

	Vector2f min_p;
	min_p[0] = pt.points[0].x;
	min_p[1] = pt.points[0].y;

	Vector3f max_p;
	max_p[0] = pt.points[0].x;
	max_p[1] = pt.points[0].y;
	max_p[2] = pt.points[0].z;

	for(size_t i=1;i<pt.points.size();i++){
		centroid[0] += pt.points[i].x;
		centroid[1] += pt.points[i].y;
		centroid[2] += pt.points[i].z;
		if (pt.points[i].x<min_p[0]) min_p[0] = pt.points[i].x;
		if (pt.points[i].y<min_p[1]) min_p[1] = pt.points[i].y;

		if (pt.points[i].x>max_p[0]) max_p[0] = pt.points[i].x;
		if (pt.points[i].y>max_p[1]) max_p[1] = pt.points[i].y;
		if (pt.points[i].z>max_p[2]) max_p[2] = pt.points[i].z;
	}

	cluster.x         = centroid[0]/(float)pt.points.size();
	cluster.y         = centroid[1]/(float)pt.points.size();
	cluster.z         = centroid[2]/(float)pt.points.size();
	cluster.normal_x  = max_p[0]-min_p[0];
	cluster.normal_y  = max_p[1]-min_p[1];
	cluster.normal_z  = max_p[2];
	cluster.intensity = start_index; 
	cluster.curvature = pt.points.size(); //component size

	start_index += pt.points.size(); 

	return cluster.normal_x * cluster.normal_y * cluster.normal_z;
}

void cpu_clustering(CloudAPtr pcl_in, CloudA& cluster_centroid, CloudA& dynamic_centroid, CloudA& cluster_pt, CloudA& cloud_filtered, CloudA& cloud_rm_object)/*{{{*/
{
	// //Downsample//
	// pcl::VoxelGrid<pcl::PointXYZINormal> vg;  
	// CloudAPtr ds_cloud (new CloudA);  
	// vg.setInputCloud (pcl_in);  
	// vg.setLeafSize (0.07f, 0.07f, 0.07f);
	// vg.filter (*ds_cloud);


	//downsampled point's z =>0
	vector<float> tmp_z;
	tmp_z.resize(pcl_in->points.size());
	for(int i=0;i<(int)pcl_in->points.size();i++){
		tmp_z[i]=pcl_in->points[i].z;
		pcl_in->points[i].z  = 0.0;
	}
	// for(int i=0;i<(int)pcl_in_->points.size();i++){
	// 	tmp_z[i]=pcl_in_->points[i].z;
	// 	if(pcl_in_->points[i].normal_y >100) continue;
	// 	pcl_tutorials
	// 	
	// 	pcl_in->points[i].x=pcl_in_->points[i].x;
	// 	pcl_in->points[i].y=pcl_in_->points[i].y;
	// 	pcl_in->points[i].z=pcl_in_->points[i].z;
	// 	pcl_in->points[i].normal_x=pcl_in_->points[i].normal_x;
	// 	pcl_in->points[i].normal_y=pcl_in_->points[i].normal_y;
	// 	pcl_in->points[i].normal_z=pcl_in_->points[i].normal_z;
	// 	pcl_in->points[i].intensity=pcl_in_->points[i].intensity;
	// 	pcl_in->points[i].curvature=pcl_in_->points[i].curvature;
	// 		
	// 	pcl_in->points[i].z  = 0.0;
	// }


	//Clustering//
	// Creating the KdTree object for the search method of the extraction
	// clock_t clustering_start=clock();
	pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
	tree->setInputCloud (pcl_in);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
	// ec.setClusterTolerance (0.10); // 10cm
	ec.setClusterTolerance (Tolerance);
	ec.setMinClusterSize (min_num);
	ec.setMaxClusterSize (max_num);
	ec.setSearchMethod (tree);
	ec.setInputCloud(pcl_in);
	ec.extract (cluster_indices);
	// std::cout<<"clustering time="<<(double)(clock()-clustering_start)/CLOCKS_PER_SEC<<std::endl;

	//reset z value
	for(int i=0;i<(int)pcl_in->points.size();i++)
		pcl_in->points[i].z=tmp_z[i];

	//get cluster information//
	size_t num=0;
	cluster_centroid.resize(cluster_indices.size());
	dynamic_centroid.resize(cluster_indices.size());
	for(size_t iii=0;iii<cluster_indices.size();iii++){
		//cluster cenrtroid
		CloudAPtr cloud_cluster (new CloudA);
		cloud_cluster->points.resize(cluster_indices[iii].indices.size());
		//cluster points
		cluster_pt.points.resize(cluster_indices[iii].indices.size()+num);
		cloud_filtered.points.resize(cluster_indices[iii].indices.size()+num);
		cloud_rm_object.points.resize(cluster_indices[iii].indices.size()+num);

		for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
			int p_num = cluster_indices[iii].indices[jjj];
			cloud_cluster->points[jjj] = pcl_in->points[p_num];
			cluster_pt.points[num+jjj] = pcl_in->points[p_num];
		}
		//get bounding box's centroid
		// Vector3f centroid=calculateCentroid(*cloud_cluster);
		cluster_size[iii] = getClusterInfo(*cloud_cluster, cluster_centroid[iii]);
		// cout<<"cluster_size["<<iii<<"] = "<<cluster_size[iii]<<endl;

		// dynamic_centroid[iii].x = cluster_centroid[iii].x;
		// dynamic_centroid[iii].y = cluster_centroid[iii].y;
		// dynamic_centroid[iii].z = cluster_centroid[iii].z;
		

		for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
			int p_num = cluster_indices[iii].indices[jjj];
			if(MIN_SIZE < cluster_size[iii] && cluster_size[iii] < MAX_SIZE) {
				cloud_filtered.points[num+jjj] = pcl_in->points[p_num];
				dynamic_centroid[iii].x = cluster_centroid[iii].x;
				dynamic_centroid[iii].y = cluster_centroid[iii].y;
				dynamic_centroid[iii].z = cluster_centroid[iii].z;
				dynamic_centroid[iii].intensity = cluster_centroid[iii].intensity;
				dynamic_centroid[iii].curvature = cluster_centroid[iii].curvature;
				// cout<<"if"<<num+jjj<<endl;
			}
			else {
				cloud_rm_object.points[num+jjj] = pcl_in->points[p_num];
				dynamic_centroid[iii].x = -10000;
				dynamic_centroid[iii].y = -10000;
				dynamic_centroid[iii].z = -100;
				dynamic_centroid[iii].intensity = 0;
				dynamic_centroid[iii].curvature = 0;
				// cout<<"else "<<num+jjj<<endl;
			}
		}

		// getClusterInfo(cloud_filtered, dynamic_centroid[iii]);


		// if(0.01 < cluster_size < 0.5) cloud_filtered = cloud_cluster; 
		// getClusterInfo(*cloud_cluster, cloud_filtered[iii]);
		// cloud_filtered = *cloud_cluster;

		// cluster_centroid[iii].x=centroid[0];
		// cluster_centroid[iii].y=centroid[1];
		// cluster_centroid[iii].z=centroid[2];
		//the number of points which constitute cluster[iii]
		cluster_centroid[iii].curvature=cloud_cluster->points.size();
		//the start index of cluster[iii]
		// cluster_centroid[iii].normal_x=num;
		cluster_centroid[iii].intensity=num;
		num=cluster_pt.points.size();//save previous size
	}
		start_index = 0;
}/*}}}*/


// void cpu_clustering(CloudAPtr pcl_in, CloudA& cluster_centroid, CloudA& dynamic_centroid, CloudA& cluster_pt, CloudA& cloud_filtered, CloudA& cloud_rm_object)/*{{{*/
// {
//
// 	//downsampled point's z =>0
// 	vector<float> tmp_z;
// 	tmp_z.resize(pcl_in->points.size());
// 	for(int i=0;i<(int)pcl_in->points.size();i++){
// 		tmp_z[i]=pcl_in->points[i].z;
// 		pcl_in->points[i].z  = 0.0;
// 	}
//
//
// 	//Clustering//
// 	// Creating the KdTree object for the search method of the extraction
// 	// clock_t clustering_start=clock();
// 	pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
// 	tree->setInputCloud (pcl_in);
// 	std::vector<pcl::PointIndices> cluster_indices;
// 	pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
// 	ec.setClusterTolerance (0.09); // 9cm
// 	ec.setMinClusterSize (20);
// 	ec.setMaxClusterSize (16000);
// 	ec.setSearchMethod (tree);
// 	ec.setInputCloud(pcl_in);
// 	ec.extract (cluster_indices);
// 	// std::cout<<"clustering time="<<(double)(clock()-clustering_start)/CLOCKS_PER_SEC<<std::endl;
//
// 	//reset z value
// 	for(int i=0;i<(int)pcl_in->points.size();i++)
// 		pcl_in->points[i].z=tmp_z[i];
//
// 	//get cluster information//
// 	size_t num=0;
// 	cluster_centroid.resize(cluster_indices.size());
// 	dynamic_centroid.resize(cluster_indices.size());
// 	for(size_t iii=0;iii<cluster_indices.size();iii++){
// 		//cluster cenrtroid
// 		CloudAPtr cloud_cluster (new CloudA);
// 		cloud_cluster->points.resize(cluster_indices[iii].indices.size());
// 		//cluster points
// 		cluster_pt.points.resize(cluster_indices[iii].indices.size()+num);
// 		cloud_filtered.points.resize(cluster_indices[iii].indices.size()+num);
// 		cloud_rm_object.points.resize(cluster_indices[iii].indices.size()+num);
//
// 		for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
// 			int p_num = cluster_indices[iii].indices[jjj];
// 			cloud_cluster->points[jjj] = pcl_in->points[p_num];
// 			cluster_pt.points[num+jjj] = pcl_in->points[p_num];
// 		}
// 		//get bounding box's centroid
// 		// Vector3f centroid=calculateCentroid(*cloud_cluster);
// 		cluster_size[iii] = getClusterInfo(*cloud_cluster, cluster_centroid[iii]);
// 		// cout<<"cluster_size["<<iii<<"] = "<<cluster_size[iii]<<endl;
//
// 		// dynamic_centroid[iii].x = cluster_centroid[iii].x;
// 		// dynamic_centroid[iii].y = cluster_centroid[iii].y;
// 		// dynamic_centroid[iii].z = cluster_centroid[iii].z;
//
// 		for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
// 			int p_num = cluster_indices[iii].indices[jjj];
// 			if(MIN_SIZE < cluster_size[iii] && cluster_size[iii] < MAX_SIZE) {
// 				cloud_filtered.points[num+jjj] = pcl_in->points[p_num];
// 				dynamic_centroid[iii].x = cluster_centroid[iii].x;
// 				dynamic_centroid[iii].y = cluster_centroid[iii].y;
// 				dynamic_centroid[iii].z = cluster_centroid[iii].z;
// 				// cout<<"if"<<num+jjj<<endl;
// 			}
// 			else {
// 				cloud_rm_object.points[num+jjj] = pcl_in->points[p_num];
// 				// cout<<"else "<<num+jjj<<endl;
// 			}
// 		}
//
// 		// getClusterInfo(cloud_filtered, dynamic_centroid[iii]);
//
//
// 		// if(0.01 < cluster_size < 0.5) cloud_filtered = cloud_cluster; 
// 		// getClusterInfo(*cloud_cluster, cloud_filtered[iii]);
// 		// cloud_filtered = *cloud_cluster;
//
// 		// cluster_centroid[iii].x=centroid[0];
// 		// cluster_centroid[iii].y=centroid[1];
// 		// cluster_centroid[iii].z=centroid[2];
// 		//the number of points which constitute cluster[iii]
// 		cluster_centroid[iii].curvature=cloud_cluster->points.size();
// 		//the start index of cluster[iii]
// 		// cluster_centroid[iii].normal_x=num;
// 		cluster_centroid[iii].intensity=num;
// 		num=cluster_pt.points.size();//save previous size
// 	}
//
// }/*}}}*/

boost::mutex pt_mutex;
bool Callback_flag = false;
CloudAPtr rmg_pt (new CloudA);

void objectCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	boost::mutex::scoped_lock(pt_mutex);
	pcl::fromROSMsg(*msg,*rmg_pt);
	Callback_flag=true;
}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "clustering");
	ros::NodeHandle n;

	ros::Publisher cluster_centroid_pub = n.advertise<sensor_msgs::PointCloud2>("/cluster/centroid",1); // クラスタの重心
	ros::Publisher dynamic_centroid_pub = n.advertise<sensor_msgs::PointCloud2>("/dynamic_cluster/centroid",1); // 動的物体と思われるクラスタの重心
	ros::Publisher cluster_points_pub   = n.advertise<sensor_msgs::PointCloud2>("/cluster/points",1); // クラスタリングされた点群
	ros::Publisher dynamic_cluster_pub   = n.advertise<sensor_msgs::PointCloud2>("/dynamic_cluster/points",1); // 動的物体と思われるクラスタ
	ros::Publisher static_cluster_pub   = n.advertise<sensor_msgs::PointCloud2>("/static_cluster/points",1); // 静的物体と思われるクラスタ
		
	ros::Subscriber object_sub = n.subscribe("velodyne_points", 1, objectCallback);

	ros::Rate loop_rate(20);

	getParams(n);
	cout<<"MIN_SIZE = "<<MIN_SIZE<<endl;
	cout<<"MAX_SIZE = "<<MAX_SIZE<<endl;

	while (ros::ok()){
		if (Callback_flag){
			
			CloudA cluster_centroid, dynamic_centroid, cluster_points, dynamic_cluster, static_cluster;
			cpu_clustering(rmg_pt, cluster_centroid, dynamic_centroid, cluster_points, dynamic_cluster, static_cluster);
            

			std::string frame_id = "velodyne";
			ros::Time time= ros::Time::now();
			pubPC2Msg(cluster_centroid_pub, cluster_centroid, frame_id, time);
			pubPC2Msg(dynamic_centroid_pub, dynamic_centroid, frame_id, time);
			pubPC2Msg(cluster_points_pub, cluster_points, frame_id, time);
			pubPC2Msg(dynamic_cluster_pub, dynamic_cluster, frame_id, time);
			pubPC2Msg(static_cluster_pub, static_cluster, frame_id, time);
			Callback_flag = false;        
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}
