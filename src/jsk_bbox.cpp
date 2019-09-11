
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

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

ros::Publisher bb_pos_pub_;

boost::mutex pt_mutex;
bool Callback_flag = false;
CloudAPtr rmg_pt (new CloudA);

void pubPC2Msg(ros::Publisher& pub, CloudA& p_in, std::string& frame_id, ros::Time& time)
{
	sensor_msgs::PointCloud2 p_out;
	toROSMsg(p_in, p_out);
	p_out.header.frame_id = frame_id;
	p_out.header.stamp    = time;
	pub.publish(p_out);
}

void visualizeBBox_j(jsk_recognition_msgs::BoundingBoxArray& bbox, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& bbox_pt, const int& mode)
{
	bbox.header.frame_id="/velodyne";
	bbox.header.stamp=ros::Time::now();
	bbox.boxes.resize(bbox_pt->points.size());
	for(size_t i=0;i<bbox_pt->points.size();i++){
		bbox.boxes[i].header.frame_id="/velodyne";
		bbox.boxes[i].header.stamp=ros::Time::now();

		
		bbox.boxes[i].pose.position.x =bbox_pt->points[i].x;
		bbox.boxes[i].pose.position.y =bbox_pt->points[i].y;
		bbox.boxes[i].pose.position.z =bbox_pt->points[i].z;
		bbox.boxes[i].pose.orientation.x =0.0;
		bbox.boxes[i].pose.orientation.y =0.0;
		bbox.boxes[i].pose.orientation.z =0.0;
		bbox.boxes[i].pose.orientation.w =1.0;
		
		bbox.boxes[i].dimensions.x =5.0;
		bbox.boxes[i].dimensions.y =3.0;
		bbox.boxes[i].dimensions.z =2.0;
	}
}



void objectCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	boost::mutex::scoped_lock(pt_mutex);
	pcl::fromROSMsg(*msg,*rmg_pt);
	jsk_recognition_msgs::BoundingBoxArray bb_pos_;
	visualizeBBox_j(bb_pos_,rmg_pt,1);
	bb_pos_pub_.publish(bb_pos_);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "jsk_bbox");
	ros::NodeHandle n;

	bb_pos_pub_ = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cluster/jsk_bounding_box",1);
		
	ros::Subscriber object_sub = n.subscribe("cluster/centroid", 1, objectCallback);

	ros::spin();
	
	return (0);
}

