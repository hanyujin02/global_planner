#ifndef VIEWPOINT_GENERATOR_H
#define VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

using std::cout; using std::endl;
namespace globalPlanner{
	struct ClusterInfo {
		std::vector<Eigen::Vector3d> vert;  // 8 vertices of the AABB
		Eigen::Vector3d normal;                     // Average normal of the cluster
	};

	struct ViewPoint {
		Eigen::Vector3d pose;
		double yaw;
	};

	class vpPlanner{
	private:
		std::string ns_;
		std::string hint_;

		// ROS
		ros::NodeHandle nh_;
		ros::Timer visTimer_;
		ros::Publisher mapVisPub_;
		ros::Publisher pointVisPub_;
		ros::Publisher segMapVisPub_;
		ros::Publisher normalVisPub_;

		// Param
		std::string mapDir_;

		pcl::PointCloud<pcl::PointXYZ> refCloud_;
		std::vector<ClusterInfo> segMap_;
		std::vector<std::vector<ViewPoint>> vpSet_;
	public:
		vpPlanner(const ros::NodeHandle& nh);

		void initParam();
		void registerPub();
		void registerCallback();
		void initMap();
		ClusterInfo genClusterInfo(const Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ> &cluster);
		void segMap();
		void generateViewPoint();

		void visCB(const ros::TimerEvent&);
		void publishMap();
		void publishSeg();
		void publishViewPoints();

		

	};
	
}

#endif