#ifndef VIEWPOINT_GENERATOR_H
#define VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <algorithm>
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
#include <pcl/surface/concave_hull.h>

using std::cout; using std::endl;
namespace globalPlanner{
	struct ClusterInfo {
		std::vector<Eigen::Vector3d> vert;  // 8 vertices of the AABB
		Eigen::Vector3d normal;                     // Average normal of the cluster
		Eigen::Vector3d centroid;
	};

	struct ViewPoint {
		Eigen::Vector3d pose;
		double yaw;
	};

	// struct VPSetInfo {
	// 	int vpNum;

	// }

	struct occMap {
		int width, height, depth;
		std::vector<bool> occ;
		std::vector<double> reward;
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
		ros::Publisher blockedVisPub_;

		// Param
		std::string mapDir_;
		double offset_;
		double step_, stepZ_;
		double groundHgt_, ceilingHgt_;
		double resolution_;

		double minClusterSize_, maxClusterSize_;
		double curvThres_;
		double angThres_;

		int mergeThres_;

		Eigen::Vector3d mapMin_, mapMax_;
		Eigen::Vector3d currPos_{0.0, 0.0, 1.0};
		pcl::PointCloud<pcl::PointXYZ> refCloud_;
		occMap occupancy_;
		std::vector<ClusterInfo> segMap_;
		std::vector<std::vector<ViewPoint>> vpCluster_; // unarranged viewpoints
		std::vector<std::vector<ViewPoint>> vpSetRaw_;
		std::vector<std::vector<ViewPoint>> vpSet_;
		// std::vector<int> vpSeq_;
		int n;
		
	public:
		vpPlanner(const ros::NodeHandle& nh);

		void initParam();
		void registerPub();
		void registerCallback();

		void initMap();
		void genOccMap();
		int getIdx(double x, double y, double z);
		bool isInMap(double x, double y, double z);
		bool isOccupied(double x, double y, double z);
		void segMap();
		ClusterInfo genClusterInfo(const Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ> &cluster);
		void makePlan();
		bool vpHasCollision(const Eigen::Vector3d &viewpoint);
		std::vector<std::vector<ViewPoint>> solveSequence();
		// std::vector<int> solveSequence(const std::vector<std::vector<ViewPoint>> &vpSet);
		std::vector<std::vector<ViewPoint>> rearrangeVP(const std::vector<int> &vpSeq);
		std::pair<bool, std::pair<int, int>> merge(const std::array<int, 3> &original, const std::array<int, 3> &target);
		
		// helper function:
		std::pair<int, int> getSegIdx(const int &targetIdx);
		double getDistance(const int &Seg1Idx, const int & p1Idx, const int &Seg2Idx, const int & p2Idx);
		void visCB(const ros::TimerEvent&);
		void publishMap();
		void publishSeg();
		void publishBlockedPoint();
		void publishViewPoints(const std::vector<std::vector<ViewPoint>> &vpSet);

		// user functions
		std::vector<std::vector<Eigen::Vector4d>> getViewpoints();
		void updateCurrPos(const Eigen::Vector3d &currPos);
		void updateInaccessibleView(const std::vector<Eigen::Vector2i> &inaccessibleIdx);
		double updateViewAngle(const std::vector<std::vector<Eigen::Vector3d>> &hitPoints, const double &yaw);
		double getReward(const Eigen::Vector3d &hitPoint);
	};
	
}

#endif