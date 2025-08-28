#ifndef VIEWPOINT_GENERATOR_H
#define VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_manager/occupancyMap.h>
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
				
		std::shared_ptr<mapManager::occMap> mapRT_;
		Eigen::Vector3d mapMin_, mapMax_;
		Eigen::Vector3d currPos_{0.0, 0.0, 1.0};
		pcl::PointCloud<pcl::PointXYZ> refCloud_;
		occMap occupancy_;
		std::vector<ClusterInfo> segMap_;
		std::vector<std::vector<ViewPoint>> vpCluster_; // unarranged viewpoints
		std::vector<std::vector<ViewPoint>> vpSetRaw_;
		std::vector<std::vector<ViewPoint>> vpSet_; // processed viewpoints for execution
		int n;
		
		std::vector<Eigen::Vector2i> vpIdx_;
		int goalIdx_ = -1;
		int segIdx_;
	public:
		vpPlanner(const ros::NodeHandle& nh);

		void initParam();
		void registerPub();
		void registerCallback();

		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void initMap();
		void genOccMap();
		void segMap();
		ClusterInfo genClusterInfo(const Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ> &cluster);
		void makePlan();
		void initViewpoints();

		std::vector<std::vector<ViewPoint>> solveSequence();
		std::vector<std::vector<ViewPoint>> rearrangeVP(const std::vector<int> &vpSeq);
		std::pair<bool, std::pair<int, int>> merge(const std::array<int, 3> &original, const std::array<int, 3> &target);
		
		
		void visCB(const ros::TimerEvent&);
		void publishMap();
		void publishSeg();
		void publishBlockedPoint();
		void publishViewPoints(const std::vector<std::vector<ViewPoint>> &vpSet);
		
		// helper function:
		std::pair<int, int> getSegIdx(const int &targetIdx);
		double getDistance(const int &Seg1Idx, const int & p1Idx, const int &Seg2Idx, const int & p2Idx);
		bool vpHasCollision(const Eigen::Vector3d &viewpoint);
		bool vpHasCollisionRT(const Eigen::Vector3d &viewpoint);

		int toLinearIdx(int ix, int iy, int iz);
    	bool indexInRange(int ix, int iy, int iz);
		int getIdx(double x, double y, double z);
		bool isInMap(double x, double y, double z);
		bool isOccupied(double x, double y, double z);
		bool isSurfaceVoxel(double x, double y, double z);

		// user functions
		std::vector<std::vector<Eigen::Vector4d>> getViewpoints();
		bool getNewGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &yawTuning, double &yaw);
		bool getNewReplanGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &yawTuning, double &yaw);
		void updateCurrPos(const Eigen::Vector3d &currPos);
		void getInaccessibleView(std::vector<Eigen::Vector2i> &inaccessibleIdx);
		void updateInaccessibleView(const std::vector<Eigen::Vector2i> &inaccessibleIdx);
		double updateViewAngle(const std::vector<std::vector<Eigen::Vector3d>> &hitPoints, const double &yaw);
		double getReward(const Eigen::Vector3d &hitPoint);
	};
	
	inline int vpPlanner::toLinearIdx(int ix, int iy, int iz) {
        return ix + this->occupancy_.width * (iy + this->occupancy_.height * iz);
    }

    inline bool vpPlanner::indexInRange(int ix, int iy, int iz) {
        return (0 <= ix && ix < this->occupancy_.width) &&
            (0 <= iy && iy < this->occupancy_.height) &&
            (0 <= iz && iz < this->occupancy_.depth);
    }

    // Safer: returns -1 if OOB
    inline int vpPlanner::getIdx(double xPos, double yPos, double zPos){
        const double h = this->resolution_;

        // Convert to zero-based integer indices using floor
        int ix = static_cast<int>(std::floor((xPos - this->mapMin_(0)) / h));
        int iy = static_cast<int>(std::floor((yPos - this->mapMin_(1)) / h));
        int iz = static_cast<int>(std::floor((zPos - this->mapMin_(2)) / h));

        if (!indexInRange(ix, iy, iz)) return -1;
        return toLinearIdx(ix, iy, iz);
    }

    // Prefer half-open check: [min, max)
    inline bool vpPlanner::isInMap(double x, double y, double z){
        return ((x >= this->mapMin_(0)) && (x < this->mapMax_(0)) &&
                (y >= this->mapMin_(1)) && (y < this->mapMax_(1)) &&
                (z >= this->mapMin_(2)) && (z < this->mapMax_(2)));
    }

    inline bool vpPlanner::isOccupied(double x, double y, double z){
        int idx = this->getIdx(x, y, z);
        return this->occupancy_.occ[idx]; // adjust to your encoding
    }

    inline bool vpPlanner::isSurfaceVoxel(double x, double y, double z) {
        if (this->isInMap(x, y, z) && this->isOccupied(x, y, z)) {
            const double voxelSize = 0.1;
            const std::array<std::array<double,3>,6> neighborOffsets = {{
                { voxelSize,  0.0,       0.0 },
                {-voxelSize,  0.0,       0.0 },
                { 0.0,        voxelSize, 0.0 },
                { 0.0,       -voxelSize, 0.0 },
                { 0.0,        0.0,       voxelSize },
                { 0.0,        0.0,      -voxelSize }
            }};

            for (const auto& off : neighborOffsets) {
                double nx = x + off[0];
                double ny = y + off[1];
                double nz = z + off[2];

                if (!this->isInMap(nx, ny, nz))
                    continue; // skip out of bounds neighbors

                if (!this->isOccupied(nx, ny, nz)) {
                    return true; // found free neighbor → surface voxel
                }
            }
            return false; // all neighbors occupied → interior voxel
        } else {
            return false; // not in map or not occupied
        }
    }
}

#endif