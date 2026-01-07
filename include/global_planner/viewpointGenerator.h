#ifndef VIEWPOINT_GENERATOR_H
#define VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
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
		ros::Timer replanTimer_;
		ros::Publisher polygonVisPub_;
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
		double mapRes_;

		double minClusterSize_, maxClusterSize_;
		double curvThres_;
		double angThres_;

		int mergeThres_;
		bool manualVert_;
		std::vector<Eigen::Vector3d> poly_;
		
		std::shared_ptr<mapManager::occMap> mapRT_;
		Eigen::Vector3d mapSizeMin_, mapSizeMax_; // reserved min/max map size
		Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
		Eigen::Vector3d currPos_{0.0, 0.0, 1.0};
		pcl::PointCloud<pcl::PointXYZ> refCloud_;
		std::vector<bool> occupancy_;
		std::vector<double> reward_;
		std::vector<ClusterInfo> segMap_;
		std::vector<std::vector<ViewPoint>> vpCluster_; // unarranged viewpoints
		std::vector<std::vector<ViewPoint>> vpSetRaw_;
		std::vector<std::vector<ViewPoint>> vpSet_; // processed viewpoints for execution
		int n;
		std::vector<std::vector<Eigen::Vector2i>> inaccessibleVps_; // inaccessible viewpoints for each segment
		
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
		
		
		void replanCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void publishPolygon();
		void publishMap();
		void publishSeg();
		void publishBlockedPoint();
		void publishViewPoints(const std::vector<std::vector<ViewPoint>> &vpSet);
		
		// viewpoint accessability check
		double orient(const Eigen::Vector3d &a,	const Eigen::Vector3d &b, const Eigen::Vector3d &c);
		bool onSegment(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &p);
		int isInPolygon(const Eigen::Vector3d &p);

		// helper function:
		void setVertice(const std::vector<Eigen::Vector3d> & inputVert);
		std::pair<int, int> getSegIdx(const int &targetIdx);
		double getDistance(const int &Seg1Idx, const int & p1Idx, const int &Seg2Idx, const int & p2Idx);
		bool vpHasCollision(const Eigen::Vector3d &viewpoint);
		bool vpHasCollisionRT(const Eigen::Vector3d &viewpoint);

		int indexToAddress(const Eigen::Vector3i& idx);
		void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx);
		void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos);
		void getVoxelCentroid(const Eigen::Vector3d &pos, Eigen::Vector3d &centroid);
    	bool indexInRange(const Eigen::Vector3i& idx);
		int posToAddress(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3i& idx);
		bool isOccupied(const Eigen::Vector3d& pos);
		bool isOccupied(const Eigen::Vector3i& idx);
		bool isFree(const Eigen::Vector3d& pos);
		bool isFree(const Eigen::Vector3i& idx);
		bool isSurfaceVoxel(const Eigen::Vector3i& idx);
		bool vpHasOcclusionInViewCone(
        const ViewPoint& vp,
        double max_range = 1.5,      // x 米，例如 1.0 / 1.5 / offset_
        double yaw_half_fov = 35/180*M_PI,   // 35deg
        double pitch_half_fov = 21/180*M_PI, // 21deg
        double yaw_step = 0.1,       // 0.1 rad
        double pitch_step = 0.1,     // 0.1 rad
        double ray_step = 0.1        // 0.1 or 0.5 * map resolution
    	) ;

		// user functions
		std::vector<std::vector<Eigen::Vector4d>> getViewpoints();
		bool getInputTraj(nav_msgs::Path &inputPath);
		bool getNewGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &yawTuning, double &yaw);
		bool getNewReplanGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &yawTuning, double &yaw);
		void updateCurrPos(const Eigen::Vector3d &currPos);
		void getInaccessibleView(std::vector<Eigen::Vector2i> &inaccessibleIdx);
		void updateInaccessibleView(const std::vector<Eigen::Vector2i> &inaccessibleIdx, std::vector<Eigen::Vector3d> &inaccessibleGrid);
		double updateViewAngle(const std::vector<std::vector<Eigen::Vector3d>> &hitPoints, const double &yaw);
		double getReward(const Eigen::Vector3d &hitPoint);
		void setReward(const Eigen::Vector3d &hitPoint, const double &value);
	};
	
	// inline int vpPlanner::toLinearIdx(int ix, int iy, int iz) {
    //     return ix * this->occupancy_.height * this->occupancy_.depth + 
	// 		   iy * this->occupancy_.depth + 
	// 		   iz	;
    // }

	inline bool vpPlanner::isOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isOccupied(idx);
	}

	inline bool vpPlanner::isOccupied(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] == true;
	}

	inline bool vpPlanner::isFree(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isFree(idx);
	}

	inline bool vpPlanner::isFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return false;
		}
		int address = this->indexToAddress(idx);
		return (this->occupancy_[address] == false);
	}

	inline int vpPlanner::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

    inline bool vpPlanner::indexInRange(const Eigen::Vector3i& idx){
        if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) < this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) < this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) < this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
    }

	inline void vpPlanner::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) {
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

    inline int vpPlanner::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline void vpPlanner::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline void vpPlanner::getVoxelCentroid(const Eigen::Vector3d &pos, Eigen::Vector3d &centroid) {
		// Convert world position to grid index
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		if (!this->indexInRange(idx)) {
			centroid = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
									   std::numeric_limits<double>::quiet_NaN(),
									   std::numeric_limits<double>::quiet_NaN());
			return;
		}

		centroid(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0);
		centroid(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		centroid(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}


    // Prefer half-open check: [min, max)
    inline bool vpPlanner::isInMap(const Eigen::Vector3d& pos){
		if ((pos(0) >= this->mapSizeMin_(0)) and (pos(0) <= this->mapSizeMax_(0)) and 
			(pos(1) >= this->mapSizeMin_(1)) and (pos(1) <= this->mapSizeMax_(1)) and 
			(pos(2) >= this->mapSizeMin_(2)) and (pos(2) <= this->mapSizeMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool vpPlanner::isInMap(const Eigen::Vector3i& idx){
		if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) < this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) < this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) < this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool vpPlanner::isSurfaceVoxel(const Eigen::Vector3i& idx) {
		if (!this->indexInRange(idx)){
			return false;
		} 

		// 用 index 版本判断是否 occupied（比直接用 pos 更稳妥）
		if (!this->isOccupied(idx)){
			return false;
		}

		// 6-连通邻居
		const Eigen::Vector3i neighborOffsets[6] = {
			Eigen::Vector3i( 1, 0, 0),
			Eigen::Vector3i(-1, 0, 0),
			Eigen::Vector3i(0,  1, 0),
			Eigen::Vector3i(0, -1, 0),
			Eigen::Vector3i(0,  0, 1),
			Eigen::Vector3i(0,  0,-1)
		};

		for (int i = 0; i < 6; ++i) {
			Eigen::Vector3i nidx = idx + neighborOffsets[i];

			if (!this->indexInRange(nidx)) {
				continue; 
			}

			if (this->isFree(nidx)) {
				return true;
			}
		}
		return false;
	}



	// cross product (b - a) x (c - a) using only x,y components
	inline double vpPlanner::orient(const Eigen::Vector3d &a,
								const Eigen::Vector3d &b,
								const Eigen::Vector3d &c) {
		double bax = b.x() - a.x();
		double bay = b.y() - a.y();
		double cax = c.x() - a.x();
		double cay = c.y() - a.y();
		return bax * cay - bay * cax;
	}

	// check if p lies on segment [a,b] (in 2D, with tolerance)
	bool vpPlanner::onSegment(const Eigen::Vector3d &a,
					const Eigen::Vector3d &b,
					const Eigen::Vector3d &p) {
		double o = orient(a, b, p);
		double eps = 1e-12;
		if (std::fabs(o) > eps) return false;
		double minx = std::min(a.x(), b.x()) - eps;
		double maxx = std::max(a.x(), b.x()) + eps;
		double miny = std::min(a.y(), b.y()) - eps;
		double maxy = std::max(a.y(), b.y()) + eps;
		return (p.x() >= minx && p.x() <= maxx && p(1) >= miny && p(1) <= maxy);
	}

	// Ray casting: returns 1 inside, 0 boundary, -1 outside
	int vpPlanner::isInPolygon(const Eigen::Vector3d &p) {
		std::cout << "check if it's in polygon\n";
		int n = static_cast<int>(this->poly_.size());
		if (n < 3) return -1;

		// 1) Boundary check (segment)
		for (int i = 0; i < n; ++i) {
			const Eigen::Vector3d &a = this->poly_[i];
			const Eigen::Vector3d &b = this->poly_[(i + 1) % n];
			if (onSegment(a, b, p)) return 0;
		}

		// 2) Ray-casting (cast to +X)
		bool inside = false;
		for (int i = 0, j = n - 1; i < n; j = i++) {
			const Eigen::Vector3d &pi = this->poly_[i];
			const Eigen::Vector3d &pj = this->poly_[j];

			// DEBUG (uncomment if needed)
			// std::cout << "edge: " << pj.transpose() << " -> " << pi.transpose() << "\n";

			// use y() for vertical coordinate
			bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y()));
			if (intersect) {
				// safe because pi.y() != pj.y() when intersect == true
				double xint = pj.x() + (pi.x() - pj.x()) * (p.y() - pj.y()) / (pi.y() - pj.y());
				if (xint > p.x()) {
					inside = !inside;
				}
			}
		}

		return inside ? 1 : -1;
	}
}

#endif