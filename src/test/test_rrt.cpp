#include <ros/ros.h>
#include <global_planner/rrtOctomap.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	ros::NodeHandle nh;
	cout << "test for rrt base class~" << endl;
	const int N = 3;
	KDTree::Point<N> start_point; start_point[0] = 1.1; start_point[1] = 0; start_point[2] = -1;
	KDTree::Point<N> goal_point; goal_point[0] = 2.1; goal_point[1] = 1.123; goal_point[2] = 1000.05;
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_;
	parent_[goal_point] = start_point;

	cout << start_point + goal_point << endl;
	cout << start_point - goal_point << endl;
	cout << 5 * start_point << endl;
	cout << parent_[goal_point] << endl;

	// Test 1: initialize object in two ways and get the private class values
	// rrt::rrtBase<N> r (); // default constructor

	std::vector<double> start, goal, collisionBox, envBox;
	double delQ, dR;
	nh.getParam("/start_position", start);
	nh.getParam("/goal_position", goal);
	nh.getParam("/collision_box", collisionBox);
	nh.getParam("/env_box", envBox);
	nh.getParam("/rrt_incremental_distance", delQ);
	nh.getParam("/goal_reach_distance", dR);
	
	// rrt::rrtBase<N> rrt_planner (start, goal, collisionBox, envBox, delQ, dR);
	rrt::rrtOctomap<N> rrt_planner (start, goal, collisionBox, envBox, delQ, dR);
	

	return 0;
}