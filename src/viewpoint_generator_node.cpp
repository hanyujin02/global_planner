#include <ros/ros.h>
#include <global_planner/viewpointGenerator.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "viewpoint_generator_node");
	ros::NodeHandle nh;

	globalPlanner::vpPlanner vp(nh);

	ros::spin();

	return 0;
}