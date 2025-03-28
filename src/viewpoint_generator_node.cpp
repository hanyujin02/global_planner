#include <ros/ros.h>
#include <global_planner/viewpointGenerator.h>
#include <thread>

ros::Publisher trajPub;
std::thread visWorker;
std::vector<std::vector<Eigen::Vector4d>> vps;

void visualizeTraj(){
	
	if (!vps.empty()) {
		visualization_msgs::Marker line;
		// visualization_msgs::MarkerArray lines;

		// Marker common properties
		line.header.frame_id = "map";
		line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
		line.ns = "view_lines";
		line.scale.x = 0.2;
		line.scale.y = 0.2;
		line.scale.z = 0.2;
		line.color.r = 1.0;
		line.color.g = 0.0;
		line.color.b = 0.0;
		line.color.a = 1.0;
		// line.lifetime = ros::Duration(0.2);
		line.id = 0;

		int id = 0; // Unique marker ID
		// int vpIdx = 0;
		ros::Rate r(5);

		while(ros::ok() and id<int(vps.size())){
			// std::cout << "Press Enter to continue..." << std::endl;
			// std::cin.get();

		// for (size_t i = 0; i < vps.size(); ++i) {
			// for (size_t j = 0; j < vps.size(); ++j) {
				geometry_msgs::Point p1, p2;
				// Re-initialize and update position for each line
				p1.x = vps[id].front()(0);
				p1.y = vps[id].front()(1);
				p1.z = vps[id].front()(2);
				// cout<<"start: "<<p1.x<<", "<<p1.y<<", "<<p1.z<<endl;
				
				p2.x = vps[id].back()(0);
				p2.y = vps[id].back()(1);
				p2.z = vps[id].back()(2);
				// cout<<"end: "<<p2.x<<", "<<p2.y<<", "<<p2.z<<endl;
				
				line.points.push_back(p1);
				line.points.push_back(p2);
				// cout<<vps[id].front()<<endl;
				id++;
				// // Publish the MarkerArray
				trajPub.publish(line);
				r.sleep();
				// cout<<"publish"<<endl;
			// }
		}

		// Publish the MarkerArray
		trajPub.publish(line);
	}

}
int main(int argc, char** argv){
	ros::init(argc, argv, "viewpoint_generator_node");
	ros::NodeHandle nh;

	trajPub = nh.advertise<visualization_msgs::Marker>("/inspection_trajectory", 1000);
	
	globalPlanner::vpPlanner vp(nh);
	vp.makePlan();
	vps = vp.getViewpoints();
	visWorker = std::thread(&visualizeTraj);
	// TODO publish the sequence;
	// visualizeTraj(vps);
	ros::spin();

	return 0;
}