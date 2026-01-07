#include <ros/ros.h>
#include <global_planner/viewpointGenerator.h>
#include <thread>
#include <global_planner/utils.h>

ros::Publisher trajPub;
std::thread visWorker;
std::vector<std::vector<Eigen::Vector4d>> vps;

ros::Publisher waypointsVisPub;
std::vector<std::vector<double>> waypoints;
bool newWaypoints = true;
bool newMsg = false;
std::vector<double> newPoint {0, 0, 1.0, 0};
void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
	newPoint[0] = cp->pose.position.x;
	newPoint[1] = cp->pose.position.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newPoint[3] = globalPlanner::rpy_from_quaternion(cp->pose.orientation);
	newMsg = true;
}

visualization_msgs::MarkerArray waypoints2vis(const std::vector<std::vector<double>>& waypoints){
	visualization_msgs::MarkerArray wpMsg;
	int id = 0;
	for (int i=0; i<int(waypoints.size()); ++i){
		visualization_msgs::Marker wp;
		wp.header.frame_id = "map";
		wp.header.stamp = ros::Time();
		wp.ns = "waypoints";
		wp.id = id;
		wp.type = visualization_msgs::Marker::SPHERE;
		wp.action = visualization_msgs::Marker::ADD;
		wp.pose.position.x = waypoints[i][0];
		wp.pose.position.y = waypoints[i][1];
		wp.pose.position.z = waypoints[i][2];
		wp.lifetime = ros::Duration(0.5);
		wp.scale.x = 0.4;
		wp.scale.y = 0.4;
		wp.scale.z = 0.4;
		wp.color.a = 0.7;
		wp.color.r = 1.0;
		wp.color.g = 0.5;
		wp.color.b = 1.0;		
		++id;
		wpMsg.markers.push_back(wp);
	}
	return wpMsg;
}

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

		if (waypoints.size() != 0){
			visualization_msgs::MarkerArray wpMsg = waypoints2vis(waypoints);
			waypointsVisPub.publish(wpMsg);
		}
	}

}

void saveVectorsToTxt(std::string &filename,
                      const std::vector<Eigen::Vector3d> &vecs) {
	std::string package_path = ros::package::getPath("global_planner");
	filename = package_path + "/cfg/" + filename;
    std::ofstream out(filename);
    if (!out) throw std::runtime_error("Cannot open file for writing: " + filename);
	out<<"["<<"\n";
    for (int i=0;i<int(vecs.size());i++) {
		Eigen::Vector3d v = vecs[i];
		if (i < int(vecs.size()) - 1){
        	out << v.x() << "," << v.y() << "," << v.z() <<","<<"\n";
		}
		else{
			out << v.x() << "," << v.y() << "," << v.z() <<"]";
		}
    }
}

// Load back
std::vector<Eigen::Vector3d> loadVectorsFromTxt(const std::string &filename) {
    std::ifstream in(filename);
    if (!in) throw std::runtime_error("Cannot open file for reading: " + filename);

    std::vector<Eigen::Vector3d> vecs;
    double x, y, z;
    while (in >> x >> y >> z) {
        vecs.emplace_back(x, y, z);
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "viewpoint_generator_node");
	ros::NodeHandle nh;
	globalPlanner::vpPlanner vp(nh);

	trajPub = nh.advertise<visualization_msgs::Marker>("/inspection_trajectory", 1000);
	// ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	// waypointsVisPub = nh.advertise<visualization_msgs::MarkerArray>("/waypoints", 1000);
	
	// int countLoop = 0;
	// ros::Rate r(10);
	// 	cout << "----------------------------------------------------" << endl;
	// 	cout << "[Test Viewpoint Node]: Request No. " << countLoop+1 << endl;
	// 	cout << "[Test Viewpoint Node]: Wait for waypoints..." << endl;
	// 	int countPoints = 0;
	// 	bool lastPoint = false;
	// 	while (ros::ok() and not lastPoint){
	// 		std::vector<double> currPose;
	// 		while (ros::ok() ){
	// 			if (newMsg){
	// 				if (newWaypoints){
	// 					waypoints.clear();
	// 					newWaypoints = false;
	// 				}
	// 				currPose = newPoint;
	// 				newMsg = false;
	// 				if (currPose[3] != 0){
	// 					if (waypoints.size() > 1){
	// 						lastPoint = true;
	// 						newWaypoints = true;
	// 						cout << "[Test Viewpoint Node]: received last point. Waypoints size: " << waypoints.size() << endl;
	// 						break;
	// 					}
	// 				}
	// 				else{
	// 					cout << "[Test Viewpoint Node]: current point" << " i=" << countPoints << " (" << currPose[0] << " " << currPose[1] << " " << currPose[2] << " " << currPose[3] << ")" << endl;
	// 				}
	// 				waypoints.push_back(currPose);
	// 				++countPoints;

	// 				break;
	// 			}
	// 			ros::spinOnce();
	// 			r.sleep();
	// 		}
	// 	}

	// // waypoint to vert: 
	// std::vector<Eigen::Vector3d> vertice;
	// for (int i=0;i<waypoints.size();i++){
	// 	Eigen::Vector3d v;
	// 	v(0) = waypoints[i][0];
	// 	v(1) = waypoints[i][1];
	// 	v(2) = waypoints[i][2];
	// 	vertice.push_back(v);

	// }
	// // save vertice to file
	// std::string filename = "mill19_floor2.txt";
	// saveVectorsToTxt(filename, vertice);
	
	// vp.setVertice();
	vp.makePlan();
	vps = vp.getViewpoints();
	visWorker = std::thread(&visualizeTraj);
	// TODO publish the sequence;
	// visualizeTraj(vps);
	ros::spin();

	return 0;
}