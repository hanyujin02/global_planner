#include <global_planner/viewpointGenerator.h>

namespace globalPlanner{
    vpPlanner::vpPlanner(const ros::NodeHandle& nh) : nh_(nh){
        this->ns_ = "viewpoint_planner";
        this->hint_ = "[vpPlanner]";
        this->initParam();
        this->registerPub();
        this->registerCallback();
        this->initMap();
        this->segMap();
        // this->generateViewPoint();
    }

    void vpPlanner::initParam(){
        // absolute dir of prebuilt map file (.pcd)
		if (not this->nh_.getParam(this->ns_ + "/map_directory", this->mapDir_)){
			this->mapDir_ = "";
			cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->mapDir_ << endl;
		}

        // inspection offset
		if (not this->nh_.getParam(this->ns_ + "/inspection_offset", this->offset_)){
			this->offset_ = 2.0;
		}
		else{
			cout << this->hint_ << ": inspection offset: " << this->offset_ << endl;
		}
		
        // viewpoint step
        if (not this->nh_.getParam(this->ns_ + "/inspection_step", this->step_)){
			this->step_ = 1.0;
			cout << this->hint_ << ": No inspection step param found. Use 1.0 m" << endl;
		}
		else{
			cout << this->hint_ << ": the inspection step param is found: " << this->step_ << endl;
		}

        // viewpoint height step
        if (not this->nh_.getParam(this->ns_ + "/inspection_Z_step", this->stepZ_)){
			this->stepZ_ = 1.0;
			cout << this->hint_ << ": No inspection Z step param found. Use 1.0 m" << endl;
		}
		else{
			cout << this->hint_ << ": the inspection Z step param is found: " << this->stepZ_ << endl;
		}

        // ground height
        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHgt_)){
			this->groundHgt_ = 1.0;
			cout << this->hint_ << ": No ground height param found. Use 1.0 m" << endl;
		}
		else{
			cout << this->hint_ << ": the ground height param is found: " << this->groundHgt_ << endl;
		}

        // ceiling height
        if (not this->nh_.getParam(this->ns_ + "/ceiling_height", this->ceilingHgt_)){
			this->ceilingHgt_ = 3.0;
			cout << this->hint_ << ": No ceiling height param found. Use 3.0 m" << endl;
		}
		else{
			cout << this->hint_ << ": the ceiling height param is found: " << this->ceilingHgt_ << endl;
		}

        // map resolution
        if (not this->nh_.getParam(this->ns_ + "/map_resolution", this->resolution_)){
			this->resolution_ = 0.1;
			cout << this->hint_ << ": No map resolution param found. Use 0.1 m" << endl;
		}
		else{
			cout << this->hint_ << ": the map resolution param is found: " << this->resolution_ << endl;
		}

        // segmentation param
        if (not this->nh_.getParam(this->ns_ + "/min_cluster_size", this->minClusterSize_)){
			this->minClusterSize_ = 50;
			cout << this->hint_ << ": No minimum cluster size param found. Use 50." << endl;
		}
		else{
			cout << this->hint_ << ": the minimum cluster size resolution param is found: " << this->minClusterSize_ << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/max_cluster_size", this->maxClusterSize_)){
			this->maxClusterSize_ = 5000;
			cout << this->hint_ << ": No minimum cluster size param found. Use 5000." << endl;
		}
		else{
			cout << this->hint_ << ": the minimum cluster size resolution param is found: " << this->maxClusterSize_ << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/curvature_threshold", this->curvThres_)){
            this->curvThres_ = 1.0;
            cout<< this->hint_ << ": No curvature threshold param found. Use 1.0. "<<endl;
        }
        else{
            cout<< this->hint_ <<": the curvature threshold param is found: " << this->curvThres_ << endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/angle_threshold", this->angThres_)){
            this->angThres_ = 1.0;
            cout<< this->hint_ << ": No angle threshold param found. Use 1.0. "<<endl;
        }
        else{
            cout<< this->hint_ <<": the angle threshold param is found: " << this->angThres_ << endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/merge_threshold", this->mergeThres_)){
            this->mergeThres_ = 1;
            cout<< this->hint_ << ": No merge threshold param found. Use 1. "<<endl;
        }
        else{
            cout<< this->hint_ <<": the merge threshold param is found: " << this->mergeThres_ << endl;
        }
    }

    void vpPlanner::registerPub(){
        this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/ref_map", 10);
        this->pointVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/view_points", 10);
        this->segMapVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/seg_map", 10);
        this->normalVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/normal", 10);
        this->blockedVisPub_ = this->nh_.advertise<visualization_msgs::Marker>(this->ns_ + "/blocked", 10);
    }

    void vpPlanner::registerCallback(){
        this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &vpPlanner::visCB, this);
    }

    void vpPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->mapRT_ = map;
	}

    void vpPlanner::initMap(){
        std::string type = this->mapDir_.substr(this->mapDir_.size()-3);
        
        if (type == "pcd"){
            cout << this->hint_ << ": Loading PCD..." << endl;
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->mapDir_, this->refCloud_) == -1) //* load the file
            {
                cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
            }
            else {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(this->refCloud_.makeShared());
                pass.setFilterFieldName("z");  // Filter along the z-axis, for example
                pass.setFilterLimits(this->groundHgt_, this->ceilingHgt_); // Adjust range based on your path
                pass.filter(this->refCloud_);

                cout << this->hint_ << ": Map loaded with " << this->refCloud_.width * this->refCloud_.height << " data points. " << endl;			
            }
        }
        else if (type == "STL"){
            // TODO: Convert STL to PCD
            cout << this->hint_ << ": Loading STL..." << endl;
            pcl::PolygonMesh mesh;
            if (pcl::io::loadPolygonFileSTL(this->mapDir_, mesh) == 0) {
                cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
            }
            else{
                pcl::fromPCLPointCloud2(mesh.cloud, this->refCloud_);
                cout << this->hint_ << ": Map loaded with " << this->refCloud_.width * this->refCloud_.height << " data points. " << endl;			
            }
        }

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(this->refCloud_, minPoint, maxPoint);
        this->mapMin_ = {minPoint.x, minPoint.y, minPoint.z};
        this->mapMax_ = {maxPoint.x, maxPoint.y, maxPoint.z};
        this->genOccMap();
	}

    void vpPlanner::genOccMap(){
        // Define grid dimensions (based on point cloud bounds and resolution)
        this->occupancy_.width = ceil((this->mapMax_(0) - this->mapMin_(0)) / this->resolution_) + 1;
        this->occupancy_.height = ceil((this->mapMax_(1) - this->mapMin_(1)) / this->resolution_) + 1;
        this->occupancy_.depth = ceil((this->mapMax_(2) - this->mapMin_(2)) / this->resolution_) + 1;

        this->occupancy_.occ.resize(this->occupancy_.width * this->occupancy_.height * this->occupancy_.depth, false); 
        this->occupancy_.reward.resize(this->occupancy_.width * this->occupancy_.height * this->occupancy_.depth, 0.0); 
        
        // Fill the occupancy grid with the point cloud data
        for (const auto& point : this->refCloud_.points) {
            int idx = this->getIdx(point.x,point.y,point.z);
            this->occupancy_.occ[idx] = true;
            if (this->isSurfaceVoxel(point.x,point.y,point.z)){
                this->occupancy_.reward[idx] = 0.5;
            }
        }
    }

    void vpPlanner::segMap(){
        std::vector<globalPlanner::ClusterInfo> clusterInfos;  // Vector to store cluster info

        // 1. Estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(this->refCloud_.makeShared());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree1);
        ne.setRadiusSearch(0.5);  // Set radius for normal estimation
        ne.compute(*normals);

        // 2. Perform region growing segmentation
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(this->minClusterSize_);
        reg.setMaxClusterSize(this->maxClusterSize_);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
        reg.setSearchMethod(tree2);
        reg.setNumberOfNeighbours(100);
        reg.setInputCloud(this->refCloud_.makeShared());
        reg.setInputNormals(normals);
        // TODO: Smoothness Threshold in Param
        reg.setSmoothnessThreshold(this->angThres_ / 180.0 * M_PI);  // 3 degrees
        reg.setCurvatureThreshold(this->curvThres_);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // 3. Process each cluster to compute the vertices and average normal
        for (int i=0;i<int(clusters.size());i++) {
            pcl::PointCloud<pcl::PointXYZ> cluster;
            Eigen::Vector3d avgNormal{0.0, 0.0, 0.0};
            for (int j=0;j<int(clusters[i].indices.size());j++) {
                cluster.points.push_back(this->refCloud_.points[clusters[i].indices[j]]);
                pcl::Normal normal = normals->points[clusters[i].indices[j]];
                Eigen::Vector3d n{normal.normal_x, normal.normal_y, normal.normal_z};
                if (not std::isnan(n.norm())){
                    avgNormal = avgNormal + n;
                }
            }

            pcl::PointXYZ minPoint, maxPoint;
            avgNormal /= int(clusters[i].indices.size());
            avgNormal(2) = 0.0;
            pcl::getMinMax3D(cluster, minPoint, maxPoint);

            // Store the cluster info
            ClusterInfo info = this->genClusterInfo(avgNormal,cluster);
            clusterInfos.push_back(info);
        }
        this->segMap_ = clusterInfos;
    }

    ClusterInfo vpPlanner::genClusterInfo(const Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ> &cluster){
        //Compute the rotation matrix to align with the desired axis
        Eigen::Vector3d targetAxis(0, 1, 0); // Aligning with the Y-axis
        Eigen::Vector3d rotationAxis = normal.cross(targetAxis);
        double angle = std::acos(normal.dot(targetAxis) / (normal.norm() * targetAxis.norm()));

        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(angle, rotationAxis.normalized()).toRotationMatrix();

        // Apply the rotation to the entire point cloud
        for (auto& point : cluster.points) {
            Eigen::Vector3d pointVec(point.x, point.y, point.z);
            pointVec = rotationMatrix * pointVec; // Rotate the point
            point.x = pointVec(0);
            point.y = pointVec(1);
            point.z = pointVec(2);
        }

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(cluster, minPoint, maxPoint);

        double xinflate, yinflate, zinflate;
        xinflate = 0.1;
        yinflate = this->offset_;
        zinflate = 0.0;
        // Compute the 8 vertices
        std::vector<Eigen::Vector3d> vertices(8);
        vertices[0] = Eigen::Vector3d(minPoint.x-xinflate, minPoint.y-yinflate, minPoint.z-zinflate);  // Vertex 0
        vertices[1] = Eigen::Vector3d(minPoint.x-xinflate, maxPoint.y+yinflate, minPoint.z-zinflate);  // Vertex 1
        vertices[2] = Eigen::Vector3d(maxPoint.x+xinflate, maxPoint.y+yinflate, minPoint.z-zinflate);  // Vertex 2
        vertices[3] = Eigen::Vector3d(maxPoint.x+xinflate, minPoint.y-yinflate, minPoint.z-zinflate);  // Vertex 3
        vertices[4] = Eigen::Vector3d(minPoint.x-xinflate, minPoint.y-yinflate, maxPoint.z+zinflate);  // Vertex 4
        vertices[5] = Eigen::Vector3d(minPoint.x-xinflate, maxPoint.y+yinflate, maxPoint.z+zinflate);  // Vertex 5
        vertices[6] = Eigen::Vector3d(maxPoint.x+xinflate, maxPoint.y+yinflate, maxPoint.z+zinflate);  // Vertex 6
        vertices[7] = Eigen::Vector3d(maxPoint.x+xinflate, minPoint.y-yinflate, maxPoint.z+zinflate);  // Vertex 7

        Eigen::Vector3d centroid;
        centroid(0) = minPoint.x + (maxPoint.x - minPoint.x) / 2;
        centroid(1) = minPoint.y + (maxPoint.y - minPoint.y) / 2;
        centroid(2) = minPoint.z + (maxPoint.z - minPoint.z) / 2;

        Eigen::Matrix3d rotationMatrix2 = Eigen::AngleAxisd(-angle, rotationAxis.normalized()).toRotationMatrix();

        for (int i=0;i<8;i++){
            Eigen::Vector3d point = vertices[i];
            point = rotationMatrix2*point;
            vertices[i] = point;
        }
        centroid = rotationMatrix2*centroid;

        // Store the cluster info
        ClusterInfo info;
        info.vert = vertices;
        info.normal = normal;
        info.centroid = centroid;
        return info;

    }

    void vpPlanner::makePlan(){
        // TODO: ignore viewpoints outside selected polygons
        std::vector<std::vector<ViewPoint>> vpSet;
        for (const ClusterInfo &cluster: this->segMap_){
            std::vector<ViewPoint> vps;
            Eigen::Vector3d n = cluster.normal;
            Eigen::Vector3d centroid = cluster.centroid;
            std::vector<Eigen::Vector3d> vertex = cluster.vert;
            
            int vert_idx[4][2] = {
                {0,1},
                {1,2},
                {2,3},
                {3,0}
            };
            double height = vertex[4](2)-vertex[0](2);
            
            for(int i=0;i<4;i++){
                Eigen::Vector3d start = vertex[vert_idx[i][0]];
                Eigen::Vector3d end = vertex[vert_idx[i][1]];
                Eigen::Vector3d mid = start + (end-start)/2;
                
                Eigen::Vector3d direction = end-start;
                // check orientation to normal
                double cosTheta = (direction.dot(n))/(direction.norm()*n.norm());
                cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
                double angle = std::acos(cosTheta);
                
                if (std::abs(angle-M_PI/2)<=0.1){
                    Eigen::Vector3d angleVec = centroid-mid;
                    double viewAngle = atan2(angleVec(1),angleVec(0));
                    cout<<"origin view angle: "<<viewAngle<<endl;

                    for (double j=0.1;j<height;j+=this->stepZ_){
                        Eigen::Vector3d p = start;
                        p(2) += j;
                        std::vector<ViewPoint> vps;
                        for(double dist=0.1;dist<direction.norm();dist+=this->step_){                                                    
                            Eigen::Vector3d point = p;
                            point = point + dist*direction/direction.norm();
                            if (this->isInMap(point(0), point(1), point(2))){
                                if (not this->vpHasCollision(point)){   
                                    ViewPoint vp;
                                    vp.pose = point;
                                    vp.yaw = viewAngle;
                                    vps.push_back(vp);
                                }
                            }
                        }
                        if (vps.size()>0){
                            vpSet.push_back(vps);   
                        }
                    }
                }
            }
        }
        this->vpCluster_ = vpSet;
        this->vpSet_ = this->solveSequence();
        this->initViewpoints();
    }

    void vpPlanner::initViewpoints(){
		// this->vpSet_ = this->vpPlanner_->getViewpoints();
		// Init Viewpoints Indices
		this->vpIdx_.clear();
		for (int i=0;i<int(this->vpSet_.size());i++){
			Eigen::Vector2i idx1, idx2;
			idx1<<i, 0;
			idx2<<i, int(this->vpSet_[i].size())-1;
			this->vpIdx_.push_back(idx1);
			this->vpIdx_.push_back(idx2);
		}
		// // Init First Goal
		// this->goal_.pose.position.x = 0.0;
		// this->goal_.pose.position.y = 0.0;
		// this->goal_.pose.position.z = 1.0;
		// // this->needGlobalPlan_=false;
		// startPos_<<0.0, 0.0, 1.0;
	}

    //Lin-Kernighan heuristic symetric TSP
    std::vector<std::vector<ViewPoint>> vpPlanner::solveSequence(){
        // std::vector<ViewPoint> vpSeqTemp;

        ros::Time start = ros::Time::now();
        cout<<"start"<<endl;
        std::vector<int> nodeIdx;
        std::string package_path = ros::package::getPath("global_planner");
        cout<<package_path<<endl;
        std::string filepath = package_path + "/lkh/nodes.tsp";
        ofstream outFile(filepath);
        if (!outFile.is_open()) {
            cerr << "Error opening file for writing: " << filepath << endl;
            // return vpSeqTemp;
        }

        int size = 1; // start with current position
        for (int i=0;i<int(this->vpCluster_.size());i++){
            for (int j=0;j<int(this->vpCluster_[i].size());j++){
                size++;
            }
        }

        // Write the header
        outFile << "NAME : GeneratedFile\n";
        outFile << "COMMENT : Example file generated by program\n";
        outFile << "TYPE : TSP\n";
        outFile << "DIMENSION : " << size << "\n";
        outFile << "EDGE_WEIGHT_TYPE : EUC_3D\n";
        outFile << "NODE_COORD_SECTION\n";

        int idx = 1;// start with current position
        outFile<< idx << " " << this->currPos_(0) << " " << this->currPos_(1) << " " << this->currPos_(2) << endl;
        idx++;
        for (int i=0;i<int(this->vpCluster_.size());i++){
            for (int j=0;j<int(this->vpCluster_[i].size());j++){
                outFile << idx << " " <<this->vpCluster_[i][j].pose(0) << " " << this->vpCluster_[i][j].pose(1) << " " << this->vpCluster_[i][j].pose(2) << endl;
                idx++;
            }
        }

        outFile.close();
        cout << "Data saved to " << filepath << endl;

        // Command with arguments
        std::string command = package_path + "/lkh/LKH" + " " + package_path + "/lkh/nodes.par";

        int ret_code = system(command.c_str());
        if (ret_code != 0) {
            ROS_ERROR("Executable failed with return code %d", ret_code);
        } else {
            ROS_INFO("Executable ran successfully.");
        }

        std::string solvedfilepath = package_path + "/lkh/nodes_solved.tsp";

        ifstream inFile(solvedfilepath);
        if (!inFile.is_open()) {
            cerr << "Error opening file: " << solvedfilepath << endl;
            // return vpSeqTemp;
            // return;
        }

        std::string line;
        bool readingNodes = false;
        while (getline(inFile, line)) {
            if (line == "EOF") break;
            if (line == "TOUR_SECTION") {
                readingNodes = true;
                continue;
                
            }
            if (readingNodes){
                int id;
                std::stringstream ss(line);
                ss >> id;
                nodeIdx.push_back(id-2);
            }
            

            
        }
        inFile.close();
        for (int i=0;i<int(nodeIdx.size());i++){
        }
        nodeIdx.erase(nodeIdx.begin());
        nodeIdx.pop_back();
        ros::Time end = ros::Time::now();

        std::vector<std::vector<ViewPoint>> vpSet = this->rearrangeVP(nodeIdx);
        return vpSet;

    }

    std::vector<std::vector<ViewPoint>> vpPlanner::rearrangeVP(const std::vector<int> &vpSeq){
        int prevSeg = -1;
        std::vector<ViewPoint> seg;
        std::vector<std::vector<ViewPoint>> vpSet;
        std::vector<std::array<int, 3>> vpSetInfo;
        int startIdx = -1;
        int endIdx = -1;

        for(int i=0;i<int(vpSeq.size());i++){
            int targetIdx = vpSeq[i];
            // find segment
            std::pair<int, int> segIdx = this->getSegIdx(targetIdx);
            int currSeg = segIdx.first;
            int idx = segIdx.second;
            std::array<int, 3> info;

            if (currSeg != prevSeg){
                if (int(seg.size())>0){
                    vpSet.push_back(seg);
                    if (endIdx <0){
                        endIdx = startIdx;
                    }
                    std::array<int, 3> info = {prevSeg,startIdx,endIdx};
                    endIdx = -1;
                    startIdx = -1;
                    vpSetInfo.push_back(info);
                }
                seg.clear();
                seg.push_back(this->vpCluster_[currSeg][idx]);
                startIdx = idx;
            }
            else{
                seg.push_back(this->vpCluster_[currSeg][idx]);
                endIdx = idx;
                if (i==int(vpSeq.size()-1)){
                    vpSet.push_back(seg);
                    std::array<int, 3> info = {prevSeg,startIdx,endIdx};
                    endIdx = -1;
                    startIdx = -1;
                    vpSetInfo.push_back(info);
                }
            }
            prevSeg = currSeg;
        }
        this->vpSetRaw_ = vpSet;

        // Post Process
        // filter:
        for (int threshold=1;threshold<=this->mergeThres_;threshold++){
            for (int i=0;i<int(vpSetInfo.size());i++){
                std::array<int, 3> info = vpSetInfo[i];
                if (info[0]>0){
                    if (std::abs(info[1]-info[2])<threshold){
                        // find closest
                        int setIdx = -1;
                        int minDist = INFINITY;
                        for (int j=0;j<int(vpSetInfo.size());j++){
                            if (vpSetInfo[j][0] == info[0] and j!=i){
                                if (abs(j-i)<minDist){
                                    minDist = abs(j-i);
                                    setIdx = j;
                                }
                            }
                        }
                        // cout<<"original: "<<"seg: "<<info[0]<<", start"<<info[1]<<", end: "<<info[2]<<endl;
                        if (setIdx >= 0){
                            // cout<<"target: "<<"seg: "<<vpSetInfo[setIdx][0]<<", start"<<vpSetInfo[setIdx][1]<<", end: "<<vpSetInfo[setIdx][2]<<endl;
                            std::pair<int,std::pair<int, int>> newInfo = this->merge(info,vpSetInfo[setIdx]);
                            if (newInfo.first == true){
                                vpSetInfo[i] = {-1, -1, -1};
                                vpSetInfo[setIdx][1] = newInfo.second.first;
                                vpSetInfo[setIdx][2] = newInfo.second.second;
                                // cout<<"new start: "<<vpSetInfo[setIdx][1]<<", new end: "<<vpSetInfo[setIdx][2]<<endl;
                            }
                        }
                    }
                }   
            }
        }
        
        // delete
        std::vector<std::array<int, 3>> vpSetInfoFiltered;
        for(int i=0;i<int(vpSetInfo.size());i++){
            std::array<int, 3> info = vpSetInfo[i];
            if (info[0]>=0) {
                vpSetInfoFiltered.push_back(info);
            }
        }
        // swap
        // for (int i=1;i<int(vpSetInfoFiltered.size());i++){
        //     std::array<int, 3> info1 = vpSetInfoFiltered[i];
        //     std::array<int, 3> info2 = vpSetInfoFiltered[i-1];
        //     double totalDist = this->getDistance(info1[0], info1[2], info1[0], info1[1]);
        //     double currDist = this->getDistance(info1[0], info1[1], info2[0], info2[2]);
        //     double swapDist = this->getDistance(info1[0], info1[2], info2[0], info2[2]);
        //     double threshold = 3.0;
        //     if (totalDist<threshold and currDist>swapDist){
        //         // cout<<"num segment: "<<i<<"swap"<<endl;
        //         vpSetInfoFiltered[i][1] = info1[2];
        //         vpSetInfoFiltered[i][2] = info1[1];
        //     }
        // }

        // reassign vp
        std::vector<std::vector<ViewPoint>> vpSetArranged;
        for (int i=0;i<int(vpSetInfoFiltered.size());i++){
            std::array<int, 3> info = vpSetInfoFiltered[i];
            std::vector<ViewPoint> segArranged;
            // cout<<"segIDX: "<<info[0]<<" start: "<<info[1]<<"end: "<<info[2]<<endl; 
            // cout<<"start: "<<this->vpCluster_[info[0]][info[1]].pose<<endl;
            // cout<<"end: "<<this->vpCluster_[info[0]][info[2]].pose<<endl;
            if (info[1] > info[2]){
                for (int j = info[1];j>=info[2];j--){
                    segArranged.push_back(this->vpCluster_[info[0]][j]);
                }
            }
            else{
                for (int j = info[1];j<=info[2];j++){
                    segArranged.push_back(this->vpCluster_[info[0]][j]);
                }
            }
            vpSetArranged.push_back(segArranged);
        }
        
        return vpSetArranged;
    }

    std::pair<bool, std::pair<int, int>> vpPlanner::merge(const std::array<int, 3>& original, const std::array<int, 3>& target) {
        int orig_start = std::min(original[1], original[2]);
        int orig_end = std::max(original[1], original[2]);
        int target_start = std::min(target[1], target[2]);
        int target_end = std::max(target[1], target[2]);
    
        // Check for overlap first — merge immediately
        if (orig_end >= target_start && target_end >= orig_start) {
            int merged_start = std::min(orig_start, target_start);
            int merged_end = std::max(orig_end, target_end);
    
            if (target[1] > target[2]) {
                return {true, {merged_end, merged_start}};
            } else {
                return {true, {merged_start, merged_end}};
            }
        }
    
        // Now check for continuity (edge-to-edge touch)
        if (orig_end + 1 == target_start || target_end + 1 == orig_start) {
            // Determine connecting endpoints
            int p1 = (orig_end + 1 == target_start) ? orig_end : orig_start;
            int p2 = (orig_end + 1 == target_start) ? target_start : target_end;
            
            int SegIdx = original[0];
            // --- INSERT YOUR DISTANCE CALCULATION HERE ---
            double distance = this->getDistance(SegIdx, p1, SegIdx, p2); // placeholder
    
            double threshold = 1.1; // adjust as needed
            if (distance > threshold) {
                return {false, {0, 0}};
            }
    
            int merged_start = std::min(orig_start, target_start);
            int merged_end = std::max(orig_end, target_end);
    
            if (target[1] > target[2]) {
                return {true, {merged_end, merged_start}};
            } else {
                return {true, {merged_start, merged_end}};
            }
        }
    
        // Not overlapping or continuous
        return {false, {0, 0}};
    }    

    void vpPlanner::visCB(const ros::TimerEvent&){
        this->publishMap();
        this->publishSeg();
        this->publishViewPoints(this->vpSetRaw_);
        this->publishBlockedPoint();
    }

    void vpPlanner::publishMap(){
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(this->refCloud_, cloudMsg);
        cloudMsg.header.frame_id = "map";
        cloudMsg.header.stamp = ros::Time::now();
        this->mapVisPub_.publish(cloudMsg);
    }

    void vpPlanner::publishBlockedPoint(){
        visualization_msgs::Marker points;
		points.header.frame_id = "map"; // Set your frame
		points.header.stamp = ros::Time::now();
		points.ns = "raycast";
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.action = visualization_msgs::Marker::ADD;

		// Set the scale of the points (size in RViz)
		points.scale.x = 0.05; // Point width
		points.scale.y = 0.05; // Point height

		// Set the color (RGBA)
		points.color.r = 1.0f;
		points.color.g = 0.0f;
		points.color.b = 0.0f;
		points.color.a = 1.0f; // Fully opaque

		// Add the hitpoints to the marker
		for (int i=0;i<int(this->refCloud_.points.size());i++){
				geometry_msgs::Point p;
				p.x = this->refCloud_.points[i].x;
				p.y = this->refCloud_.points[i].y;
				p.z = this->refCloud_.points[i].z;
                if (this->isSurfaceVoxel(p.x,p.y,p.z)){
                    points.points.push_back(p);
                }
		}
        // cout<<this->refCloud_.points.size()<<" points, "<<points.points.size()<<" surface"<<endl;
		this->blockedVisPub_.publish(points);
    }

    void vpPlanner::publishSeg(){
        if (this->segMap_.size()>0){
		    visualization_msgs::Marker line;
		    visualization_msgs::MarkerArray lines;
		    line.header.frame_id = "map";
		    line.type = visualization_msgs::Marker::LINE_LIST;
		    line.action = visualization_msgs::Marker::ADD;
		    line.ns = "segmented_box";  
		    line.scale.x = 0.06;
		    line.color.r = 0;
		    line.color.g = 1;
		    line.color.b = 1;
		    line.color.a = 1.0;
		    line.lifetime = ros::Duration(0.2);

            visualization_msgs::MarkerArray arrows;
            
            Eigen::Vector3d vertex_pose;
		    for(int i=0; i<int(this->segMap_.size()); ++i){
		        ClusterInfo v = this->segMap_[i];
		        std::vector<geometry_msgs::Point> verts;
		        geometry_msgs::Point p;

				for (int j=0; j<int(v.vert.size());++j){
					p.x = v.vert[j](0); p.y = v.vert[j](1); p.z = v.vert[j](2);
		        	verts.push_back(p);
				}

		        int vert_idx[12][2] = {
		            {0,1},
		            {1,2},
		            {2,3},
		            {0,3},
		            {0,4},
		            {1,5},
		            {3,7},
		            {2,6},
		            {4,5},
		            {5,6},
		            {4,7},
		            {6,7}
		        };
		        for (int j=0;j<12;++j){
		            line.points.push_back(verts[vert_idx[j][0]]);
		            line.points.push_back(verts[vert_idx[j][1]]);
		        }
		        lines.markers.push_back(line);
		        line.id++;

                visualization_msgs::Marker arrow;
                arrow.header.frame_id = "map";
                arrow.ns = "normals";
                arrow.id = i;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.action = visualization_msgs::Marker::ADD;
                arrow.scale.x = 0.1;
                arrow.scale.y = 0.3;
                arrow.scale.z = 0.3;
                arrow.color.r = 1;
                arrow.color.g = 0;
                arrow.color.b = 0;
                arrow.color.a = 1.0;

                // Set the start point of the arrow (point position)
                Eigen::Vector3d start = (v.vert[0]+v.vert[6])/2;
                geometry_msgs::Point start_point;
                start_point.x = start(0);
                start_point.y = start(1);
                start_point.z = start(2);

                // Set the end point of the arrow (normal direction)
                geometry_msgs::Point end_point;
                end_point.x = start_point.x + v.normal(0); // Scale normal by 0.1
                end_point.y = start_point.y + v.normal(1);
                end_point.z = start_point.z + v.normal(2);

                arrow.points.push_back(start_point);
                arrow.points.push_back(end_point);

                arrows.markers.push_back(arrow);
		    }
		    // publish
            this->segMapVisPub_.publish(lines);
            this->normalVisPub_.publish(arrows);
        }
    }

    void vpPlanner::publishViewPoints(const std::vector<std::vector<ViewPoint>> &vpSet) {
        if (!vpSet.empty()) {
            visualization_msgs::Marker point;
            visualization_msgs::Marker text;
            visualization_msgs::MarkerArray points;

            // Marker common properties
            point.header.frame_id = "map";
            point.type = visualization_msgs::Marker::SPHERE;
            point.action = visualization_msgs::Marker::ADD;
            point.ns = "view_points";
            point.scale.x = 0.6;
            point.scale.y = 0.6;
            point.scale.z = 0.6;
            point.color.r = 0.0;
            point.color.g = 1.0;
            point.color.b = 1.0;
            point.color.a = 1.0;
            point.lifetime = ros::Duration(0.2);

            text.header.frame_id = "map";
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.ns = "sequence";
            text.scale.x = 0.6;
            text.scale.y = 0.6;
            text.scale.z = 0.6;
            text.color.r = 1.0;
            text.color.g = 0.0;
            text.color.b = 0.0;
            text.color.a = 1.0;
            text.lifetime = ros::Duration(0.2);

            int id = 0; // Unique marker ID
            int vpIdx = 0;
            for (size_t i = 0; i < vpSet.size(); ++i) {
                for (size_t j = 0; j < vpSet[i].size(); ++j) {
                    // Re-initialize and update position for each point
                    point.pose.position.x = vpSet[i][j].pose(0);
                    point.pose.position.y = vpSet[i][j].pose(1);
                    point.pose.position.z = vpSet[i][j].pose(2);
                    
                    text.pose.position.x = vpSet[i][j].pose(0);
                    text.pose.position.y = vpSet[i][j].pose(1);
                    text.pose.position.z = vpSet[i][j].pose(2)+0.5;
                    
                    text.text = std::to_string(vpIdx);
                    point.id = id++; // Assign unique ID for each marker
                    text.id = id++;
                    // Add the marker to the array
                    points.markers.push_back(point);
                    points.markers.push_back(text);
                    vpIdx++;
                }
            }

            // Publish the MarkerArray
            this->pointVisPub_.publish(points);
        }
    }

    // helper function
    std::pair<int, int> vpPlanner::getSegIdx(const int &targetIdx){
        // find segment
        int idx = -1;
        int currSeg = -1;
        for (int i=0;i<int(this->vpCluster_.size());i++){
            if (idx<targetIdx and idx+int(this->vpCluster_[i].size())>=targetIdx){
                currSeg = i;
                break;
            }
            idx += int(this->vpCluster_[i].size());
        }
        std::pair<int, int> segIdx;
        segIdx.first = currSeg;
        segIdx.second = targetIdx-idx-1;
        return segIdx;
    }    
    
    double vpPlanner::getDistance(const int &Seg1Idx, const int & p1Idx, const int &Seg2Idx, const int & p2Idx){
        ViewPoint vp1 = this->vpCluster_[Seg1Idx][p1Idx];
        ViewPoint vp2 = this->vpCluster_[Seg2Idx][p2Idx];
        double dist = (vp1.pose-vp2.pose).norm();
        return dist;
    }

    bool vpPlanner::vpHasCollision(const Eigen::Vector3d &viewpoint){
        Eigen::Vector3d p;
        double r = 0.5;
        for (double i=-r; i<=r;i+=0.1){
            for(double j=-r;j<=r;j+=0.1){
                // for (double k = -r; k<=r; k+=0.1){
                    p(0) = viewpoint(0)+i;
                    p(1) = viewpoint(1)+j;
                    p(2) = viewpoint(2);
                    if (this->isInMap(p(0), p(1), p(2)) and this->isOccupied(p(0), p(1), p(2))){
                        return true;
                    }
                    else if (not this->isInMap(p(0), p(1), p(2))){
                        return true;
                    }
                // }
            }
        }
        return false;
    }

    bool vpPlanner::vpHasCollisionRT(const Eigen::Vector3d &viewpoint){
        Eigen::Vector3d p;
        double r = 0.5;
        for (double i=-r; i<=r;i+=0.1){
            for(double j=-r;j<=r;j+=0.1){
                // for (double k = -r; k<=r; k+=0.1){
                    p(0) = viewpoint(0)+i;
                    p(1) = viewpoint(1)+j;
                    p(2) = viewpoint(2);
                    if (this->mapRT_->isOccupied(p)){
                        return true;
                    }
                    else{
                        return true;
                    }
                // }
            }
        }
        return false;
    }    
        
    // user function
    std::vector<std::vector<Eigen::Vector4d>> vpPlanner::getViewpoints(){
        std::vector<std::vector<Eigen::Vector4d>> viewpoints;
        viewpoints.resize(this->vpSet_.size());
        for(int i=0;i<int(this->vpSet_.size());i++){
            viewpoints[i].resize(this->vpSet_[i].size());
            for (int j=0;j<int(this->vpSet_[i].size());j++){
                Eigen::Vector4d vp;
                vp<<this->vpSet_[i][j].pose(0), this->vpSet_[i][j].pose(1), this->vpSet_[i][j].pose(2), this->vpSet_[i][j].yaw;
                viewpoints[i][j] = vp;
            }

        }
        return viewpoints;
    }

    bool vpPlanner::getNewGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &noYawTurning, double &yaw){
        Eigen::Vector3d vp;
        this->goalIdx_++;
            if (this->goalIdx_ >= int(this->vpIdx_.size()))
            {
                goal.pose.position.x = 0;
                goal.pose.position.y = 0;
                goal.pose.position.z = 1.0;
                noYawTurning = false;
                needGlobalPlan = true;
                return true;
            }
            
            int segIdx = this->vpIdx_[this->goalIdx_](0);
            int vpIdx = this->vpIdx_[this->goalIdx_](1);
            vp = this->vpSet_[segIdx][vpIdx].pose;

            goal.pose.position.x = vp(0);
            goal.pose.position.y = vp(1);
            goal.pose.position.z = vp(2);
            
            
            if (this->goalIdx_%2){// facing view angle
                needGlobalPlan = false;
                noYawTurning = true;
                double currentYaw;
                currentYaw = this->vpSet_[segIdx][vpIdx].yaw;
                yaw = currentYaw;
                cout<<"output yaw: "<<yaw<<endl;
            }
            else{// facing next goal when navigating from one segment to another
                needGlobalPlan = true;
                noYawTurning = false;
            }
            
            return true;
    }

    // TODO: check viewpoint collision
    bool vpPlanner::getNewReplanGoal(geometry_msgs::PoseStamped &goal, bool &needGlobalPlan, bool &noYawTurning, double &yaw){

        cout<<"looking for new goal"<<endl;
        Eigen::Vector3d vp;
        int segIdx = this->vpIdx_[this->goalIdx_](0);
        int vpIdx = this->vpIdx_[this->goalIdx_](1);
        int newVPIdx = -1;
        vp = this->vpSet_[segIdx][vpIdx].pose;
        bool replanSuccess = false;
        if (this->goalIdx_%2){
            int lastGoalIdx = this->goalIdx_-1;
            int lastvpIdx = this->vpIdx_[lastGoalIdx](1);
            for(int i=vpIdx;i>lastvpIdx; i--){
                vp = this->vpSet_[segIdx][i].pose;
                if (not this->vpHasCollisionRT(vp)){
                    replanSuccess = true;
                    newVPIdx = i;
                    break;
                }
            }
            // if not replanSuccess, go to next segment
            if (not replanSuccess){
                // this->goalIdx_+=1;
                return false;
            }
        }
        else{
            int nextGoalIdx = this->goalIdx_+1;
            int nextvpIdx = this->vpIdx_[nextGoalIdx](1);
            for (int i=vpIdx;i < nextvpIdx; i++){
                vp = this->vpSet_[segIdx][i].pose;
                if (not this->vpHasCollisionRT(vp)){
                    replanSuccess = true;
                    newVPIdx = i;
                    break;
                }
            }
            // if not replanSuccess, go to next segment
            if (not replanSuccess){
                this->goalIdx_+=1;
                return false;
            }
        }

        this->vpIdx_[this->goalIdx_](1) = newVPIdx;
        vp = this->vpSet_[segIdx][newVPIdx].pose;
        cout<<"new goal: "<<vp;
        goal.pose.position.x = vp(0);
        goal.pose.position.y = vp(1);
        goal.pose.position.z = vp(2);

        if (this->goalIdx_%2){// facing view angle
            noYawTurning = true;
            needGlobalPlan = false;
            double currentYaw;
            currentYaw = this->vpSet_[segIdx][vpIdx].yaw;
            yaw = currentYaw;
        }
        else{// facing next goal when navigating from one segment to another
            needGlobalPlan = true;
            noYawTurning = false;
        }
       
        return true;
    }

    void vpPlanner::updateCurrPos(const Eigen::Vector3d &currPos){
        this->currPos_ = currPos;
    }

    void vpPlanner::getInaccessibleView(std::vector<Eigen::Vector2i> &inaccessibleIdx){
		int segIdx = this->vpIdx_[this->goalIdx_](0);
		for (int i=0;i<int(this->vpSet_[segIdx].size());i++){
			Eigen::Vector3d vp = this->vpSet_[segIdx][i].pose;
			if (this->vpHasCollisionRT(vp)){
				Eigen::Vector2i idx{segIdx,i};
				inaccessibleIdx.push_back(idx);
			}
		}
	}

    void vpPlanner::updateInaccessibleView(const std::vector<Eigen::Vector2i> &inaccessibleIdx){
        for (int i=0; i<int(inaccessibleIdx.size());i++){
            // get blocked viewpoint
            ViewPoint vp = this->vpSet_[inaccessibleIdx[i][0]][inaccessibleIdx[i][1]];
            // get blocked occmap
            double minAngle, maxAngle;
            minAngle = vp.yaw-35/180*M_PI;
            maxAngle = vp.yaw+35/180*M_PI;
            for (double j=minAngle;j<maxAngle;j+=0.1){
                // TODO : n param (height angle)
                for (double n=-21/180*M_PI;n<21/180*M_PI;n+=0.1){
                    Eigen::Vector3d dir{cos(j), sin(j), tan(n)};
                    for (double k=0;k<this->offset_+2.0;k+=0.1){
                        Eigen::Vector3d projPoint = vp.pose + dir*k;
                        if (this->isInMap(projPoint(0), projPoint(1),projPoint(2)) and this->isSurfaceVoxel(projPoint(0), projPoint(1),projPoint(2))){
                            int pointIdx = this->getIdx(projPoint(0), projPoint(1), projPoint(2));
                            // TODO: check reward value
                            this->occupancy_.reward[pointIdx] = 1.0;
                        }  
                    }
                }   
            }
        }        
    }

    double vpPlanner::updateViewAngle(const std::vector<std::vector<Eigen::Vector3d>> &hitPoints, const double &yaw){
        // getviewangle according to currpos, raycast, and currYaw
        double hres = 360/int(hitPoints.size());
        std::vector<double> reward;
        for (int i=0;i<int(hitPoints.size());i++){
            // get reward for each angle
            double r = 0;
            for (int j=0;j<int(hitPoints[i].size());j++){
                Eigen::Vector3d p = hitPoints[i][j];
                // if (this->isInMap(p(0), p(1), p(2)) and this->isOccupied(p(0), p(1), p(2))){
                //     int idx = this->getIdx(p(0), p(1), p(2));
                //     r += this->occupancy_.reward[idx];
                // }
                r += this->getReward(p);
            }          
            reward.push_back(r);
        }
        std::vector<std::pair<double, double>> angleReward;
        for (int i=0;i<int(reward.size());i++){
            double ang = i*hres;
            // TODO: Angle threshold
            // if (std::abs(ang-yaw/M_PI*180)<10){
                double r = 0;
                double minAngle, maxAngle;
                minAngle = i*hres - 69/2;
                maxAngle = i*hres + 69/2;
                for (double angle = minAngle;angle < maxAngle;angle += hres){
                    int idx = int(angle/hres);
                    if (idx < 0){
                        idx = int(reward.size())+idx;
                    }
                    else if (idx >= int(reward.size())){
                        idx = idx - int(reward.size());
                    }
                    r += reward[idx];
                }
                std::pair<double, double> angleR;
                
                angleR.second = i*hres/180*M_PI;
                // weighted score
                double weight = 10*cos(angleR.second-yaw)+10;
                angleR.first = r*weight;
                angleReward.push_back(angleR);
            // }
        }
        if (angleReward.size()>0){
            auto maxElement = std::max_element(angleReward.begin(), angleReward.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.first < b.first;
            });

            double viewAngle;
            
            if (maxElement->first < 0.1){
                viewAngle = yaw;
            }
            else{
                viewAngle = maxElement->second;
            }
            return viewAngle;
        }

        return yaw;
    }

    double vpPlanner::getReward(const Eigen::Vector3d &hitPoint){
        Eigen::Vector3d p;
		double r = 0.2;//radius for goal collision check
        double reward = 0;
		for (double i=-r; i<=r;i+=0.1){
			for(double j=-r;j<=r;j+=0.1){
				for (double k = -r; k<=r; k+=0.1){
					p(0) = hitPoint(0)+i;
					p(1) = hitPoint(1)+j;
					p(2) = hitPoint(2)+k;
					if (this->isInMap(p(0), p(1), p(2))){
                        int idx = this->getIdx(p(0), p(1), p(2));
						reward += this->occupancy_.reward[idx];
					}
                    // else if (not this->isInMap(p(0), p(1), p(2))){
                    //     return true;
                    // }
				}
			}
		}
        return reward;
    }
}