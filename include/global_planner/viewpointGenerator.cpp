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
            this->occupancy_.reward[idx] = 0.5;
        }
    }

    int vpPlanner::getIdx(double xPos, double yPos, double zPos){
        int x = ceil((xPos - this->mapMin_(0)) / this->resolution_);
        int y = ceil((yPos - this->mapMin_(1)) / this->resolution_);
        int z = ceil((zPos - this->mapMin_(2)) / this->resolution_);
        int idx = x + this->occupancy_.width * (y + this->occupancy_.height * z);
        return idx;
    }

    bool vpPlanner::isInMap(double x, double y, double z){
        return ((x <= this->mapMax_(0)) && (x >= this->mapMin_(0)) &&
                (y <= this->mapMax_(1)) && (y >= this->mapMin_(1)) &&
                (z <= this->mapMax_(2)) && (z >= this->mapMin_(2)));
    }

    bool vpPlanner::isOccupied(double x, double y, double z){
        int idx = this->getIdx(x,y,z);
        return this->occupancy_.occ[idx];
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
        xinflate = this->offset_;
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
            
            // for (double j=0.1;j<height;j+=this->step_){
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

                    // for(double dist=0.1;dist<direction.norm();dist+=this->step_){
                    //     Eigen::Vector3d p = start + dist*direction/direction.norm();
                    //     for (double j=0.1;j<height;j+=this->step_){
                    //         Eigen::Vector3d point = p;
                    //         point(2) += j;
                    //         if (this->isInMap(point(0), point(1), point(2))){
                    //             if (not this->vpHasCollision(point)){   
                    //                 ViewPoint vp;
                    //                 vp.pose = point;
                    //                 vp.yaw = viewAngle;
                    //                 vps.push_back(vp);
                    //             }
                    //         }
                    //     }
                    // }
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
        this->vpSet_ = this->solveSequence(vpSet);
    }

    bool vpPlanner::vpHasCollision(const Eigen::Vector3d &viewpoint){
		Eigen::Vector3d p;
		double r = std::max(0.0,this->offset_-0.5);//radius for goal collision check
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

    // std::vector<std::vector<ViewPoint>> vpPlanner::makePlan(const std::vector<std::vector<ViewPoint>> &vpSet){
    //     // TODO: set start point
    //     Eigen::Vector3d start{0.0, 0.0, 0.0};
    //     std::vector<std::vector<ViewPoint>> plannedSeg;
    //     plannedSeg.resize(vpSet.size());
    //     std::vector<std::pair<int, int>> vpIdx;
    //     for (int i=0;i<int(vpSet.size());i++){
    //         std::pair<int, int> idx1, idx2;
    //         idx1.first = i; 
    //         idx1.second = 0;
    //         idx2.first = i;
    //         idx2.second = int(vpSet[i].size())-1;
    //         vpIdx.push_back(idx1);
    //         vpIdx.push_back(idx2);
    //     }
    //     for (int i=0;i<int(vpSet.size());i++){
    //         double minDist = INFINITY;
    //         int idx = -1;
    //         for (int j=0;j<int(vpIdx.size());j++){
    //             ViewPoint vp = vpSet[vpIdx[j].first][vpIdx[j].second];
    //             double dist = (vp.pose-start).norm();
    //             if (dist < minDist){
    //                 minDist = dist;
    //                 idx = j;
    //             }
    //         }
    //         std::vector<ViewPoint> vec = vpSet[vpIdx[idx].first];
    //         if (vpIdx[idx].second==0){
    //             plannedSeg[i] = vec;
    //             start = vec.back().pose;
    //             vpIdx.erase(vpIdx.begin()+idx);
    //             vpIdx.erase(vpIdx.begin()+idx);

    //         }
    //         else{
    //             std::reverse(vec.begin(), vec.end());
    //             plannedSeg[i] = vec;
    //             start = vec.back().pose;
    //             vpIdx.erase(vpIdx.begin()+idx);
    //             vpIdx.erase(vpIdx.begin()+idx-1);
    //         } 
    //     }
    //     return plannedSeg;
    // }

    //Lin-Kernighan heuristic symetric TSP
    std::vector<std::vector<ViewPoint>> vpPlanner::solveSequence(const std::vector<std::vector<ViewPoint>> &vpSet){
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
        for (int i=0;i<int(vpSet.size());i++){
            for (int j=0;j<int(vpSet[i].size());j++){
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
        // std::vector<ViewPoint> vpSeqTemp;
        outFile<< idx << " " << this->currPos_(0) << " " << this->currPos_(1) << " " << this->currPos_(2) << endl;
        // outFile<< idx << " 0.0 0.0 1.0" << endl;
        idx++;
        for (int i=0;i<int(vpSet.size());i++){
            for (int j=0;j<int(vpSet[i].size());j++){
                outFile << idx << " " <<vpSet[i][j].pose(0) << " " << vpSet[i][j].pose(1) << " " << vpSet[i][j].pose(2) << endl;
                // vpSeqTemp.push_back(vpSet[i][j]);
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

        // std::vector<int> nodeIdx;
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
        cout<<"nodeIdx Size: "<<nodeIdx.size()<<endl;
        inFile.close();
        for (int i=0;i<int(nodeIdx.size());i++){
            cout<<"idx: "<<nodeIdx[i]<<endl;
        }
        nodeIdx.erase(nodeIdx.begin());
        nodeIdx.pop_back();
        ros::Time end = ros::Time::now();
        cout<<"time taken: "<<(end-start).toSec()<<endl;

        // // rearrange seqence
        // std::vector<ViewPoint> vpSeq;
        // for (int i=0;i<int(nodeIdx.size());i++){
        //     vpSeq.push_back(vpSeqTemp[nodeIdx[i]-1]);
        // }

        std::vector<std::vector<ViewPoint>> vpSetArranged = this->rearrangeVP(nodeIdx, vpSet);
        return vpSetArranged;

    }

    std::vector<std::vector<ViewPoint>> vpPlanner::rearrangeVP(const std::vector<int> &vpSeq, const std::vector<std::vector<ViewPoint>> &vpSet){
        // pose processing:
        // this->vpSeq_;
        int prevSeg = -1;
        std::vector<ViewPoint> seg;
        std::vector<std::vector<ViewPoint>> vpSetArranged;
        for(int i=0;i<int(vpSeq.size());i++){
            int targetIdx = vpSeq[i];
            cout<<"vp number "<<targetIdx<<endl;
            // find segment
            int idx = -1;
            int currSeg = -1;
            for (int j=0;j<int(vpSet.size());j++){
                if (idx<targetIdx and idx+int(vpSet[j].size())>=targetIdx){
                    currSeg = j;
                    // cout<<"belongs to segment "<<currSeg<<endl;
                    break;
                }
                idx += int(vpSet[j].size());
            }

            if (currSeg != prevSeg){
                if (int(seg.size()>0)){
                    vpSetArranged.push_back(seg);
                }
                seg.clear();
                seg.push_back(vpSet[currSeg][targetIdx-idx-1]);
                // cout<<"new segment: "<<currSeg<<", "<<targetIdx-idx<<endl;
            }
            else{
                seg.push_back(vpSet[currSeg][targetIdx-idx-1]);
                // cout<<"continue segment: "<<currSeg<<", "<<targetIdx-idx<<endl;
                if (i==int(vpSeq.size()-1)){
                    vpSetArranged.push_back(seg);
                }
            }
            prevSeg = currSeg;
        }

        // TODO: Reduce the 
        cout<<"seg size: "<<vpSetArranged.size()<<endl;
        return vpSetArranged;
    }

    void vpPlanner::visCB(const ros::TimerEvent&){
        this->publishMap();
        this->publishSeg();
        this->publishViewPoints();
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
                int idx = this->getIdx(p.x, p.y, p.z);
                if (this->occupancy_.reward[idx] > 0.5){
                    points.points.push_back(p);
                }
		}
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

    void vpPlanner::publishViewPoints() {
        if (!this->vpSet_.empty()) {
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
            for (size_t i = 0; i < this->vpSet_.size(); ++i) {
                for (size_t j = 0; j < this->vpSet_[i].size(); ++j) {
                    // Re-initialize and update position for each point
                    point.pose.position.x = this->vpSet_[i][j].pose(0);
                    point.pose.position.y = this->vpSet_[i][j].pose(1);
                    point.pose.position.z = this->vpSet_[i][j].pose(2);
                    
                    text.pose.position.x = this->vpSet_[i][j].pose(0);
                    text.pose.position.y = this->vpSet_[i][j].pose(1);
                    text.pose.position.z = this->vpSet_[i][j].pose(2)+0.5;
                    
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

    void vpPlanner::updateCurrPos(const Eigen::Vector3d &currPos){
        this->currPos_ = currPos;
    }

    void vpPlanner::updateInaccessibleView(const std::vector<Eigen::Vector2i> &inaccessibleIdx){
        for (int i=0; i<int(inaccessibleIdx.size());i++){
            // get blocked viewpoint
            ViewPoint vp = this->vpSet_[inaccessibleIdx[i][0]][inaccessibleIdx[i][1]];
            // get blocked occmap
            double minAngle, maxAngle;
            minAngle = vp.yaw-M_PI/8;
            maxAngle = vp.yaw+M_PI/8;
            for (double j=minAngle;j<maxAngle;j+=0.1){
                // TODO : n param (height angle)
                for (double n=-M_PI/6;n<M_PI/6;n+=0.1){
                    Eigen::Vector3d dir{cos(j), sin(j), tan(n)};
                    for (double k=0;k<this->offset_+2.0;k+=0.1){
                        Eigen::Vector3d projPoint = vp.pose + dir*k;
                        if (this->isInMap(projPoint(0), projPoint(1),projPoint(2)) and this->isOccupied(projPoint(0), projPoint(1),projPoint(2))){
                            int pointIdx = this->getIdx(projPoint(0), projPoint(1), projPoint(2));
                            // TODO: check reward value
                            this->occupancy_.reward[pointIdx] = 2.0;
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
            if (std::abs(ang-yaw/M_PI*180)<10){
                double r = 0;
                double minAngle, maxAngle;
                minAngle = i*hres - 87/2;
                maxAngle = i*hres + 87/2;
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
                angleR.first = r;
                angleR.second = i*hres;
                angleReward.push_back(angleR);
            }
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
                viewAngle = maxElement->second/180*M_PI;
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