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
        this->generateViewPoint();
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
    }

    void vpPlanner::registerPub(){
        this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/ref_map", 10);
        this->pointVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/view_points", 10);
        this->segMapVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/seg_map", 10);
        this->normalVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/normal", 10);
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
                cout << this->hint_ << ": Map loaded with " << this->refCloud_.width * this->refCloud_.height << " data points. " << endl;			
            }
        }
        else if (type == "STL"){
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
		
	}

    void vpPlanner::visCB(const ros::TimerEvent&){
        this->publishMap();
        this->publishSeg();
        this->publishViewPoints();
    }

    void vpPlanner::publishMap(){
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(this->refCloud_, cloudMsg);
        cloudMsg.header.frame_id = "map";
        cloudMsg.header.stamp = ros::Time::now();
        this->mapVisPub_.publish(cloudMsg);
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

            int id = 0; // Unique marker ID
            for (size_t i = 0; i < this->vpSet_.size(); ++i) {
                for (size_t j = 0; j < this->vpSet_[i].size(); ++j) {
                    // Re-initialize and update position for each point
                    point.pose.position.x = this->vpSet_[i][j].pose(0);
                    point.pose.position.y = this->vpSet_[i][j].pose(1);
                    point.pose.position.z = this->vpSet_[i][j].pose(2);
                    point.id = id++; // Assign unique ID for each marker

                    // Add the marker to the array
                    points.markers.push_back(point);
                }
            }

            // Publish the MarkerArray
            this->pointVisPub_.publish(points);
        }
    }



    void vpPlanner::segMap(){
        std::vector<globalPlanner::ClusterInfo> clusterInfos;  // Vector to store cluster info
        cout<<"segmenting .."<<endl;

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(this->refCloud_.makeShared());
        pass.setFilterFieldName("z");  // Filter along the z-axis, for example
        // TODO: param groundHeight, ceilingHeight
        pass.setFilterLimits(1.0, 5.0); // Adjust range based on your path
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(this->refCloud_);

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
        reg.setMinClusterSize(500);
        reg.setMaxClusterSize(10000);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
        reg.setSearchMethod(tree2);
        reg.setNumberOfNeighbours(100);
        reg.setInputCloud(this->refCloud_.makeShared());
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // 3 degrees
        reg.setCurvatureThreshold(1.0);
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // 3. Process each cluster to compute the AABB vertices and average normal
        for (int i=0;i<int(clusters.size());i++) {
            pcl::PointCloud<pcl::PointXYZ> cluster;
            Eigen::Vector3d avgNormal{0.0, 0.0, 0.0};
            for (int j=0;j<int(clusters[i].indices.size());j++) {
                cluster.points.push_back(this->refCloud_.points[clusters[i].indices[j]]);
                pcl::Normal normal = normals->points[clusters[i].indices[j]];
                Eigen::Vector3d n{normal.normal_x, normal.normal_y, normal.normal_z};
                avgNormal += n;
            }

            pcl::PointXYZ minPoint, maxPoint;
            avgNormal /= int(clusters[i].indices.size());
            avgNormal(2) = 0.0;
            // Use getMinMax3D to find the min and max points
            pcl::getMinMax3D(cluster, minPoint, maxPoint);

            // Store the cluster info
            ClusterInfo info = this->genClusterInfo(avgNormal,cluster);
            // info.vert = vertices;
            // info.normal = avgNormal;
            clusterInfos.push_back(info);
        }
        this->segMap_ = clusterInfos;
    }

    // void vpPlanner::visClusters(){
    //     for (ClusterInfo cluster: this->segMap_){
    //         Eigen::Vector3d axis = cluster.noraml;
    //         Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    //         transformation.block<3, 1>(0, 1) = axis; // Align with the Y-axis (example)
            
    //         pcl::PointCloud<pcl::PointXYZ> rotatedCloud;
    //         pcl::transformPointCloud(cluster, rotatedCloud, rotation_matrix);
    //         pcl::PointXYZ minPoint, maxPoint;
    //         pcl::getMinMax3D(rotatedCloud, minPoint, maxPoint);

    //         // Compute the 8 vertices of the AABB
    //         std::vector<Eigen::Vector3d> vertices(8);
    //         vertices[0] = Eigen::Vector3d(minPoint.x, minPoint.y, minPoint.z);  // Vertex 0
    //         vertices[1] = Eigen::Vector3d(minPoint.x, maxPoint.y, minPoint.z);  // Vertex 1
    //         vertices[2] = Eigen::Vector3d(maxPoint.x, maxPoint.y, minPoint.z);  // Vertex 2
    //         vertices[3] = Eigen::Vector3d(maxPoint.x, minPoint.y, minPoint.z);  // Vertex 3
    //         vertices[4] = Eigen::Vector3d(minPoint.x, minPoint.y, maxPoint.z);  // Vertex 4
    //         vertices[5] = Eigen::Vector3d(minPoint.x, maxPoint.y, maxPoint.z);  // Vertex 5
    //         vertices[6] = Eigen::Vector3d(maxPoint.x, maxPoint.y, maxPoint.z);  // Vertex 6
    //         vertices[7] = Eigen::Vector3d(maxPoint.x, minPoint.y, maxPoint.z);  // Vertex 7



    //     }

    // }

    ClusterInfo vpPlanner::genClusterInfo(const Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ> &cluster){
        // Step 4: Compute the rotation matrix to align with the desired axis
        Eigen::Vector3d targetAxis(0, 1, 0); // Aligning with the Y-axis
        // Eigen::Vector3f rotationAxis = secondPrincipalAxis.cross(targetAxis);
        Eigen::Vector3d rotationAxis = normal.cross(targetAxis);
        double angle = std::acos(normal.dot(targetAxis) / (normal.norm() * targetAxis.norm()));

        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(angle, rotationAxis.normalized()).toRotationMatrix();

        // Step 5: Apply the rotation to the entire point cloud
        for (auto& point : cluster.points) {
            Eigen::Vector3d pointVec(point.x, point.y, point.z);
            pointVec = rotationMatrix * pointVec; // Rotate the point
            point.x = pointVec(0);
            point.y = pointVec(1);
            point.z = pointVec(2);
        }

        pcl::PointXYZ minPoint, maxPoint;
        // pcl::getMinMax3D(cluster, minPoint, maxPoint);
        pcl::getMinMax3D(cluster, minPoint, maxPoint);


        // // TODO: inflate param: consider FOV

        double xinflate, yinflate, zinflate;
        xinflate = 0.0;
        yinflate = 0.0;
        zinflate = 0.0;
        // Compute the 8 vertices of the AABB
        std::vector<Eigen::Vector3d> vertices(8);
        vertices[0] = Eigen::Vector3d(minPoint.x-xinflate, minPoint.y-yinflate, minPoint.z-zinflate);  // Vertex 0
        vertices[1] = Eigen::Vector3d(minPoint.x-xinflate, maxPoint.y+yinflate, minPoint.z-zinflate);  // Vertex 1
        vertices[2] = Eigen::Vector3d(maxPoint.x+xinflate, maxPoint.y+yinflate, minPoint.z-zinflate);  // Vertex 2
        vertices[3] = Eigen::Vector3d(maxPoint.x+xinflate, minPoint.y-yinflate, minPoint.z-zinflate);  // Vertex 3
        vertices[4] = Eigen::Vector3d(minPoint.x-xinflate, minPoint.y-yinflate, maxPoint.z+zinflate);  // Vertex 4
        vertices[5] = Eigen::Vector3d(minPoint.x-xinflate, maxPoint.y+yinflate, maxPoint.z+zinflate);  // Vertex 5
        vertices[6] = Eigen::Vector3d(maxPoint.x+xinflate, maxPoint.y+yinflate, maxPoint.z+zinflate);  // Vertex 6
        vertices[7] = Eigen::Vector3d(maxPoint.x+xinflate, minPoint.y-yinflate, maxPoint.z+zinflate);  // Vertex 7

        Eigen::Matrix3d rotationMatrix2 = Eigen::AngleAxisd(-angle, rotationAxis.normalized()).toRotationMatrix();

        for (int i=0;i<8;i++){
            Eigen::Vector3d point = vertices[i];
            point = rotationMatrix2*point;
            vertices[i] = point;
        }
        // cout<<"normal: "<<norm<<endl;
        // Store the cluster info
        ClusterInfo info;
        info.vert = vertices;
        info.normal = normal;
        return info;

    }

    void vpPlanner::generateViewPoint(){
        std::vector<std::vector<ViewPoint>> vpSet;
        for (const ClusterInfo &cluster: this->segMap_){
            std::vector<ViewPoint> vps;
            Eigen::Vector3d n = cluster.normal;
            
            // ViewPoint vp;
            // Eigen::Vector3d viewPoint;

            std::vector<Eigen::Vector3d> vertex = cluster.vert;
            
            int vert_idx[2][4] = {
                {0,1,3,2},
                {1,2,0,3}
            };
            double height = vertex[4](2)-vertex[0](2);
            // cout<<"height: "<<height<<endl;
            // TODO: Param step dist between each view point is step
            double step = 1.0;
            for(int i=0;i<2;i++){
                Eigen::Vector3d start1 = vertex[vert_idx[i][0]];
                Eigen::Vector3d end1 = vertex[vert_idx[i][1]];
                Eigen::Vector3d start2 = vertex[vert_idx[i][2]];
                Eigen::Vector3d end2 = vertex[vert_idx[i][3]];

                // double height = vertex[vert_idx[i][1]](2)-vertex[vert_idx[i][0]](2);
                
                Eigen::Vector3d direction = end1-start1;
                // check orientation to normal
                double cosTheta = (direction.dot(n))/(direction.norm()*n.norm());
                cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
                double angle = std::acos(cosTheta);
                // cout<<"vert idx: "<<i<<", angle: "<<angle<<endl;
                if (std::abs(angle-M_PI/2)<=0.1){
                    double globalYaw = atan2(n(1),n(0));

                    for(double dist = 0.0;dist<direction.norm();dist+=step){
                        // cout<<"dist: "<<dist<<endl;
                        Eigen::Vector3d p1 = start1 + dist*direction/direction.norm();
                        Eigen::Vector3d p2 = start2 + dist*direction/direction.norm();
                        Eigen::Vector3d p = (p1 + p2) / 2;
                        p = p + 2.0 * n / n.norm();
                        // points.push_back(point);
                        // ViewPoint vp;
                        // vp. pose = point;
                        // vp.yaw = globalYaw;
                        for (double j=0.0;j<height;j+=step){
                            Eigen::Vector3d point = p;
                            point(2) += j;
                            // cout<<"point height: "<<point(2)<<endl;
                            ViewPoint vp;
                            vp.pose = point;
                            vp.yaw = globalYaw;
                            vps.push_back(vp);
                            // j+=step;
                        }
                        
                    }
                }
            }

            vpSet.push_back(vps);
        }
        this->vpSet_ = vpSet;
        // pose process: ignore edges and overlap
        

    }

    // save all the view point. stamps are goals, rest for check

}