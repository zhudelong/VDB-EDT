/*
 * Copyright (c) Deron (Delong Zhu)
   The Chinese University of Hong Kong
   Carnegie Mellon University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include"vdb_edt/vdbmap.h"

//# define ASYNC_SPINNER

VDBMap::VDBMap():
    L_FREE(-0.13), L_OCCU(+1.01), L_THRESH(0.0), L_MAX(+3.5), L_MIN(-2.0), VOX_SIZE(0.05),
    START_RANGE(0.0), SENSOR_RANGE(5.0), HIT_THICKNESS(1), VERSION(1),
    MAX_UPDATE_DIST(20.0), EDT_UPDATE_DURATION(0.5), VIS_UPDATE_DURATION(10),
    VIS_MAP_MINX(-200.0), VIS_MAP_MINY(-200.0), VIS_MAP_MINZ(-1.0),
    VIS_MAP_MAXX(+200.0), VIS_MAP_MAXY(+200.0), VIS_MAP_MAXZ(10.0), VIS_SLICE_LEVEL(2.0),
    msg_ready_(false), occu_update_count_(0), dist_update_count_(0)
{
    node_handle_ = new ros::NodeHandle();
    private_node_handle_ = new ros::NodeHandle("~");

    setup_parameters();
    load_mapping_para();

    // LADY_AND_COW dataset
    ref_transform_<< 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
    T_B_C_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    T_D_B_ << 0.971048, -0.120915,  0.206023, 0.00114049,
              0.157010,  0.973037, -0.168959, 0.04509360,
             -0.180038,  0.196415,  0.963850, 0.04307650,
              0.0, 0.0, 0.0, 1.0;


    // lady_and_cow dataset
    cloud_sub_ = node_handle_->subscribe
            ("/camera/depth_registered/points", 10, &VDBMap::lady_cow_cloud_callback, this);
    pose_sub_ = node_handle_->subscribe
            ("/kinect/vrpn_client/estimated_transform", 10, &VDBMap::lady_cow_pose_callback, this);


    // general synced dataset
    pointCloudSub_ = new message_filters::Subscriber<CloudMsg>(*node_handle_, pcl_topic, 5);
    tfPointCloudSub_ = new tf::MessageFilter<CloudMsg> (*pointCloudSub_, tfListener_, worldframeId, 5);
    tfPointCloudSub_->registerCallback(boost::bind(&VDBMap::cloud_callback, this, _1));

    // visualization dataset
    occu_vis_pub_ = node_handle_->advertise<sensor_msgs::PointCloud2>("/occ_grid", 5);
    slice_vis_pub_ = node_handle_->advertise<VisMarker>("/dist_slice", 5);


    // initialization grid map
    openvdb::initialize();
    grid_logocc_ = openvdb::FloatGrid::create(0.0);
    this->set_voxel_size(*grid_logocc_, VOX_SIZE);
    max_coor_dist_ = int(MAX_UPDATE_DIST / VOX_SIZE);
    max_coor_sqdist_ = max_coor_dist_ * max_coor_dist_;
    grid_distance_ = std::make_shared<DynamicVDBEDT>(max_coor_dist_);
    grid_distance_->initialize(dist_map_, VOX_SIZE, VERSION);
    grid_distance_->setAccessor(dist_map_);


    // convert SENSOR_RANGE to index space
    FloatGrid::Accessor acc = grid_logocc_->getAccessor();
    Vec3d max_sense_dist(SENSOR_RANGE);
    Vec3d sense_range_ijk = grid_logocc_->worldToIndex(max_sense_dist);
    SENSOR_RANGE = sense_range_ijk.x();

    Vec3d min_sense_dist(START_RANGE);
    Vec3d min_sense_ijk = grid_logocc_->worldToIndex(min_sense_dist);
    START_RANGE = min_sense_ijk.x();


    // timer for distance map updation
    update_edt_timer_ = node_handle_->createTimer(ros::Duration(EDT_UPDATE_DURATION),
                                                  &VDBMap::update_edtmap, this);
    update_vis_timer_ = node_handle_->createTimer(ros::Duration(VIS_UPDATE_DURATION),
                                                   &VDBMap::visualize_maps, this);
}



void VDBMap::cloud_callback(const CloudMsg::ConstPtr &pc_msg)
{
    tf::StampedTransform transform;
    std::string pcl_frame = pc_msg->header.frame_id;

    // pcl frame id is used for raycasting origin lookup
    try{
        tfListener_.lookupTransform(worldframeId, pc_msg->header.frame_id, pc_msg->header.stamp, transform);
    }catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    auto origin = transform.getOrigin();
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2Ptr pc_tform_msg(new sensor_msgs::PointCloud2);

    // transforming pc message to map frame
    pcl_ros::transformPointCloud(worldframeId, transform, *pc_msg, *pc_tform_msg);

    // converting messages to pcl data type
    pcl::fromROSMsg(*pc_tform_msg, *xyz);

    // update float occupancy map
    occu_update_count_ ++;
    std::cout << "Running " << occu_update_count_ << " updates." << std::endl;
    timing::Timer update_OCC_timer("UpdateOccu");
    this->update_occmap(grid_logocc_, origin, xyz);
    update_OCC_timer.Stop();
    timing::Timing::Print(std::cout);
    msg_ready_ = true;
}



void VDBMap::lady_cow_pose_callback(const PoseMsg::ConstPtr &pose)
{
    pose_queue_.push(*pose);
}


void VDBMap::lady_cow_cloud_callback(const CloudMsg::ConstPtr &cloud)
{
    cloud_queue_.push(*cloud);
    sync_pose_and_cloud_fiesta();
}


bool VDBMap::sync_pose_and_cloud(PoseMsg& latest_pose, const CloudMsg& latest_cloud)
{
    double time_delay = 3e-3; //3e-3;
    auto duration = ros::Duration(time_delay);

    // sleep 0.006 second for future pose
#ifdef ASYNC_SPINNER
    duration.sleep();
#endif

    // index the nearest pose
    std::vector<PoseMsg> candidate_poses;
    auto cloud_time = latest_cloud.header.stamp;
    uint check_count = pose_queue_.size();

    ReadLock read_lock(pose_queue_lock);
    while (check_count > 0){
        auto pose_time = pose_queue_.front().header.stamp;

        check_count --; // decrease check count
        if ((cloud_time - pose_time) > duration){ // throw poses behind the cloud
            pose_queue_.pop();
            continue;
        } else if ((pose_time - cloud_time) < duration){ // not consider poses in far future
            continue; // only with ASYNC_SPINNER, this will happen
        } else {
            candidate_poses.push_back(pose_queue_.front());
            pose_queue_.pop();
        }
    }

    // no pose is matched
    if (candidate_poses.empty()){
        return false;
    }

    // search nearest pose
    int mini_cost = INT_MAX;
    uint mini_count = POSE_QUEUE_SIZE + 1;
    for (uint i = 0; i < candidate_poses.size(); ++i) {
        int cost = abs((candidate_poses[i].header.stamp - cloud_time).nsec);
        if (cost < mini_cost) {
            mini_cost = cost;
            mini_count = i;
        }
    }
    latest_pose = candidate_poses[mini_count];
    std::cout <<  latest_pose.header.seq << std::endl;
    std::cout <<  latest_cloud.header.seq << std::endl;
    return true;
}


void VDBMap::sync_pose_and_cloud_fiesta()
{
    double time_delay = 3e-3; //3e-3;
    auto duration = ros::Duration(time_delay);

    while (!cloud_queue_.empty()) {
        bool new_pos = false;
        auto cloud_time = cloud_queue_.front().header.stamp;
        while (pose_queue_.size() > 1 && pose_queue_.front().header.stamp <= cloud_time + duration) {
            latest_pose_ = pose_queue_.front();
            pose_queue_.pop();
             new_pos = true;
        }

        if (pose_queue_.empty() || pose_queue_.front().header.stamp <= cloud_time + duration) {
             break;
        }

        if (!new_pos) {
             cloud_queue_.pop();
             continue;
        }

        msg_ready_ = true;

        // calculate tranfromation
        ref_transform_ = cur_transform_;
        auto translation = Eigen::Vector3d(latest_pose_.transform.translation.x,
                                           latest_pose_.transform.translation.y,
                                           latest_pose_.transform.translation.z);
        auto rotation = Eigen::Quaterniond(latest_pose_.transform.rotation.w,
                                           latest_pose_.transform.rotation.x,
                                           latest_pose_.transform.rotation.y,
                                           latest_pose_.transform.rotation.z);
        cur_transform_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        cur_transform_.block<3, 1>(0, 3) = translation;
        cur_transform_(3, 0) = cur_transform_(3, 1) = cur_transform_(3, 2) = 0;
        cur_transform_(3, 3) = 1;
        cur_transform_ = cur_transform_ * T_D_B_ * T_B_C_;
        Eigen::Affine3d cur_eig_trans;
        cur_eig_trans.matrix() = cur_transform_;
        tf::Transform cur_tf_trans;
        tf::transformEigenToTF(cur_eig_trans, cur_tf_trans);
        origin_ = cur_tf_trans.getOrigin();


	occu_update_count_ ++;
        // transform point cloud from robot frame to world frame
        if (cloud_queue_.front().data.size() == 0){
            std::cout << "invalid point data!" << std::endl;
            cloud_queue_.pop();
            continue;
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2Ptr cloud_out(new sensor_msgs::PointCloud2);
        auto cloud_in = cloud_queue_.front();
        cloud_in.header.frame_id = robotframeId;

        pcl_ros::transformPointCloud(worldframeId, cur_tf_trans, cloud_queue_.front(), *cloud_out);
        pcl::fromROSMsg(*cloud_out, *xyz);


        // update float occupancy map
        std::cout << "Running " << occu_update_count_ << " updates." << std::endl;
        timing::Timer update_OCC_timer("UpdateOccu");
        this->update_occmap(grid_logocc_, origin_, xyz);
        update_OCC_timer.Stop();
        timing::Timing::Print(std::cout);

        cloud_queue_.pop();
    }

}


VDBMap::~VDBMap()
{
    delete node_handle_;
    delete private_node_handle_;
    delete tfPointCloudSub_;
    delete pointCloudSub_;
}


//////////////////////////////////////////////////////////////////////////////////
std::string VDBMap::get_node_name()
{
    return ros::this_node::getName();
}

ros::NodeHandle *VDBMap::get_node_handle()
{
    return node_handle_;
}

ros::NodeHandle *VDBMap::get_private_node_handle()
{
    return private_node_handle_;
}


void VDBMap::set_voxel_size(openvdb::GridBase &grid, double vs)
{
    const openvdb::math::Vec3d offset(vs/2., vs/2., vs/2.);
    openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(vs);
    tf->postTranslate(offset);
    grid.setTransform(tf);
}



//////////////////////////////////////////////////////////////////////////////////

void VDBMap::setup_parameters()
{
    ros::NodeHandle* pnh = get_private_node_handle();
    pnh->setParam("pcl_topic", "cloud_in");
    pnh->setParam("world_frame_id", "world");
    pnh->setParam("robot_frame_id", "robot");
    pnh->setParam("data_set", "subt");
}

bool VDBMap::load_mapping_para()
{
    ros::NodeHandle* pnh = get_private_node_handle();
    // pcl topic
    if (pnh->getParam("pcl_topic", pcl_topic)) {
        ROS_INFO("Parameter pcl_topic set to: %s", pcl_topic.c_str());
    } else {
        ROS_ERROR("Please set input point cloud topic before running the node.");
        return false;
    }

    // world frame id
    if (pnh->getParam("world_frame_id", worldframeId)) {
        ROS_INFO("Parameter world_frame_id set to: %s", worldframeId.c_str());
    } else {
        ROS_ERROR("Please set input world frame id before running the node.");
        return false;
    }

    // robot frame id
    if (pnh->param<std::string>("robot_frame_id", robotframeId, " ")) {
        ROS_INFO("Overriding robot_frame_id to: %s", robotframeId.c_str());
    } else {
        ROS_ERROR("Please set input robot frame id before running the node.");
        return false;
    }

    // voxel map parameters
    if (pnh->param<double>("l_free", L_FREE, L_FREE)) {
        ROS_INFO("Overriding Parameter L_FREE to: %f", L_FREE);
    } else {
        ROS_WARN("Using the default L_FREE: %f", L_FREE);
    }

    if (pnh->param<double>("l_occu", L_OCCU, L_OCCU)) {
        ROS_INFO("Overriding Parameter L_OCCU to: %f", L_OCCU);
    } else {
        ROS_WARN("Using the default L_OCCU: %f", L_OCCU);
    }

    if (pnh->param<double>("l_max", L_MAX, L_MAX)) {
        ROS_INFO("Overriding Parameter L_MAX to: %f", L_MAX);
    } else {
        ROS_WARN("Using the default L_MAX: %f", L_MAX);
    }

    if (pnh->param<double>("l_min", L_MIN, L_MIN)) {
        ROS_INFO("Overriding Parameter L_MIN to: %f", L_MIN);
    } else {
        ROS_WARN("Using the default L_MIN: %f", L_MIN);
    }

    if (pnh->param<double>("l_thresh", L_THRESH, L_THRESH)) {
        ROS_INFO("Overriding Parameter L_THRESH to: %f", L_THRESH);
    } else {
        ROS_WARN("Using the default L_THRESH: %f", L_THRESH);
    }

    if (pnh->param<double>("vox_size", VOX_SIZE, VOX_SIZE)) {
        ROS_INFO("Overriding Parameter VOX_SIZE to: %f", VOX_SIZE);
    } else {
        ROS_WARN("Using the default VOX_SIZE: %f", VOX_SIZE);
    }

    if (pnh->param<double>("start_range", START_RANGE, START_RANGE)) {
        ROS_INFO("Overriding Parameter START_RANGE to: %f", START_RANGE);
    } else {
        ROS_WARN("Using the default START_RANGE: %f", START_RANGE);
    }

    if (pnh->param<double>("sensor_range", SENSOR_RANGE, SENSOR_RANGE)) {
        ROS_INFO("Overriding Parameter SENSOR_RANGE to: %f", SENSOR_RANGE);
    } else {
        ROS_WARN("Using the default SENSOR_RANGE: %f", SENSOR_RANGE);
    }

    // parameters for vdb edt
    if (pnh->param<int>("vdbedt_version", VERSION, VERSION)) {
        ROS_INFO("Overriding Parameter VDBEDT-VERSION to: %d", VERSION);
    } else {
        ROS_WARN("Using the default VDBEDT-VERSION: %d", VERSION);
    }

    if (pnh->param<double>("max_update_dist", MAX_UPDATE_DIST, MAX_UPDATE_DIST)) {
        ROS_INFO("Overriding Parameter MAX_UPDATE_DIST to: %f", MAX_UPDATE_DIST);
    } else {
        ROS_WARN("Using the default MAX_UPDATE_DIST: %f", MAX_UPDATE_DIST);
    }

    if (pnh->param<double>("edt_update_duration", EDT_UPDATE_DURATION, EDT_UPDATE_DURATION)) {
        ROS_INFO("Overriding Parameter EDT_UPDATE_DURATION to: %f", EDT_UPDATE_DURATION);
    } else {
        ROS_WARN("Using the default EDT_UPDATE_DURATION: %f", EDT_UPDATE_DURATION);
    }

    // Parameters for visualization
    if (pnh->param<double>("vis_update_duration", VIS_UPDATE_DURATION, VIS_UPDATE_DURATION)) {
        ROS_INFO("Overriding Parameter VIS_UPDATE_DURATION to: %f", VIS_UPDATE_DURATION);
    } else {
        ROS_WARN("Using the default VIS_UPDATE_DURATION: %f", VIS_UPDATE_DURATION);
    }

    if (pnh->param<double>("vis_slice_level", VIS_SLICE_LEVEL, VIS_SLICE_LEVEL)) {
        ROS_INFO("Overriding Parameter VIS_SLICE_LEVEL to: %f", VIS_SLICE_LEVEL);
    } else {
        ROS_WARN("Using the default VIS_SLICE_LEVEL: %f", VIS_SLICE_LEVEL);
    }

    if (pnh->param<double>("vis_map_minx", VIS_MAP_MINX, VIS_MAP_MINX)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MINX to: %f", VIS_MAP_MINX);
    } else {
        ROS_WARN("Using the default VIS_MAP_MINX: %f", VIS_MAP_MINX);
    }

    if (pnh->param<double>("vis_map_miny", VIS_MAP_MINY, VIS_MAP_MINY)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MINY to: %f", VIS_MAP_MINY);
    } else {
        ROS_WARN("Using the default VIS_MAP_MINY: %f", VIS_MAP_MINY);
    }

    if (pnh->param<double>("vis_map_minz", VIS_MAP_MINZ, VIS_MAP_MINZ)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MINZ to: %f", VIS_MAP_MINZ);
    } else {
        ROS_WARN("Using the default VIS_MAP_MINZ: %f", VIS_MAP_MINZ);
    }

    if (pnh->param<double>("vis_map_maxx", VIS_MAP_MAXX, VIS_MAP_MAXX)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MAXX to: %f", VIS_MAP_MAXX);
    } else {
        ROS_WARN("Using the default VIS_MAP_MAXX: %f", VIS_MAP_MAXX);
    }

    if (pnh->param<double>("vis_map_maxy", VIS_MAP_MAXY, VIS_MAP_MAXY)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MAXY to: %f", VIS_MAP_MAXY);
    } else {
        ROS_WARN("Using the default VIS_MAP_MAXY: %f", VIS_MAP_MAXY);
    }

    if (pnh->param<double>("vis_map_maxz", VIS_MAP_MAXZ, VIS_MAP_MAXZ)) {
        ROS_INFO("Overriding Parameter VIS_MAP_MAXZ to: %f", VIS_MAP_MAXZ);
    } else {
        ROS_WARN("Using the default VIS_MAP_MAXZ: %f", VIS_MAP_MAXZ);
    }

    return true;
}

bool VDBMap::load_planning_para()
{
//   ros::NodeHandle* pnh = get_private_node_handle();
   std::cout << "planning parameters no set !" << std::endl;
   return true;
}


////////////////////////////////////////////////////////////////////////////////
void VDBMap::visualize_maps(const ros::TimerEvent &)
{
    // visualize the occupancy map
    if (occu_vis_pub_.getNumSubscribers() > 0) {
        CloudMsg cloud_vis;
        this->grid_message(grid_logocc_, cloud_vis);
        occu_vis_pub_.publish(cloud_vis);
    }

    // visualize the distance map
    if (slice_vis_pub_.getNumSubscribers() > 0){
        VisMarker slice_maker;
        auto vis_coor = int(2.0 / VOX_SIZE);
        this->get_slice_marker(slice_maker, 100, VIS_SLICE_LEVEL, vis_coor*vis_coor);
        slice_vis_pub_.publish(slice_maker);
    }

}

void VDBMap::grid_to_pcl(FloatGrid::Ptr grid, FloatGrid::ValueType thresh, XYZICloud::Ptr& pc_out)
{

    pcl::PointXYZI point_xyzi;

    using value_type = openvdb::FloatGrid::ValueType;
    using itr_type = openvdb::FloatGrid::ValueOnCIter;

    const openvdb::math::Transform& grid_tf(grid->transform());

    // just a vector of bytes
    // std::vector<uint8_t> pts;
    // pts.reserve(grid->activeVoxelCount()*16);

    for (itr_type itr = grid->cbeginValueOn(); itr.test(); ++itr) {
        if (!itr.isVoxelValue()) {
            continue;
        }
        value_type val = itr.getValue();
        if (val < thresh) {
            continue;
        }
        openvdb::Coord ijk = itr.getCoord();
        openvdb::Vec3d p = grid_tf.indexToWorld(ijk);

        point_xyzi.x = p.x();
        point_xyzi.y = p.y();
        point_xyzi.z = p.z();
        point_xyzi.intensity = val;
        pc_out->points.push_back(point_xyzi);
    }
    pc_out->width = (uint32_t) pc_out->points.size(); pc_out->height = 1;
}


void VDBMap::grid_message(FloatGrid::Ptr &grid, CloudMsg &disp_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    this->grid_to_pcl(grid, L_THRESH, pcl_cloud);
    pcl::toROSMsg(*pcl_cloud, disp_msg);
    disp_msg.header.frame_id = worldframeId;
    disp_msg.header.stamp = ros::Time::now();
}


void VDBMap::update_edtmap(const ros::TimerEvent & /*event*/)
{
    if (!msg_ready_){
        return;
    }
    // update float occupancy map
    dist_update_count_ ++;
    std::cout << "Running " << dist_update_count_ << " updates." << std::endl;
    timing::Timer update_DIST_timer("UpdateDIST");
    this->grid_distance_->update();
    update_DIST_timer.Stop();
    timing::Timing::Print(std::cout);
    std::cout <<  "changed: "  << grid_distance_->sum_occ_changed <<
                  " raised: "  << grid_distance_->sum_raised_num <<
                  " lowered: " << grid_distance_->sum_lowered_num << std::endl;
    msg_ready_ = false;
}


void VDBMap::update_occmap(FloatGrid::Ptr grid_map, const tf::Vector3 &origin, XYZCloud::Ptr xyz)
{
    auto grid_acc = grid_map->getAccessor();
    auto tfm = grid_map->transform();

    openvdb::Vec3d origin3d(origin.x(), origin.y(), origin.z());
    openvdb::Vec3d origin_ijk = grid_map->worldToIndex(origin3d);

    for (auto point = xyz->begin(); point != xyz->end(); ++point) {
        openvdb::Vec3d p_xyz(point->x, point->y, point->z);
        openvdb::Vec3d p_ijk = grid_map->worldToIndex(p_xyz);
        openvdb::Vec3d dir(p_ijk - origin_ijk);
        double range = dir.length();
        dir.normalize();

        // Note: real sensor range should stractly larger than sensor_range
        bool truncated = false;
        openvdb::math::Ray<double> ray(origin_ijk, dir);
        // openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0., std::min(SENSOR_RANGE, range));

//        if (START_RANGE >= std::min(SENSOR_RANGE, range)){
//            continue;
//        }
        openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0, std::min(SENSOR_RANGE, range));

        // decrease occupancy
        do {
            openvdb::Coord ijk(dda.voxel());

            float ll_old;
            bool isKnown = grid_acc.probeValue(ijk, ll_old);
            float ll_new = std::max(L_MIN, ll_old+L_FREE);

            if(!isKnown){
                grid_distance_->dist_acc_->setValueOn(ijk);
            } // unknown -> free -> EDT initialize

            else if(ll_old >= 0 && ll_new < 0){
                grid_distance_->removeObstacle(ijk);
            } // occupied -> free -> EDT RemoveObstacle

            grid_acc.setValueOn(ijk, ll_new);
            dda.step();

        } while (dda.time() < dda.maxTime());

        // increase occupancy
        if ((!truncated) && (range <= SENSOR_RANGE)){
            for (int i=0; i < HIT_THICKNESS; ++i) {
                openvdb::Coord ijk(dda.voxel());

                float ll_old;
                bool isKnown = grid_acc.probeValue(ijk, ll_old);
                float ll_new = std::min(L_MAX, ll_old+L_OCCU);

                if(!isKnown){
                    grid_distance_->dist_acc_->setValueOn(ijk);
                } // unknown -> occupied -> EDT SetObstacle
                else if(ll_old < 0 && ll_new >= 0){
                    grid_distance_->setObstacle(ijk);
                } // free -> occupied -> EDT SetObstacle

                grid_acc.setValueOn(ijk, ll_new);
                dda.step();
            }
        } // process obstacle

    } // end inserting

    /* commit changes to the open queue*/
}


void VDBMap::get_slice_marker(VisMarker &marker, int marker_id, double slice, double max_sqdist)
{
    marker.header.frame_id = worldframeId;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.scale.x = VOX_SIZE;
    marker.scale.y = VOX_SIZE;
    marker.scale.z = VOX_SIZE;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.points.clear();
    marker.colors.clear();

    // iterate the map
    std_msgs::ColorRGBA c;
    Vec3d map_min(VIS_MAP_MINX, VIS_MAP_MINY, VIS_MAP_MINZ);
    Vec3d map_max(VIS_MAP_MAXX, VIS_MAP_MAXY, VIS_MAP_MAXZ);
    auto coor_min = dist_map_->worldToIndex(map_min);
    auto coor_max = dist_map_->worldToIndex(map_max);

    if (slice < coor_min.z() || slice > coor_max.z()){
        std::cout << "slice number is out of boundary!" << std::endl;
        return;
    }

    int z =  dist_map_->worldToIndex(Vec3d(slice)).x();
    for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
        for (int y = coor_min.y(); y <= coor_max.y(); ++y){
            Coord target_cell(x, y, z);
            Coord nearest_obst;
            auto cell_dist = grid_distance_->query_sq_distance(target_cell, nearest_obst);

            if (cell_dist < 0 || cell_dist >= max_coor_dist_){ // unknown cells
                continue;
            }
            auto world_xyz = dist_map_->indexToWorld(target_cell);

            geometry_msgs::Point p;
            p.x = world_xyz.x();
            p.y = world_xyz.y();
            p.z = world_xyz.z();

            c = rainbow_color_map(sqrt(cell_dist) < sqrt(max_sqdist)? sqrt(cell_dist * 1.0 / max_sqdist ) : 1);
            marker.points.push_back(p);
            marker.colors.push_back(c);

        } // end y loop

    } // end x loop
}


std_msgs::ColorRGBA VDBMap::rainbow_color_map(double h)
{
    std_msgs::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
      f = 1 - f;  // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
      case 6:
      case 0:color.r = v;
        color.g = n;
        color.b = m;
        break;
      case 1:color.r = n;
        color.g = v;
        color.b = m;
        break;
      case 2:color.r = m;
        color.g = v;
        color.b = n;
        break;
      case 3:color.r = m;
        color.g = n;
        color.b = v;
        break;
      case 4:color.r = n;
        color.g = m;
        color.b = v;
        break;
      case 5:color.r = v;
        color.g = m;
        color.b = n;
        break;
      default:color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }
    return color;
}
