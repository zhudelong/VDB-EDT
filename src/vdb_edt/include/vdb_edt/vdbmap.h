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

#ifndef _VDBMAP_H_
#define _VDBMAP_H_

#include <queue>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_msgs/ColorRGBA.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>

#include "openvdb/openvdb.h"
#include "openvdb/Grid.h"
#include "openvdb/math/DDA.h"
#include "openvdb/math/Ray.h"

#include "vdb_edt/dynamicVDBEDT.h"
#include "vdb_edt/timing.h"

#define POSE_QUEUE_SIZE 20
using namespace openvdb;



class VDBMap{

public:
    VDBMap();
    ~VDBMap();


private:
    // General parameters
    std::string pcl_topic;
    std::string worldframeId;
    std::string robotframeId;
    std::string dataset;

    // Mapping parameters
    double L_FREE, L_OCCU, L_THRESH, L_MIN, L_MAX, VOX_SIZE;
    double START_RANGE, SENSOR_RANGE;
    int HIT_THICKNESS;


    // VDB map
    int VERSION;
    double MAX_UPDATE_DIST;
    double VIS_MAP_MINX, VIS_MAP_MINY, VIS_MAP_MINZ; // for visualization
    double VIS_MAP_MAXX, VIS_MAP_MAXY, VIS_MAP_MAXZ; // for visualization
    double EDT_UPDATE_DURATION;
    double VIS_UPDATE_DURATION;
    double VIS_SLICE_LEVEL; // in meters



protected:
    // Ros related
    std::string node_name_;
    ros::NodeHandle* node_handle_;
    ros::NodeHandle* private_node_handle_;
    // Returns the name the user gave to the node/nodelet
    std::string get_node_name();
    // Get node handles.
    ros::NodeHandle* get_node_handle();
    ros::NodeHandle* get_private_node_handle();


public:
    typedef sensor_msgs::PointCloud2 CloudMsg;
    typedef geometry_msgs::TransformStamped PoseMsg;

    // read-write lock
    typedef boost::shared_mutex Lock;
    typedef boost::unique_lock<Lock> WriteLock;
    typedef boost::shared_lock<Lock> ReadLock;
    Lock pose_queue_lock;


    // common tool functions
    void setup_parameters();
    bool load_mapping_para();
    bool load_planning_para();


    // for visulization
    ros::Timer update_vis_timer_;
    ros::Publisher occu_vis_pub_;
    ros::Publisher slice_vis_pub_;
    void visualize_maps(const ros::TimerEvent &);



    // general dataset with tf and point cloud
    tf::TransformListener tfListener_;
    tf::MessageFilter<CloudMsg>* tfPointCloudSub_;
    message_filters::Subscriber<CloudMsg>* pointCloudSub_;
    void cloud_callback(const CloudMsg::ConstPtr& pc_msg);


    /*** specially designed for lady_and_cow dataset
         there is a sync problem in this dataset
    */
    bool msg_ready_;
    tf::Vector3 origin_;
    PoseMsg latest_pose_;
    ros::Subscriber pose_sub_;
    ros::Subscriber cloud_sub_;
    std::queue<PoseMsg> pose_queue_;
    std::queue<CloudMsg> cloud_queue_;
    void sync_pose_and_cloud_fiesta();
    bool sync_pose_and_cloud(PoseMsg &latest_pose, const CloudMsg &latest_cloud);
    void lady_cow_pose_callback(const PoseMsg::ConstPtr& pose);
    void lady_cow_cloud_callback(const CloudMsg::ConstPtr& cloud);


private: // occupancy map

    typedef pcl::PointCloud<pcl::PointXYZ> XYZCloud;
    typedef pcl::PointCloud<pcl::PointXYZI> XYZICloud;

    // occupancy map
    openvdb::FloatGrid::Ptr grid_logocc_;

    // major functions
    void set_voxel_size(openvdb::GridBase& grid, double vs);
    void update_occmap(FloatGrid::Ptr grid_map, const tf::Vector3& origin, XYZCloud::Ptr xyz);

    // visualization
    void grid_to_pcl(FloatGrid::Ptr grid, FloatGrid::ValueType thresh, XYZICloud::Ptr& pc_out);
    void grid_message(FloatGrid::Ptr &grid, CloudMsg &disp_msg);


private: // distance map

    typedef std::vector<openvdb::math::Coord> CoordList;
    typedef visualization_msgs::Marker VisMarker;

    // distance map
    int max_coor_dist_;
    int max_coor_sqdist_;
    EDTGrid::Ptr dist_map_;
    std::shared_ptr<DynamicVDBEDT> grid_distance_;

    // functions for updating distance map
    ros::Timer update_edt_timer_;
    void update_edtmap(const ros::TimerEvent &);
    inline std_msgs::ColorRGBA rainbow_color_map(double h);
    void get_slice_marker(VisMarker &marker, int marker_id,
                          double slice, double max_sqdist);


private: // pose correction for lady and cow dataset
    int occu_update_count_;
    int dist_update_count_;
    Eigen::Matrix4d cur_transform_;
    Eigen::Matrix4d ref_transform_;
    Eigen::Matrix4d T_B_C_, T_D_B_;
};

#endif
