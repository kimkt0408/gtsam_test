// Standard Libraries
#include <queue>
#include <string>
#include <vector>

// ROS Libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

// TF and Eigen Libraries
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// GTSAM Libraries
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/io.h>

#include <chrono>

using namespace std;
using namespace boost;
using namespace gtsam;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;


class GtsamOptimizer 
{
    private:
        // ROS handles and subscribers/publishers
        ros::NodeHandle nh_;
        ros::Subscriber originalOdomSub_, odomSub_, gpsSub_, vCloudSub_;
        ros::Publisher posePub_, trajectoryPub_, mapCloudPub_, gpsMapPub_, odomMapPub_;

        // GTSAM objects
        ISAM2 isam;
        NonlinearFactorGraph gtSAMgraph;
        Values initialEstimate;
        Symbol lastKey;

        // TF objects
        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        
        void originalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void vCloudCallback(const sensor_msgs::PointCloud2ConstPtr &vCloudMsg);
        void gpsCallback(const nav_msgs::Odometry::ConstPtr& gpsMsg);

        Values getCurrentEstimate();
        
        void graphUpdate();
        void addGPSFactor();
        void optimize();

        void publishCurrentPose();
        void publishCloudMap();
        
        gtsam::Pose3 trans2GtsamPose(std::array<float, 6>& transformIn);
        Eigen::Matrix4d convertToMatrix(double x, double y, double z, const Eigen::Quaterniond &q); 
        void getNormalizedRPY(const tf::Quaternion &q, double &roll, double &pitch, double &yaw);
        void transformMapCloud(tf2::Transform tf, CloudT::Ptr& inCloud, CloudT::Ptr& tfm_inCloud);
        
        void trajectoryVisualization(const std::vector<std::array<float, 6>> &vec_odom);
        void gpsMapFrameVisualization(const nav_msgs::Odometry& thisGPS);
        void odomMapFrameVisualization(const nav_msgs::Odometry& thisOdom);
        void mapCloudVisualization(const std::vector<CloudT::Ptr> vec_map_cloud);
        // visualization_msgs::MarkerArray mapCloudVisualization(const std::vector<CloudT::Ptr> vec_map_cloud);
        
        void printFactors();
        void addCloud();
        bool transformFrame(const string source_frame, const string target_frame, tf2::Transform& tf_sourceToTarget);

        string original_odom_topic_;
        string odom_topic_;
        string gps_topic_;
        string v_cloud_topic_;

        string v_lidar_frame_id_;
        string robot_frame_id_;
        string map_frame_id_;

        bool boolCloud_;
        bool debugMode_;

        float newOdomDist_;

        float largeGpsNoiseThreshold_;
        float smallGpsNoiseThreshold_;
        float gpsNoiseThreshold_; 

        float largePoseCovThreshold_;
        float smallPoseCovThreshold_;
        float poseCovThreshold_ = largePoseCovThreshold_;

        std::deque<nav_msgs::Odometry> gpsQueue_;
        std::queue<sensor_msgs::PointCloud2ConstPtr> vCloudQueue_; 

        std::array<float, 6> odom_;
        std::array<float, 6> prev_odom_; 
        std::array<float, 6> optimized_odom_; 
        std::array<float, 6> prev_optimized_odom_;

        std::vector<std::array<float, 6>> vec_odom_; 
        std::vector<CloudT::Ptr> vec_cloud_; 
        std::vector<CloudT::Ptr> vec_map_cloud_; 

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
        gtsam::noiseModel::Diagonal::shared_ptr odomNoise_;

        Eigen::MatrixXd poseCovariance_ = Eigen::MatrixXd::Zero(6, 6);

        geometry_msgs::PoseStamped poseStamped_;
        
        double timeLaserInfoCur_;
        double odomZInfoCur_;
        
    public:
        explicit GtsamOptimizer(ros::NodeHandle nh);
};


GtsamOptimizer::GtsamOptimizer(ros::NodeHandle nh) : nh_(nh) {
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    // (1) ROSTOPIC & FRAME_ID SETTING
    nh_.param<string>("original_odom_topic", original_odom_topic_, "odometry/filtered");
    // nh_.param<string>("odom_topic", odom_topic_,  "odometry/filtered");
    nh_.param<string>("odom_topic", odom_topic_, "pagslam/debug/ref_frame_pagslam_pose_");

    nh_.param<string>("gps_topic", gps_topic_, "odometry/gps_");
    nh_.param<string>("v_cloud_topic", v_cloud_topic_, "ns2/velodyne_points");
    
    nh_.param<string>("v_lidar_frame_id", v_lidar_frame_id_, "velodyne2");
    nh_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
    nh_.param<string>("map_frame_id", map_frame_id_, "map");

    // (2) PARAMETERS
    nh_.param<bool>("bool_cloud_publisher", boolCloud_, true);
    nh_.param<bool>("debug_mode", debugMode_, true);
    
    nh_.param<float>("new_odom_distance", newOdomDist_, 0.1);

    // nh_.param<float>("large_gps_noise_threshold", largeGpsNoiseThreshold_, 4e-1);  // 9e-1 / 4e-2 / 9e-2 / 2023-08-22-11-19-45: 1e-1 
    // nh_.param<float>("small_gps_noise_threshold", smallGpsNoiseThreshold_, 9e-2);  // 4e-2 / 4e-4 / 2023-09-14
    
    // nh_.param<float>("large_pose_covariance_threshold", largePoseCovThreshold_, 2e-1);  // 5e-2 / 2e-2
    // nh_.param<float>("small_pose_covariance_threshold", smallPoseCovThreshold_, 1e-4);  // 1e-2 / 4e-2 / 2e-2, 2023-09-14
    
    // gpsNoiseThreshold_ = smallGpsNoiseThreshold_; 
    // poseCovThreshold_ = largePoseCovThreshold_; 


    // (1) multiple/mid
    // nh_.param<float>("large_gps_noise_threshold", largeGpsNoiseThreshold_, 4e-1);  // 9e-1 / 4e-2 / 9e-2 / 2023-08-22-11-19-45: 1e-1 
    // nh_.param<float>("small_gps_noise_threshold", smallGpsNoiseThreshold_, 9e-2);  // 4e-2 / 4e-4 / 2023-09-14
    
    // nh_.param<float>("large_pose_covariance_threshold", largePoseCovThreshold_, 2e-1);  // 5e-2 / 2e-2
    // nh_.param<float>("small_pose_covariance_threshold", smallPoseCovThreshold_, 1e-4);  // 1e-2 / 4e-2 / 2e-2, 2023-09-14
    
    // gpsNoiseThreshold_ = largeGpsNoiseThreshold_; 
    // poseCovThreshold_ = smallPoseCovThreshold_; 

    // priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6).finished()); // rad,rad,rad,m, m, m
    // odomNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m


    // // (2) multiple/short
    // nh_.param<float>("large_gps_noise_threshold", largeGpsNoiseThreshold_, 4e-1);  // 9e-1 / 4e-2 / 9e-2 / 2023-08-22-11-19-45: 1e-1 
    // nh_.param<float>("small_gps_noise_threshold", smallGpsNoiseThreshold_, 9e-2);  // 4e-2 / 4e-4 / 2023-09-14
    
    // nh_.param<float>("large_pose_covariance_threshold", largePoseCovThreshold_, 2e-1);  // 5e-2 / 2e-2
    // nh_.param<float>("small_pose_covariance_threshold", smallPoseCovThreshold_, 1e-4);  // 1e-2 / 4e-2 / 2e-2, 2023-09-14
    
    // gpsNoiseThreshold_ = largeGpsNoiseThreshold_; 
    // poseCovThreshold_ = largePoseCovThreshold_; 

    // priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6).finished()); // rad,rad,rad,m, m, m
    // odomNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    
    // priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-6, 1e-6, 1e-6).finished()); // rad,rad,rad,m, m, m
    // odomNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    
    // ACRE2024
    // nh_.param<float>("large_gps_noise_threshold", largeGpsNoiseThreshold_, 8e-1);  // 9e-1 / 4e-2 / 9e-2 / 2023-08-22-11-19-45: 1e-1 
    nh_.param<float>("large_gps_noise_threshold", largeGpsNoiseThreshold_, 9e-1);  // 9e-1 / 4e-2 / 9e-2 / 2023-08-22-11-19-45: 1e-1 
    nh_.param<float>("small_gps_noise_threshold", smallGpsNoiseThreshold_, 4e-1);  // 4e-2 / 4e-4 / 2023-09-14 / 3e-1 / 2024-09-09
    
    nh_.param<float>("large_pose_covariance_threshold", largePoseCovThreshold_, 2e-1);  // 5e-2 / 2e-2
    nh_.param<float>("small_pose_covariance_threshold", smallPoseCovThreshold_, 1e-6);  // 1e-2 / 4e-2 / 2e-2, 2023-09-14
    
    // gpsNoiseThreshold_ = largeGpsNoiseThreshold_; 
    // poseCovThreshold_ = smallPoseCovThreshold_; 
    gpsNoiseThreshold_ = smallGpsNoiseThreshold_; 
    poseCovThreshold_ = largePoseCovThreshold_; 

    priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-3, 1e-3, 1e-3).finished()); // rad,rad,rad,m, m, m
    odomNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m

    // priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6).finished()); // rad,rad,rad,m, m, m
    // odomNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m


    // ROS subscribers

    originalOdomSub_ = nh.subscribe(original_odom_topic_, 100, &GtsamOptimizer::originalOdometryCallback, this);
    odomSub_ = nh.subscribe(odom_topic_, 100, &GtsamOptimizer::odometryCallback, this);
    gpsSub_ = nh.subscribe(gps_topic_, 100, &GtsamOptimizer::gpsCallback, this);
    vCloudSub_ = nh.subscribe(v_cloud_topic_, 100, &GtsamOptimizer::vCloudCallback, this);

    // ROS publishers
    posePub_ = nh.advertise<geometry_msgs::PoseStamped>("gtsam/optimized_pose_", 10);
    trajectoryPub_ = nh.advertise<visualization_msgs::MarkerArray>("gtsam/trajectory_", 10);
    mapCloudPub_ = nh.advertise<visualization_msgs::MarkerArray>("gtsam/mapCloud_", 10);

    // ROS publishers (convert rostopics to map frame)
    gpsMapPub_ = nh.advertise<nav_msgs::Odometry>("gtsam/gpsOnMap_", 10);
    odomMapPub_ = nh.advertise<nav_msgs::Odometry>("gtsam/odomOnMap_", 10);
    
    // Initialize iSAM with your desired parameters.
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = ISAM2(parameters);  

    for (int i = 0; i < 6; ++i){
        odom_[i] = 0;
    }

    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

    ROS_INFO("GTSAM initialized");
}


void GtsamOptimizer::originalOdometryCallback(const nav_msgs::Odometry::ConstPtr& originalOdomMsg) {
    odomMapFrameVisualization(*originalOdomMsg);
}  


void GtsamOptimizer::odometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {   
    timeLaserInfoCur_ = odomMsg->header.stamp.toSec();
    // odomZInfoCur_ = odomMsg->pose.pose.orientation.z;
    odomZInfoCur_ = odomMsg->pose.pose.position.z;

    tf::Quaternion q(
        odomMsg->pose.pose.orientation.x,
        odomMsg->pose.pose.orientation.y,
        odomMsg->pose.pose.orientation.z,
        odomMsg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    getNormalizedRPY(q, roll, pitch, yaw);

    // Initial setting for odom_ and prev_optimized_odom_
    if (vec_odom_.size() == 0){
        odom_[0] = roll;
        odom_[1] = pitch;
        odom_[2] = yaw;
        odom_[3] = odomMsg->pose.pose.position.x;
        odom_[4] = odomMsg->pose.pose.position.y;
        odom_[5] = odomMsg->pose.pose.position.z;

        prev_optimized_odom_ = odom_;
    }
    else{
        tf::Quaternion q_odom = tf::createQuaternionFromRPY(roll, pitch, yaw);
        tf::Quaternion q_prev_odom = tf::createQuaternionFromRPY(prev_odom_[0], prev_odom_[1], prev_odom_[2]);
        
        Eigen::Quaterniond eigen_q(q.w(), q.x(), q.y(), q.z());
        Eigen::Matrix4d T_odom = convertToMatrix(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z, eigen_q);

        Eigen::Quaterniond eigen_q_prev_odom(q_prev_odom.w(), q_prev_odom.x(), q_prev_odom.y(), q_prev_odom.z());
        Eigen::Matrix4d T_prev_odom = convertToMatrix(prev_odom_[3], prev_odom_[4], prev_odom_[5], eigen_q_prev_odom);

        // Compute the relative transformation matrix
        Eigen::Matrix4d T_delta = T_prev_odom.inverse() * T_odom;

        // Compute combined matrix
        tf::Quaternion q_prev_optimized_odom = tf::createQuaternionFromRPY(prev_optimized_odom_[0], prev_optimized_odom_[1], prev_optimized_odom_[2]);

        Eigen::Quaterniond eigen_q_prev_optimized_odom(q_prev_optimized_odom.w(), q_prev_optimized_odom.x(), q_prev_optimized_odom.y(), q_prev_optimized_odom.z());
        Eigen::Matrix4d T_prev_optimized = convertToMatrix(prev_optimized_odom_[3], prev_optimized_odom_[4], prev_optimized_odom_[5], eigen_q_prev_optimized_odom);
        Eigen::Matrix4d T_combined = T_prev_optimized * T_delta;
        
        if (debugMode_){
            cout << "!!!" << prev_optimized_odom_[0] << " " << prev_optimized_odom_[1] << " " << prev_optimized_odom_[2] << " " << 
            prev_optimized_odom_[3] << " " << prev_optimized_odom_[4] << " " << prev_optimized_odom_[5] << " " << endl;  
        }

        // Extract the 3x3 rotation matrix from T_combined
        Eigen::Matrix3d R_combined = T_combined.block<3,3>(0,0);

        // Convert this rotation matrix to RPY angles using tf
        tf::Matrix3x3 tf_rot(
            R_combined(0,0), R_combined(0,1), R_combined(0,2),
            R_combined(1,0), R_combined(1,1), R_combined(1,2),
            R_combined(2,0), R_combined(2,1), R_combined(2,2)
        );

        double curr_roll, curr_pitch, curr_yaw;
        tf_rot.getRPY(curr_roll, curr_pitch, curr_yaw);

        // Extract the translation vector
        double curr_x = T_combined(0,3);
        double curr_y = T_combined(1,3);
        double curr_z = T_combined(2,3);

        prev_optimized_odom_[0] = curr_roll;
        prev_optimized_odom_[1] = curr_pitch;
        prev_optimized_odom_[2] = curr_yaw;
        prev_optimized_odom_[3] = curr_x;
        prev_optimized_odom_[4] = curr_y;
        prev_optimized_odom_[5] = curr_z; 
    }

    prev_odom_[0] = roll;
    prev_odom_[1] = pitch;
    prev_odom_[2] = yaw;
    prev_odom_[3] = odomMsg->pose.pose.position.x;
    prev_odom_[4] = odomMsg->pose.pose.position.y;
    prev_odom_[5] = odomMsg->pose.pose.position.z;

    float dist;

    if (vec_odom_.size() == 0){
        dist = sqrt(pow(odom_[3],2) + pow(odom_[4],2) + pow(odom_[5],2));            
    }
    else{
        dist = sqrt(pow(prev_optimized_odom_[3]-vec_odom_.back()[3],2) + pow(prev_optimized_odom_[4]-vec_odom_.back()[4],2) + pow(prev_optimized_odom_[5]-vec_odom_.back()[5],2));   
        // dist = sqrt((odom_[3]-vec_odom_.back()[3])**2 + (odom_[4]-vec_odom_.back()[4])**2 + (odom_[5]-vec_odom_.back()[5])**2);                
    }

    if (debugMode_){
        cout << "*******************Odom Proceeding*******************" << dist << endl;
    }

    if (dist > newOdomDist_){
        if (debugMode_){
            cout << "*******************Odom Factor Added*******************" << endl;
        }
        odom_ = prev_optimized_odom_;
        vec_odom_.push_back(odom_);
        
        if (boolCloud_){
            addCloud();
        }
        
        graphUpdate();
        addGPSFactor();
        optimize();
    }
}


void GtsamOptimizer::vCloudCallback(const sensor_msgs::PointCloud2ConstPtr &vCloudMsg){
    vCloudQueue_.push(vCloudMsg);

}


void GtsamOptimizer::gpsCallback(const nav_msgs::Odometry::ConstPtr& gpsMsg){
    gpsQueue_.push_back(*gpsMsg);
    gpsMapFrameVisualization(*gpsMsg);
}


Values GtsamOptimizer::getCurrentEstimate() {
    return isam.calculateEstimate();
}


void GtsamOptimizer::graphUpdate(){
    std::array<float, 6> pose = vec_odom_.back();
    int key;
    if (vec_odom_.size() == 1){
        key = 0;
        
        gtSAMgraph.add(PriorFactor<Pose3>(key, trans2GtsamPose(pose), priorPoseNoise_));
        initialEstimate.insert(0, trans2GtsamPose(pose));
    }
    else{
        key = vec_odom_.size()-1;

        std::array<float, 6> prev_pose = vec_odom_[key-1];
        
        gtsam::Pose3 poseFrom = trans2GtsamPose(prev_pose);
        gtsam::Pose3 poseTo   = trans2GtsamPose(pose);

        gtSAMgraph.add(BetweenFactor<Pose3>(key-1, key, poseFrom.between(poseTo), odomNoise_));
        initialEstimate.insert(key, poseTo);
    }
}


void GtsamOptimizer::addGPSFactor() {    
    if (gpsQueue_.empty())
        return;
    
    cout << "Pose covariance before gtsam:" <<poseCovariance_(3,3) << " " << poseCovariance_(4,4) << "====" << poseCovThreshold_ << endl;
    
    if (poseCovariance_(3,3) < poseCovThreshold_ && poseCovariance_(4,4) < poseCovThreshold_){            
        return;
    }
    
    while (!gpsQueue_.empty()) {
        nav_msgs::Odometry thisGPS = gpsQueue_.front();
        double gpsTime = thisGPS.header.stamp.toSec();
        
        if (gpsTime < timeLaserInfoCur_ - 0.2) {
            gpsQueue_.pop_front();
            continue;
        }
        
        // Time conditions to break from loop
        if (gpsTime > timeLaserInfoCur_ + 0.2)
            break;

        gpsQueue_.pop_front();

        // GPS too noisy, skip
        float noise_x = thisGPS.pose.covariance[0];
        float noise_y = thisGPS.pose.covariance[7];
        float noise_z = thisGPS.pose.covariance[14];

        cout << "GPS Noise: " << noise_x << " " << noise_y << " " << noise_z << " " << (noise_x > gpsNoiseThreshold_ || noise_y > gpsNoiseThreshold_) << endl;

        if (noise_x > gpsNoiseThreshold_ || noise_y > gpsNoiseThreshold_){
        // if (noise_x > gpsNoiseThreshold_ && noise_y > gpsNoiseThreshold_){
            cout << "Too large GPS Noise!!!" << endl;
            continue;
        }
            

        float gps_x = thisGPS.pose.pose.position.x;
        float gps_y = thisGPS.pose.pose.position.y;
        
        // float gps_z = thisGPS.pose.pose.position.z;  // (1) If you use z value of gps
        float gps_z = odomZInfoCur_;                    // (2) If you use z value of odom


        // GPS not properly initialized (0,0,0)
        if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6){
            continue;
        }
        
        gtsam::Vector Vector3(3);
        Vector3 << min(noise_x, 1.0f), min(noise_y, 1.0f), min(noise_z, 1.0f);
        noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(vec_odom_.size()-1, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
        gtSAMgraph.add(gps_factor);
        
        cout << "-----------GPS Factor Added: " << gps_x << " " << gps_y << " " << gps_z << " " << noise_x << " " << noise_y << " " << noise_z << " " << endl;

        break;
        // }
    }
}


void GtsamOptimizer::optimize() {
    isam.update(gtSAMgraph, initialEstimate);

    publishCurrentPose();

    if (boolCloud_){
        vec_map_cloud_.clear();

        publishCloudMap();
    }

    trajectoryVisualization(vec_odom_);
    cout << "-----------Trajectory Optimized-----------" << endl;

    gtSAMgraph.resize(0);
    initialEstimate.clear();
}


void GtsamOptimizer::publishCurrentPose() {
    Values currentEstimates = getCurrentEstimate();

    if (currentEstimates.empty())
        return;

    vec_odom_.clear();
    gtsam::Key lastKey;
    
    for(const gtsam::Values::ConstKeyValuePair& key_value: currentEstimates) {
        gtsam::Key key = key_value.key;
        
        Pose3 currentPose;
        try {
            currentPose = currentEstimates.at<Pose3>(key);
        } catch (const std::exception& e) {
            std::cerr << "Error retrieving pose for key: " << key << ". Error: " << e.what() << std::endl;
            continue;
        }

        gtsam::Rot3 currRotation = currentPose.rotation();
        gtsam::Point3 currTranslation = currentPose.translation();

        optimized_odom_[0] = currRotation.roll();
        optimized_odom_[1] = currRotation.pitch();
        optimized_odom_[2] = currRotation.yaw();
        optimized_odom_[3] = currTranslation.x();
        optimized_odom_[4] = currTranslation.y();
        // optimized_odom_[5] = currTranslation.z();
        optimized_odom_[5] = 0.0;

        vec_odom_.push_back(optimized_odom_);
        lastKey = key;
    }

    // poseStamped_.header.stamp = ros::Time::now();
    poseStamped_.header.stamp = ros::Time(0);
    poseStamped_.header.frame_id = map_frame_id_;

    if (currentEstimates.exists(lastKey)) {            
        if (debugMode_){
            cout << "???" << optimized_odom_[0] << " " << optimized_odom_[1] << " " << optimized_odom_[2] << " " << 
            optimized_odom_[3] << " " << optimized_odom_[4] << " " << optimized_odom_[5] << " " << endl;  
        }
        
        prev_optimized_odom_ = optimized_odom_;
        Pose3 lastPose = currentEstimates.at<Pose3>(lastKey);

        gtsam::Quaternion quat = lastPose.rotation().toQuaternion();
        tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());

        double roll, pitch, yaw;
        getNormalizedRPY(q, roll, pitch, yaw);

        poseStamped_.pose.position.x = lastPose.translation().x();
        poseStamped_.pose.position.y = lastPose.translation().y();
        poseStamped_.pose.position.z = lastPose.translation().z();
        poseStamped_.pose.orientation.w = lastPose.rotation().toQuaternion().w();
        poseStamped_.pose.orientation.x = lastPose.rotation().toQuaternion().x();
        poseStamped_.pose.orientation.y = lastPose.rotation().toQuaternion().y();
        poseStamped_.pose.orientation.z = lastPose.rotation().toQuaternion().z();

        if (debugMode_){
            cout << "*******************Odom Factor Visualized*******************" << endl;
        }
    }

    poseCovariance_ = isam.marginalCovariance(currentEstimates.size()-1);
    cout << "Pose covariance after gtsam:" <<poseCovariance_(3,3) << " " << poseCovariance_(4,4) << "====" << poseCovThreshold_ << endl;
    
    posePub_.publish(poseStamped_);
}


void GtsamOptimizer::publishCloudMap() {
    for (int i = 0; i < vec_odom_.size(); i++){
        auto pose = vec_odom_[i];
        auto v_cloud = vec_cloud_[i];
        
        if (v_cloud->points.size() == 0){
            continue;
        }

        CloudT::Ptr tfm_v_cloud(new CloudT());

        // Create a quaternion directly from roll, pitch, yaw
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(pose[0], pose[1], pose[2]);

        // Create the tf2::Vector3 object directly from translation data
        tf2::Vector3 tf2_translation_vector(pose[3], pose[4], pose[5]);

        // Construct the tf2::Transform object using the constructor
        tf2::Transform tf2_transform(tf2_quaternion, tf2_translation_vector);

        // Use the transform
        transformMapCloud(tf2_transform, v_cloud, tfm_v_cloud);

        vec_map_cloud_.push_back(tfm_v_cloud);
    }
    
    // Comment if visualization is not required
    mapCloudVisualization(vec_map_cloud_);
}


gtsam::Pose3 GtsamOptimizer::trans2GtsamPose(std::array<float, 6>& transformIn) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}


// Conversion of translation vector and quaternion to a 4x4 matrix
Eigen::Matrix4d GtsamOptimizer::convertToMatrix(double x, double y, double z, const Eigen::Quaterniond &q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Convert quaternion to 3x3 rotation matrix
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Populate the 4x4 matrix
    T.block(0, 0, 3, 3) = R;          // Top-left 3x3 is the rotation matrix
    T.block(0, 3, 3, 1) << x, y, z;   // Top-right column is the translation

    return T;
}


void GtsamOptimizer::getNormalizedRPY(const tf::Quaternion &q, double &roll, double &pitch, double &yaw) {
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    
    // Normalize angles to desired ranges
    while (roll > M_PI) roll -= 2.0 * M_PI;
    while (roll <= -M_PI) roll += 2.0 * M_PI;

    while (pitch > M_PI) pitch -= 2.0 * M_PI;
    while (pitch <= -M_PI) pitch += 2.0 * M_PI;

    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw <= -M_PI) yaw += 2.0 * M_PI;
}


void GtsamOptimizer::transformMapCloud(tf2::Transform tf, CloudT::Ptr& inCloud, CloudT::Ptr& tfm_inCloud){
    geometry_msgs::Transform transform;
    tf2::convert(tf, transform);

    pcl_ros::transformPointCloud(*inCloud, *tfm_inCloud, transform);
    tfm_inCloud->header.frame_id = map_frame_id_;
}


void GtsamOptimizer::trajectoryVisualization(const std::vector<std::array<float, 6>> &vec_odom){
    visualization_msgs::MarkerArray tMarkerArray;
    visualization_msgs::Marker points, line_strip;
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 1000;
    line_strip.id = 1001;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

    line_strip.scale.x = 0.01;

    points.header.frame_id = line_strip.header.frame_id = map_frame_id_;
    points.color.r = 0.0;
    points.color.g = 1.0;
    points.color.b = 0.0;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    for (auto odom : vec_odom)
    {
        // Between odom line strips
        geometry_msgs::Point pt;
        pt.x = odom[3];
        pt.y = odom[4];
        pt.z = odom[5];
        points.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    tMarkerArray.markers.push_back(points);
    tMarkerArray.markers.push_back(line_strip);

    trajectoryPub_.publish(tMarkerArray);
    // return tMarkerArray;
}


void GtsamOptimizer::gpsMapFrameVisualization(const nav_msgs::Odometry& thisGPS){
    nav_msgs::Odometry msg;

    msg.header = thisGPS.header;
    msg.header.frame_id = map_frame_id_;

    msg.pose = thisGPS.pose;
    msg.twist = thisGPS.twist;

    gpsMapPub_.publish(msg);
}


void GtsamOptimizer::odomMapFrameVisualization(const nav_msgs::Odometry& thisOdom){
    nav_msgs::Odometry msg;

    msg.header = thisOdom.header;
    msg.header.frame_id = map_frame_id_;

    msg.pose = thisOdom.pose;
    msg.twist = thisOdom.twist;

    odomMapPub_.publish(msg);
}


void GtsamOptimizer::mapCloudVisualization(const std::vector<CloudT::Ptr> vec_map_cloud){
    visualization_msgs::MarkerArray marker_array;
    // Loop over each point cloud and create a marker for it
    int id = 0;

    for (const auto& map_cloud : vec_map_cloud){
        float num_r = static_cast<float>(rand()) / RAND_MAX;
        float num_g = static_cast<float>(rand()) / RAND_MAX;
        float num_b = static_cast<float>(rand()) / RAND_MAX;
        
        // Create a marker for the point cloud
        visualization_msgs::Marker marker;
        marker.header.frame_id = map_cloud->header.frame_id;
        // marker.header.frame_id = map_frame_id__;

        ros::Time stamp = pcl_conversions::fromPCL(map_cloud->header).stamp;
        marker.header.stamp = stamp;

        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "point_cloud";
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.a = 1.0;
        marker.color.r = num_r;
        marker.color.g = num_g;
        marker.color.b = num_b;

        marker.pose.orientation.w = 1.0;  // Identity Quaternion
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;


        for (const auto& point : map_cloud->points) {
        geometry_msgs::Point p;

        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
        }

        // Add the marker to the array
        marker_array.markers.push_back(marker);
    }

    mapCloudPub_.publish(marker_array);
}


void GtsamOptimizer::printFactors() {
    for (const auto& factor : gtSAMgraph) {
        factor->print();
    }
}


void GtsamOptimizer::addCloud() {
    if (vCloudQueue_.empty()){

        CloudT::Ptr tfm_v_cloud(new CloudT());
        tfm_v_cloud->header.frame_id = robot_frame_id_;
        vec_cloud_.push_back(tfm_v_cloud);
    } 

    tf2::Transform tf_vCloudSourceToTarget;
    bool bool_vCloudTransformFrame = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_vCloudSourceToTarget);

    while (!vCloudQueue_.empty()){
        double msgTime = vCloudQueue_.front()->header.stamp.toSec();
        
        if (msgTime < timeLaserInfoCur_ - 0.2) {
            vCloudQueue_.pop();
            continue;
        }
        
        CloudT::Ptr tfm_v_cloud(new CloudT());
        tfm_v_cloud->header.frame_id = robot_frame_id_;

        if (msgTime > timeLaserInfoCur_ + 0.2) {
            vec_cloud_.push_back(tfm_v_cloud);
            break;
        }

        // Transform the point cloud from ROS msg format to PCL format
        CloudT::Ptr v_cloud(new CloudT());
        pcl::fromROSMsg(*vCloudQueue_.front(), *v_cloud);
        vCloudQueue_.pop();

        // tf2::Transform tf_vCloudSourceToTarget;
        // bool bool_vCloudTransformFrame = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_vCloudSourceToTarget);
        
        if (!bool_vCloudTransformFrame) {
            vec_cloud_.push_back(tfm_v_cloud);
            break;
        }

        // Convert tf2 transform to geometry_msgs::Transform
        geometry_msgs::Transform transform;
        tf2::convert(tf_vCloudSourceToTarget, transform);

        pcl_ros::transformPointCloud(*v_cloud, *tfm_v_cloud, transform);

        vec_cloud_.push_back(tfm_v_cloud);
        break;
    }
}


bool GtsamOptimizer::transformFrame(const string source_frame, const string target_frame, tf2::Transform& tf_sourceToTarget){
    // Get the transform from the source frame to the target frame
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }
    // Convert the transform to an Eigen transform
    tf2::fromMsg(transformStamped.transform, tf_sourceToTarget);
    
    return true;
}


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "gtsam_optimizer");
//     ros::NodeHandle nh;

//     double total_latency = 0.0;
//     int count = 0;

//     auto input_time = std::chrono::high_resolution_clock::now();

//     GtsamOptimizer optimizer(nh);

//     auto output_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> latency = output_time - input_time;
//     std::cout << "2 System latency: " << latency.count() << " ms\n";

//     total_latency += latency.count();
//     count++;
    
//     ros::spin();

//     double avg_latency = total_latency / count;
//     std::cout << "2 Average system latency: " << avg_latency << " " << count <<" ms\n";

//     return 0;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "gtsam_optimizer");
    ros::NodeHandle nh;

    double total_latency = 0.0;
    int count = 0;

    GtsamOptimizer optimizer(nh);

    ros::Rate rate(10); // 10 Hz, or adjust as needed
    while (ros::ok()) {
        auto input_time = std::chrono::high_resolution_clock::now();

        // Perform operations you want to measure here.
        // For instance, if you want to measure the time taken by a specific method in GtsamOptimizer, call it here.
        
        ros::spinOnce(); // Handle callbacks

        auto output_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> latency = output_time - input_time;
        total_latency += latency.count();
        count++;

        rate.sleep(); // Maintain loop rate
    }

    // double avg_latency = total_latency / count;
    // std::cout << "2 Average system latency: " << avg_latency << " " << count << " ms\n";

    return 0;
}
