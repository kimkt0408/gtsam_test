#include <ros/ros.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>



#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

#include <visualization_msgs/MarkerArray.h>
#include <gtsam/nonlinear/Marginals.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/io.h>

#include <std_msgs/Header.h>
#include <queue>

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

using namespace gtsam;

using namespace std;
using namespace boost;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;


class GtsamOptimizer {
private:
    ISAM2 isam;
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Symbol lastKey;
    ros::Subscriber originalOdomSub, odomSub, imuSub, gpsSub, vCloudSub;
    ros::Publisher posePub, trajectoryPub, mapCloudPub, gpsMapPub, odomMapPub; 


    string original_odom_topic = "odometry/filtered";
    string odom_topic = "pagslam/debug/ref_frame_pagslam_pose";
    string imu_topic = "imu/data";
    string gps_topic = "odometry/gps";
    string lidar_topic = "ns2/velodyne_points";

    string v_lidar_frame_id = "velodyne2";
    string robot_frame_id = "base_link";
    string map_frame_id = "map";

    std::array<float, 6> odom; // Use std::array instead of raw array
    std::array<float, 6> prev_odom; // Use std::array instead of raw array

    std::array<float, 6> optimized_odom; 
    std::array<float, 6> prev_optimized_odom;
    std::vector<std::array<float, 6>> vec_odom; // Change definition to store arrays
    std::vector<CloudT::Ptr> vec_cloud; // Change definition to store arrays
    std::vector<CloudT::Ptr> vec_map_cloud; // Change definition to store arrays

    // std::vector<CloudT::Ptr> mapCloud_;

    double timeLaserInfoCur;

    bool initialPose = true;
    bool bool_cloud = true;
    bool debugMode = false;

    int n_odom = 0;
    int n_gps = 0;

    float newOdomDist = 0.1;

    float large_noise = 1e-1;
    float small_noise = 4e-4; // 2023-09-14
    float gps_noise_threshold = large_noise; 


    float largeCovThreshold = 5e-2;
    float smallCovThreshold = 2e-2; // 2023-09-14
    float poseCovThreshold = largeCovThreshold;

    std::deque<nav_msgs::Odometry> gpsQueue;
    // std::deque<sensor_msgs::PointCloud2> 
    std::queue<sensor_msgs::PointCloud2ConstPtr> vCloudQueue; 
    // std::deque<CloudT::Ptr> vCloudQueue;

    
    // Transform
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool bool_vCloudTransformFrame;

    geometry_msgs::PoseStamped poseStamped;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;

    Eigen::MatrixXd poseCovariance = Eigen::MatrixXd::Zero(6, 6);

public:
    GtsamOptimizer(ros::NodeHandle& nh) {
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        // Initialize iSAM with your desired parameters.
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = ISAM2(parameters);
    
        // ROS subscribers
        
        originalOdomSub = nh.subscribe(original_odom_topic, 100, &GtsamOptimizer::originalOdometryCallback, this);
        odomSub = nh.subscribe(odom_topic, 100, &GtsamOptimizer::odometryCallback, this);
        imuSub = nh.subscribe(imu_topic, 100, &GtsamOptimizer::imuCallback, this);
        gpsSub = nh.subscribe(gps_topic, 100, &GtsamOptimizer::gpsCallback, this);

        vCloudSub = nh.subscribe(lidar_topic, 100, &GtsamOptimizer::vCloudCallback, this);

        posePub = nh.advertise<geometry_msgs::PoseStamped>("gtsam/optimized_pose", 10);
        trajectoryPub = nh.advertise<visualization_msgs::MarkerArray>("gtsam/trajectory", 10);
        mapCloudPub = nh.advertise<visualization_msgs::MarkerArray>("gtsam/mapCloud", 10);

        // Initialize publisher
        gpsMapPub = nh.advertise<nav_msgs::Odometry>("gtsam/gpsOnMap", 10);
        odomMapPub = nh.advertise<nav_msgs::Odometry>("gtsam/odomOnMap", 10);

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-6, 1e-6, 1e-6).finished()); // rad,rad,rad,m, m, m
        odometryNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m


        for (int i = 0; i < 6; ++i){
            odom[i] = 0;
        }
        
    }

    gtsam::Pose3 trans2gtsamPose(std::array<float, 6>& transformIn) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    // // Conversion of translation vector and quaternion to a 4x4 matrix
    // Eigen::Matrix4d convertToMatrix(double x, double y, double z, const tf::Quaternion &q) {
    //     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    //     tf::Matrix3x3 rot(q);
    //     for (int i = 0; i < 3; i++) {
    //         for (int j = 0; j < 3; j++) {
    //             T(i, j) = rot[i][j];
    //         }
    //     }
    //     T(0, 3) = x;
    //     T(1, 3) = y;
    //     T(2, 3) = z;
    //     return T;
    // }


// Conversion of translation vector and quaternion to a 4x4 matrix
    Eigen::Matrix4d convertToMatrix(double x, double y, double z, const Eigen::Quaterniond &q) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // Convert quaternion to 3x3 rotation matrix
        Eigen::Matrix3d R = q.toRotationMatrix();

        // Populate the 4x4 matrix
        T.block(0, 0, 3, 3) = R;          // Top-left 3x3 is the rotation matrix
        T.block(0, 3, 3, 1) << x, y, z;   // Top-right column is the translation

        return T;
    }


    void getNormalizedRPY(const tf::Quaternion &q, double &roll, double &pitch, double &yaw, double roll_offset = 0, double pitch_offset = 0, double yaw_offset = 0) {
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        
        // Add offsets if any (this could be useful in case you have a specific reference frame or orientation)
        roll += roll_offset;
        pitch += pitch_offset;
        yaw += yaw_offset;

        // Normalize angles to desired ranges
        while (roll > M_PI) roll -= 2.0 * M_PI;
        while (roll <= -M_PI) roll += 2.0 * M_PI;

        while (pitch > M_PI) pitch -= 2.0 * M_PI;
        while (pitch <= -M_PI) pitch += 2.0 * M_PI;

        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw <= -M_PI) yaw += 2.0 * M_PI;
    }

    // // Convert a 3x3 rotation matrix to a tf::Quaternion
    // tf::Quaternion convertToQuaternion(const Eigen::Matrix3d &R) {
    //     Eigen::Quaterniond eigen_quat(R);
    //     return tf::Quaternion(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    // }

    // ...


    void originalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odomMapFrameVisualization(*msg);
    }   

    // void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {  
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {   
        timeLaserInfoCur = msg->header.stamp.toSec();

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        getNormalizedRPY(q, roll, pitch, yaw);
        // m.getRPY(roll, pitch, yaw);

        if (vec_odom.size() == 0){
            odom[0] = roll;
            odom[1] = pitch;
            odom[2] = yaw;
            odom[3] = msg->pose.pose.position.x;
            odom[4] = msg->pose.pose.position.y;
            odom[5] = msg->pose.pose.position.z;

            // poseStamped.pose = msg->pose.pose;

            // cout << odom[0] << " " << odom[1] << " " << odom[2] << endl;
            // posePub.publish(poseStamped);

            prev_optimized_odom = odom;
            // dist = sqrt(pow(odom[3],2) + pow(odom[4],2) + pow(odom[5],2));    
        }
        else{
            double delta_x, delta_y, delta_z;
        
            delta_x = msg->pose.pose.position.x - prev_odom[3];
            delta_y = msg->pose.pose.position.y - prev_odom[4];
            delta_z = msg->pose.pose.position.z - prev_odom[5];


            // cout << "!!!" << msg->pose.pose.orientation.w << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << endl;
                

            // tf::Quaternion q_odom = tf::createQuaternionFromRPY(roll, pitch, yaw);
            tf::Quaternion q_odom = tf::createQuaternionFromRPY(roll, pitch, yaw);
            tf::Quaternion q_prev_odom = tf::createQuaternionFromRPY(prev_odom[0], prev_odom[1], prev_odom[2]);
            
            // tf::Quaternion q_delta = q_prev_odom.inverse() * q_odom;
            // tf::Quaternion q_delta = q_prev_odom.inverse() * q;
            
            Eigen::Quaterniond eigen_q(q.w(), q.x(), q.y(), q.z());
            Eigen::Matrix4d T_odom = convertToMatrix(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, eigen_q);

            Eigen::Quaterniond eigen_q_prev_odom(q_prev_odom.w(), q_prev_odom.x(), q_prev_odom.y(), q_prev_odom.z());
            Eigen::Matrix4d T_prev_odom = convertToMatrix(prev_odom[3], prev_odom[4], prev_odom[5], eigen_q_prev_odom);

            // Compute the relative transformation matrix
            Eigen::Matrix4d T_delta = T_prev_odom.inverse() * T_odom;

            // cout << T_delta << endl;
            // Eigen::Matrix4d T_delta = convertToMatrix(delta_x, delta_y, delta_z, q_delta);

            // // Split the T_delta into rotation and translation matrices
            // Eigen::Matrix4d T_delta_rot = Eigen::Matrix4d::Identity();
            // T_delta_rot.block<3,3>(0,0) = T_delta.block<3,3>(0,0);

            // Eigen::Matrix4d delta_translation = Eigen::Matrix4d::Identity();
            // delta_translation(0, 3) = T_delta(0, 3);
            // delta_translation(1, 3) = T_delta(1, 3);
            // delta_translation(2, 3) = T_delta(2, 3);

            // Compute combined matrix
            tf::Quaternion q_prev_optimized_odom = tf::createQuaternionFromRPY(prev_optimized_odom[0], prev_optimized_odom[1], prev_optimized_odom[2]);

            Eigen::Quaterniond eigen_q_prev_optimized_odom(q_prev_optimized_odom.w(), q_prev_optimized_odom.x(), q_prev_optimized_odom.y(), q_prev_optimized_odom.z());
            Eigen::Matrix4d T_prev_optimized = convertToMatrix(prev_optimized_odom[3], prev_optimized_odom[4], prev_optimized_odom[5], eigen_q_prev_optimized_odom);
            Eigen::Matrix4d T_combined = T_prev_optimized * T_delta;

            // Eigen::Matrix4d T_combined = T_prev_optimized * T_delta_rot * delta_translation;

            // // tf::Quaternion q_delta = q_prev_odom.inverse() * q_odom;
            // tf::Quaternion q_delta = q_prev_odom.inverse() * q;
            
            if (debugMode){
                cout << "!!!" << prev_optimized_odom[0] << " " << prev_optimized_odom[1] << " " << prev_optimized_odom[2] << " " << 
                prev_optimized_odom[3] << " " << prev_optimized_odom[4] << " " << prev_optimized_odom[5] << " " << endl;  
            }

            // tf::Quaternion q_prev_optimized_odom = tf::createQuaternionFromRPY(prev_optimized_odom[0], prev_optimized_odom[1], prev_optimized_odom[2]);
            // // tf::Quaternion q_combined = q_delta * q_prev_optimized_odom;
            // tf::Quaternion q_combined = q_prev_optimized_odom * q_delta;

            // // tf::Matrix3x3 m_delta(q_delta);
            // // double delta_roll, delta_pitch, delta_yaw;
            // // m_delta.getRPY(delta_roll, delta_pitch, delta_yaw);
            
            // tf::Matrix3x3 m_curr_odom(q_combined);
            // double curr_roll, curr_pitch, curr_yaw;
            // m_curr_odom.getRPY(curr_roll, curr_pitch, curr_yaw);

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
            double x = T_combined(0,3);
            double y = T_combined(1,3);
            double z = T_combined(2,3);

            prev_optimized_odom[0] = curr_roll;
            prev_optimized_odom[1] = curr_pitch;
            prev_optimized_odom[2] = curr_yaw;
            prev_optimized_odom[3] = T_combined(0,3);
            prev_optimized_odom[4] = T_combined(1,3);
            prev_optimized_odom[5] = T_combined(2,3);

            // odom[0] = prev_odom[0] + delta_roll;
            // odom[1] = prev_odom[1] + delta_pitch;
            // odom[2] = prev_odom[2] + delta_yaw;
            // odom[3] = prev_odom[3] + delta_x;
            // odom[4] = prev_odom[4] + delta_y;
            // odom[5] = prev_odom[5] + delta_z;

            // dist = sqrt(pow(odom[3]-vec_odom.back()[3],2) + pow(odom[4]-vec_odom.back()[4],2) + pow(odom[5]-vec_odom.back()[5],2));   
        }

        prev_odom[0] = roll;
        prev_odom[1] = pitch;
        prev_odom[2] = yaw;
        prev_odom[3] = msg->pose.pose.position.x;
        prev_odom[4] = msg->pose.pose.position.y;
        prev_odom[5] = msg->pose.pose.position.z;


        float dist = 0;

        if (vec_odom.size() == 0){
            dist = sqrt(pow(odom[3],2) + pow(odom[4],2) + pow(odom[5],2));            
        }
        else{
            dist = sqrt(pow(prev_optimized_odom[3]-vec_odom.back()[3],2) + pow(prev_optimized_odom[4]-vec_odom.back()[4],2) + pow(prev_optimized_odom[5]-vec_odom.back()[5],2));   
            // dist = sqrt((odom[3]-vec_odom.back()[3])**2 + (odom[4]-vec_odom.back()[4])**2 + (odom[5]-vec_odom.back()[5])**2);                
        }

        if (debugMode){
            cout << "*******************Odom Proceeding*******************" << dist << endl;
        }

        if (dist > newOdomDist){
            if (debugMode){
                cout << "*******************Odom Factor Added*******************" << endl;
            }
            odom = prev_optimized_odom;
            vec_odom.push_back(odom);
            // cout << vec_odom.size() << endl;

            if (bool_cloud){
                addCloud();
            }
            
            // cout << vec_odom.size() << " " << vec_cloud.size() << endl;

            // cout << "SIZE: " << vec_odom.size() << endl;

            graphUpdate();

            addGPSFactor();

            optimize();
        }

        n_odom++;
    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // // Extract the necessary IMU data
        // // Here, I'm simplifying the process. Typically, you would integrate IMU data over time.
        // PreintegratedImuMeasurements imuData;
        // // ... fill in imuData from msg ...
        // NoiseModel::shared_ptr noise = NoiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
        // addIMU(imuData, noise);
    }


    void gpsCallback(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
        gpsMapFrameVisualization(*gpsMsg);
    }

    void addGPSFactor() {    
        if (gpsQueue.empty())
            return;
        
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold){
            // cout << "No need to add GPS factor, yet" << endl; 
            
            return;
        }
        
        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];

                // if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                if (noise_x > gps_noise_threshold || noise_y > gps_noise_threshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;
                
                gtsam::Vector Vector3(3);
                Vector3 << min(noise_x, 1.0f), min(noise_y, 1.0f), min(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(vec_odom.size()-1, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);
                
                n_gps++;
                // if (debugMode){
                cout << "-----------------------GPS Factor Added-----------------------" <<  noise_x << " " << noise_y << " " << noise_z << " " << endl;
                // }
                break;
                // cout << "GPS: " << gps_x << " " << gps_y << " " << gps_z << "**************" << endl;
                // printFactors();

                // nav_msgs::Odometry thisGPS = gpsQueue.front();
                // gpsQueue.pop();

                // // GPS too noisy, skip
                // float noise_x = thisGPS.pose.covariance[0];
                // float noise_y = thisGPS.pose.covariance[7];
                // float noise_z = thisGPS.pose.covariance[14];
                // if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                //     continue;

                // float gps_x = thisGPS.pose.pose.position.x;
                // float gps_y = thisGPS.pose.pose.position.y;
                // float gps_z = thisGPS.pose.pose.position.z;
                // if (!useGpsElevation)
                // {
                //     gps_z = transformTobeMapped[5];
                //     noise_z = 0.01;
                // }

                // // GPS not properly initialized (0,0,0)
                // if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                //     continue;

                // // Add GPS every a few meters
                // PointType curGPSPoint;
                // curGPSPoint.x = gps_x;
                // curGPSPoint.y = gps_y;
                // curGPSPoint.z = gps_z;
                // if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                //     continue;
                // else
                //     lastGPSPoint = curGPSPoint;

                // gtsam::Vector Vector3(3);
                // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                // noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                // gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                // gtSAMgraph.add(gps_factor);

                // aLoopIsClosed = true;
                // break;
            }
        }
    }

    void graphUpdate() {

        // for (size_t i = 0; i < vec_odom.size(); i++) {
        std::array<float, 6> pose = vec_odom.back();
        int key;
        if (vec_odom.size() == 1){
            key = 0;
            
            // noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            
            gtSAMgraph.add(PriorFactor<Pose3>(key, trans2gtsamPose(pose), priorPoseNoise));
            initialEstimate.insert(0, trans2gtsamPose(pose));
        }
        else{
            key = vec_odom.size()-1;

            std::array<float, 6> prev_pose = vec_odom[key-1];
            // noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
            // noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = trans2gtsamPose(prev_pose);
            gtsam::Pose3 poseTo   = trans2gtsamPose(pose);

            gtSAMgraph.add(BetweenFactor<Pose3>(key-1, key, poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(key, poseTo);
        }
    }


    void optimize() {
        isam.update(gtSAMgraph, initialEstimate);
        // isam.update();
        // isam.update();
        // isam.update();
    
        publishCurrentPose();

        if (bool_cloud){
            vec_map_cloud.clear();
    
            publishCloudMap();
        }

        trajectoryVisualization(vec_odom);

        gtSAMgraph.resize(0);
        initialEstimate.clear();

    }

    Values getCurrentEstimate() {
        return isam.calculateEstimate();
    }

    // void publishCurrentPose() {
    //     Values currentEstimates = getCurrentEstimate();
    //     currentEstimates.print();
    //     if (!currentEstimates.empty()) {

    //         vec_odom.clear();
    //         cout << "SIZE: " << currentEstimates.size() << endl;

    //         for (size_t i = 0; i < currentEstimates.size(); i++) {
    //             Pose3 currentPose = currentEstimates.at<Pose3>(i);  // get the last pose
            
    void publishCurrentPose() {
        Values currentEstimates = getCurrentEstimate();
        // currentEstimates.print();

        if (!currentEstimates.empty()) {
            // Marginals marginals(gtSAMgraph, currentEstimates);

            // cout.precision(2);
            // for (int i = 0; i < vec_odom.size(); i++){
            //     cout << "Covariance:\n" << marginals.marginalCovariance(i) << endl;
            // }

            vec_odom.clear();
            // cout << "SIZE: " << currentEstimates.size() << endl;

            gtsam::Key lastKey;
            for(const gtsam::Values::ConstKeyValuePair& key_value: currentEstimates) {
                gtsam::Key key = key_value.key;
                Pose3 currentPose;
                // gtsam::Matrix currentPoseCovariance;
                try {
                    currentPose = currentEstimates.at<Pose3>(key);
                    // currentPoseCovariance = marginals.marginalCovariance(key);

                    // cout << currentPoseCovariance << endl;
                } catch (const std::exception& e) {
                    std::cerr << "Error retrieving pose for key: " << key << ". Error: " << e.what() << std::endl;
                    continue;
                }

                optimized_odom[0] = currentPose.rotation().roll();
                optimized_odom[1] = currentPose.rotation().pitch();
                optimized_odom[2] = currentPose.rotation().yaw();
                optimized_odom[3] = currentPose.translation().x();
                optimized_odom[4] = currentPose.translation().y();
                optimized_odom[5] = currentPose.translation().z();

                vec_odom.push_back(optimized_odom);
                lastKey = key;
            }


            // geometry_msgs::PoseStamped poseStamped;

            // if (i == currentEstimates.size() - 1){

            poseStamped.header.stamp = ros::Time::now();
            poseStamped.header.frame_id = "map";

            // poseStamped.pose.position.x = 0;
            // poseStamped.pose.position.y = 0;
            // poseStamped.pose.position.z = 0;
            // poseStamped.pose.orientation.w = 1;
            // poseStamped.pose.orientation.x = 0;
            // poseStamped.pose.orientation.y = 0;
            // poseStamped.pose.orientation.z = 0;

            if (currentEstimates.exists(lastKey)) {
            // if (currentEstimates.size() > 1) {
                
                if (debugMode){
                    cout << "???" << optimized_odom[0] << " " << optimized_odom[1] << " " << optimized_odom[2] << " " << 
                    optimized_odom[3] << " " << optimized_odom[4] << " " << optimized_odom[5] << " " << endl;  
                }
                
                prev_optimized_odom = optimized_odom;
                Pose3 lastPose = currentEstimates.at<Pose3>(lastKey);

                tf::Quaternion q(
                    lastPose.rotation().toQuaternion().x(),
                    lastPose.rotation().toQuaternion().y(),
                    lastPose.rotation().toQuaternion().z(),
                    lastPose.rotation().toQuaternion().w());

                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                // m.getRPY(roll, pitch, yaw);
                getNormalizedRPY(q, roll, pitch, yaw);

                poseStamped.pose.position.x = lastPose.translation().x();
                poseStamped.pose.position.y = lastPose.translation().y();
                poseStamped.pose.position.z = lastPose.translation().z();
                poseStamped.pose.orientation.w = lastPose.rotation().toQuaternion().w();
                poseStamped.pose.orientation.x = lastPose.rotation().toQuaternion().x();
                poseStamped.pose.orientation.y = lastPose.rotation().toQuaternion().y();
                poseStamped.pose.orientation.z = lastPose.rotation().toQuaternion().z();

                // cout << poseStamped.pose.position.x << " " << poseStamped.pose.position.y << " " << poseStamped.pose.position.z << " "
                // << poseStamped.pose.orientation.w << " " << poseStamped.pose.orientation.x << " " << poseStamped.pose.orientation.y << " " << poseStamped.pose.orientation.z << endl;
                
                // posePub.publish(poseStamped);
                if (debugMode){
                    cout << "*******************Odom Factor Visualized*******************" << endl;
                }
                // cout << vec_odom.size()-1 << " " << poseStamped.pose.position.x << "  " 
                // << poseStamped.pose.position.y << "  " 
                // << poseStamped.pose.position.z << "  optimized." << endl;
            }

            poseCovariance = isam.marginalCovariance(currentEstimates.size()-1);
            cout << "Pose covariance:" <<poseCovariance(3,3) << " " << poseCovariance(4,4) << "====" << poseCovThreshold << endl;
            
            // cout << prev_optimized_odom[0] << " " << prev_optimized_odom[1] << " " << prev_optimized_odom[2] << endl;
            posePub.publish(poseStamped);
        }
        
    }

    void publishCloudMap() {
        for (int i = 0; i < vec_odom.size(); i++){
            auto pose = vec_odom[i];
            auto v_cloud = vec_cloud[i];
            
            if (v_cloud->points.size() == 0){
                continue;
            }

            // cout << v_cloud->points[0].x << "***************" << pose[0] << " " << pose[1] << " " << pose[2] 
            // << " " << pose[3] << " " << pose[4] << " " << pose[5] << endl;

            CloudT::Ptr tfm_v_cloud(new CloudT());

            double roll = pose[0];
            double pitch = pose[1];
            double yaw = pose[2];
            double x = pose[3];
            double y = pose[4];
            double z = pose[5];

            // Convert the roll, pitch, yaw into a quaternion.
            tf2::Quaternion tf2_quaternion;
            tf2_quaternion.setRPY(roll, pitch, yaw);

            // Convert the quaternion to a tf2::Matrix3x3 rotation matrix
            tf2::Matrix3x3 tf2_rotation_matrix(tf2_quaternion);

            // Create the tf2::Vector3 object from the translation data
            tf2::Vector3 tf2_translation_vector(x, y, z);

            // Construct the tf2::Transform object using the setBasis and setOrigin functions
            tf2::Transform tf2_transform;
            tf2_transform.setBasis(tf2_rotation_matrix);
            tf2_transform.setOrigin(tf2_translation_vector);

            // // Create the translation vector.
            // tf2::Vector3 tf2_translation_vector(x, y, z);

            // // Create the tf2::Transform object.
            // tf2::Transform tf2_transform(tf2_quaternion, tf2_translation_vector);

            // Use the transform
            transformMapCloud(tf2_transform, v_cloud, tfm_v_cloud);

            // cout << tfm_v_cloud->points[0].x << " " << tfm_v_cloud->points[0].y << " " << tfm_v_cloud->points[0].z << endl
            // << v_cloud->points[0].x << " " << v_cloud->points[0].y << " " << v_cloud->points[0].z << endl;
            // cout << "============================================" << endl;

            vec_map_cloud.push_back(tfm_v_cloud);
        }
        
        visualization_msgs::MarkerArray viz_mapTotalCloud = mapCloudVisualization(vec_map_cloud);
        mapCloudPub.publish(viz_mapTotalCloud);
    }

    // void transformMapCloud(tf2::Transform tf, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& tfm_inCloudClusters)
    void transformMapCloud(tf2::Transform tf, CloudT::Ptr& inCloud, CloudT::Ptr& tfm_inCloud){
        geometry_msgs::Transform transform;
        tf2::convert(tf, transform);
        // tfm_inCloud = inCloud;

        pcl_ros::transformPointCloud(*inCloud, *tfm_inCloud, transform);
        tfm_inCloud->header.frame_id = map_frame_id;
    }

    void gpsMapFrameVisualization(const nav_msgs::Odometry& thisGPS)
    {
        nav_msgs::Odometry msg;

        msg.header = thisGPS.header;
        msg.header.frame_id = "map";

        msg.pose = thisGPS.pose;
        msg.twist = thisGPS.twist;

        gpsMapPub.publish(msg);
    }
    
    void odomMapFrameVisualization(const nav_msgs::Odometry& thisOdom)
    {
        nav_msgs::Odometry msg;

        msg.header = thisOdom.header;
        msg.header.frame_id = "map";

        msg.pose = thisOdom.pose;
        msg.twist = thisOdom.twist;

        odomMapPub.publish(msg);
    }

    visualization_msgs::MarkerArray mapCloudVisualization(const std::vector<CloudT::Ptr> vec_map_cloud)
    {
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
            // marker.header.frame_id = map_frame_id_;

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
        return marker_array;
    }

    void printFactors() {
        for (const auto& factor : gtSAMgraph) {
            factor->print();
            // std::cout << factor->print() << std::endl;
        }
    }

    void vCloudCallback(const sensor_msgs::PointCloud2ConstPtr &vCloudMsg) {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::fromROSMsg(*vCloudMsg, *pclCloud);
        // vCloudQueue.push_back(pclCloud);
        vCloudQueue.push(vCloudMsg);

    }

    void addCloud() {
        if (vCloudQueue.empty()){

            CloudT::Ptr tfm_v_cloud(new CloudT());
            tfm_v_cloud->header.frame_id = robot_frame_id;
            vec_cloud.push_back(tfm_v_cloud);
        } 
            
        while (!vCloudQueue.empty())
        {   
            if (vCloudQueue.front()->header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // cout << vCloudQueue.front()->header.stamp.toSec() - timeLaserInfoCur << "message too old" << endl;
                vCloudQueue.pop();
            }
            else if (vCloudQueue.front()->header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {                
                // vec_cloud.push_back(tfm_v_cloud);
                // cout << "message too new" << endl;

                CloudT::Ptr tfm_v_cloud(new CloudT());
                tfm_v_cloud->header.frame_id = robot_frame_id;
                vec_cloud.push_back(tfm_v_cloud);
                break;
            }
            else
            {
                sensor_msgs::PointCloud2ConstPtr rosCloud;
                CloudT::Ptr v_cloud(new CloudT());

                // rosCloud = vCloudQueue.front();
                pcl::fromROSMsg(*vCloudQueue.front(), *v_cloud);
                vCloudQueue.pop();
    
                tf2::Transform tf_vCloudSourceToTarget;
                bool_vCloudTransformFrame = transformFrame(v_lidar_frame_id, robot_frame_id, tf_vCloudSourceToTarget);
            
                if (!bool_vCloudTransformFrame){
                    // cout << "NO TRANSFORM" << endl;

                    CloudT::Ptr tfm_v_cloud(new CloudT());
                    tfm_v_cloud->header.frame_id = robot_frame_id;
                    vec_cloud.push_back(tfm_v_cloud);
                    break;
                }

                geometry_msgs::Transform transform;
                tf2::convert(tf_vCloudSourceToTarget, transform);

                CloudT::Ptr tfm_v_cloud(new CloudT());

                pcl_ros::transformPointCloud(*v_cloud, *tfm_v_cloud, transform);
                tfm_v_cloud->header.frame_id = robot_frame_id;

                vec_cloud.push_back(tfm_v_cloud);

                break;
                // return;
            }
        }
        // vec_cloud.push_back(tfm_v_cloud);
    }

    bool transformFrame(const std::string source_frame, const std::string target_frame, tf2::Transform& tf_sourceToTarget)
    {
        // tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

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

    void trajectoryVisualization(const std::vector<std::array<float, 6>> &vec_odom)
    {
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
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.01;

        // if (pose_type == 0){  // pose_type: 0 (original trajectory)
        //     points.header.frame_id = line_strip.header.frame_id = "map";
        //     points.color.r = 1.0;
        //     points.color.g = 0.0;
        //     points.color.b = 0.0;
        //     points.color.a = 1.0;

        //     // Line strip is blue
        //     line_strip.color.r = 0.0;
        //     line_strip.color.g = 0.0;
        //     line_strip.color.b = 0.0;
        //     line_strip.color.a = 1.0;
        // } 
        // else if (pose_type == 1){  // pose_type: 1 (trajectory)
        //     points.header.frame_id = line_strip.header.frame_id = "map";
        //     points.color.r = 0.0;
        //     points.color.g = 1.0;
        //     points.color.b = 0.0;
        //     points.color.a = 1.0;

        //     // Line strip is blue
        //     line_strip.color.r = 0.0;
        //     line_strip.color.g = 0.0;
        //     line_strip.color.b = 1.0;
        //     line_strip.color.a = 1.0;
        // }

        points.header.frame_id = line_strip.header.frame_id = "map";
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

        trajectoryPub.publish(tMarkerArray);
        // return tMarkerArray;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gtsam_optimizer");
    ros::NodeHandle nh;

    GtsamOptimizer optimizer(nh);

    ros::spin();

    return 0;
}
