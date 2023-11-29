#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// Odometry callback function
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    // Set the position of the transform
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    
    // Set the orientation of the transform
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    transform.setRotation(q);

    // Broadcasting the transform from "odom" to "base_link2"
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "map", "base_link2"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "newTf_broadcaster");

    ros::NodeHandle node;
    
    // Subscribe to the odometry topic
    ros::Subscriber sub = node.subscribe("gtsam/optimized_pose_2", 10, &odometryCallback);

    ros::spin();
    return 0;
};
