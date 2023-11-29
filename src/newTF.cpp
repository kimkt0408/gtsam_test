#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

// PoseStamped callback function
void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Set the position of the transform
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    
    // Set the orientation of the transform
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);

    // Broadcasting the transform from "base_link" to "base_link2"
    // br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link2"));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "map", "base_link2"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link2"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "new_tf_broadcaster");

    ros::NodeHandle node;
    
    // Subscribe to the PoseStamped topic
    ros::Subscriber sub = node.subscribe("gtsam/optimized_pose_2", 10, &poseStampedCallback);

    ros::spin();
    return 0;
};
