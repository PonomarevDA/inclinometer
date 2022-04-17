/**
 * @file tf_broadcaster.hpp
 * @author ponomarevda96@gmail.com
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>


void publish_state(const geometry_msgs::Pose::ConstPtr& pose) {
    static tf2_ros::TransformBroadcaster tfPub_;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "/world";
    transform.transform.translation.x = pose->position.x;
    transform.transform.translation.y = pose->position.y;
    transform.transform.translation.z = pose->position.z;
    transform.transform.rotation.x = pose->orientation.x;
    transform.transform.rotation.y = pose->orientation.y;
    transform.transform.rotation.z = pose->orientation.z;
    transform.transform.rotation.w = pose->orientation.w;
    std::string ns = ros::this_node::getNamespace();
    std::string child_frame_id = ns.append("/inclinometer");
    transform.child_frame_id = child_frame_id.c_str();
    tfPub_.sendTransform(transform);
}


void imu_cb(const geometry_msgs::Pose::ConstPtr& pose) {
    publish_state(pose);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "yellDozer_tf_broadcaster");
    ros::NodeHandle nh;
    auto poly_sub = nh.subscribe("yellDozer", 1, imu_cb);

    ros::Rate loop_rate(500);
    while ( ros::ok() ) {
        ros::spinOnce();    // handle callbacks
        loop_rate.sleep();
    }

    return 0;
}
