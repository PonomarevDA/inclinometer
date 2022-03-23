/**
 * @file tf_broadcaster.hpp
 * @author ponomarevda96@gmail.com
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>


void publish_state(float q_w, float q_x, float q_y, float q_z) {
    static tf2_ros::TransformBroadcaster tfPub_;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;
    transform.transform.rotation.x = q_x;
    transform.transform.rotation.y = q_y;
    transform.transform.rotation.z = q_z;
    transform.transform.rotation.w = q_w;
    transform.child_frame_id = "/inclinometer";
    tfPub_.sendTransform(transform);
}


void cover_cb(const sensor_msgs::Imu::ConstPtr& imu) {
    float q_w = imu->orientation.w;
    float q_x = imu->orientation.x;
    float q_y = imu->orientation.y;
    float q_z = imu->orientation.z;

    std::cout << "wxyz=[" << q_w << ", " << q_x << ", " << q_y << ", " << q_z << "]" << std::endl;
    publish_state(q_w, q_x, q_y, q_z);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cover_tf_broadcaster");
    ros::NodeHandle nh;
    auto poly_sub = nh.subscribe("/inclinometer", 1, cover_cb);

    ros::Rate loop_rate(500);
    while ( ros::ok() ) {
        ros::spinOnce();    // handle callbacks
        loop_rate.sleep();
    }

    return 0;
}
