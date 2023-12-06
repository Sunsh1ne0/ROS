#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // Extract the quaternion data from the IMU message
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imu_msg->orientation, orientation);

    // Convert the quaternion to Euler angles
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Print the Euler angles in radians
    ROS_INFO("Roll: %.2f radians, Pitch: %.2f radians, Yaw: %.2f radians", roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_to_euler_node");
    ros::NodeHandle nh;

    // Subscribe to the /imu topic with sensor_msgs/Imu messages
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, imuCallback);

    ros::spin();

    return 0;
}
