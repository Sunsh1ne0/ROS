#ifndef WHEELS_CONTROLLER_H_
#define WHEELS_CONTROLLER_H_

#include <ros/ros.h>
#include <cmath>
#include "agrobit_msg/can_bus.h"
#include "agrobit_msg/imu_l3gd20.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

#define SPIN_RATE_HZ 200

struct data{
int32_t gyro[3];
int32_t acc[3];
};

class IMU_L3GD20 {

public:
	IMU_L3GD20();
	void Run();
	void FirstPackageCallback(const agrobit_msg::can_bus& can_bus_msg);
	void SecondPackageCallback(const agrobit_msg::can_bus& can_bus_msg);
	
private:
	void Init();
	void PublishIMUl3gd20DataPackage();

	ros::NodeHandle n_;
	ros::Subscriber first_package_subscriber_;
	ros::Subscriber second_package_subscriber_;
	ros::Publisher imu_l3gd20_data_publisher_;
	ros::Publisher imu_format_publisher_;

	data imu_l3gd20_data;
};

#endif
