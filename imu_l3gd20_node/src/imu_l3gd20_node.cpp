#include "imu_l3gd20_node/imu_l3gd20_node.h"

IMU_L3GD20::IMU_L3GD20() {

	first_package_subscriber_ = n_.subscribe("/IMU_first", 1, &IMU_L3GD20::FirstPackageCallback, this);
	second_package_subscriber_ = n_.subscribe("/IMU_second", 1, &IMU_L3GD20::SecondPackageCallback, this);

	imu_l3gd20_data_publisher_ = n_.advertise<agrobit_msg::imu_l3gd20>("/imu_l3gd20_data", 1);
	imu_format_publisher_ = n_.advertise<sensor_msgs::Imu>("/imu", 1);
	
}

void IMU_L3GD20::FirstPackageCallback(const agrobit_msg::can_bus& can_bus_msg) {
int8_t temp1, temp2, temp3, temp4;
if (can_bus_msg.data[2] & 0x08){
	if (((~can_bus_msg.data[0] + 1) & 0xFF) == 0){
		temp1 = 0b1;
		if (((~can_bus_msg.data[1] + temp1) & 0xFF) == 0)
			temp2 = 0b1;
		else
			temp2 = 0b0;
	}
	else{
		temp1 = 0b0;
		temp2 = 0b0;
	}
	imu_l3gd20_data.gyro[0] = -(((~can_bus_msg.data[0] + 1) & 0xFF) + (((~can_bus_msg.data[1] + temp1) & 0xFF) << 8) + (((~(can_bus_msg.data[2] >> 4) + temp2) & 0x0F) << 16));
}
else 
	imu_l3gd20_data.gyro[0] = (can_bus_msg.data[0] + (can_bus_msg.data[1] << 8) + ((can_bus_msg.data[2] & 0xF0) << 12));

if (can_bus_msg.data[5] & 0x80){
	if (((((~can_bus_msg.data[2]) & 0x07) + 1) & 0x07) == 0)
	{
		temp1 = 1;
		if (((~can_bus_msg.data[3] + 1) & 0xFF) == 0)
		{
			temp2 = 1;
			if (((~can_bus_msg.data[4] + 1) & 0xFF) == 0)
			{
				temp3 = 1;
			}
			else
				temp3 = 0;	
		}
		else
		{
			temp2 = 0;
			temp3 = 0;
		}
	}
	else
	{
		temp1 = 0;
		temp2 = 0;
		temp3 = 0;
	}	
	imu_l3gd20_data.gyro[1] = -((((~can_bus_msg.data[2] & 0x07) + 1) & 0x07) + (((~can_bus_msg.data[3] + temp1) & 0xFF) << 3) + (((~can_bus_msg.data[4] + temp2) & 0xFF) << 11) + ((((~can_bus_msg.data[5] >> 7) + temp3) & 0x01) << 19));
}
else
	imu_l3gd20_data.gyro[1] = ((can_bus_msg.data[2] & 0x07) + (can_bus_msg.data[3] << 3) + (can_bus_msg.data[4] << 11) + ((can_bus_msg.data[5] & 0x80) << 12));
if (can_bus_msg.data[7] & 0x02){
	if (((~can_bus_msg.data[5] + 1) & 0x3F) == 0)
	{
		temp1 = 1;
		if (((~can_bus_msg.data[6] + 1) & 0xFF) == 0)
		{
			temp2 = 1;
		}
		else{
			temp2 = 0;
		}
	}
	else{
		temp1 = 0;
		temp2 = 0;
	}
	imu_l3gd20_data.gyro[2] = -(((~can_bus_msg.data[5] + 1) & 0x3F) + ((~can_bus_msg.data[6] + temp1) << 6) + ((((~can_bus_msg.data[7] >> 2) + 1) & 0x3F) << 14));
}
else
	imu_l3gd20_data.gyro[2] = ((can_bus_msg.data[5] & 0x3F) + (can_bus_msg.data[6] << 6) + (((can_bus_msg.data[7] >> 2) & 0x3F) << 14));
}

void IMU_L3GD20::SecondPackageCallback(const agrobit_msg::can_bus& can_bus_msg) {
int8_t temp1, temp2, temp3, temp4;
if (can_bus_msg.data[2] & 0x08){
	if (((~can_bus_msg.data[0] + 1) & 0xFF) == 0){
		temp1 = 0b1;
		if (((~can_bus_msg.data[1] + temp1) & 0xFF) == 0)
			temp2 = 0b1;
		else
			temp2 = 0b0;
	}
	else{
		temp1 = 0b0;
		temp2 = 0b0;
	}
	imu_l3gd20_data.acc[0] = -(((~can_bus_msg.data[0] + 1) & 0xFF) + (((~can_bus_msg.data[1] + temp1) & 0xFF) << 8) + (((~(can_bus_msg.data[2] >> 4) + temp2) & 0x0F) << 16));
}
else 
	imu_l3gd20_data.acc[0] = (can_bus_msg.data[0] + (can_bus_msg.data[1] << 8) + ((can_bus_msg.data[2] & 0xF0) << 12));

if (can_bus_msg.data[5] & 0x80){
	if (((((~can_bus_msg.data[2]) & 0x07) + 1) & 0x07) == 0)
	{
		temp1 = 1;
		if (((~can_bus_msg.data[3] + 1) & 0xFF) == 0)
		{
			temp2 = 1;
			if (((~can_bus_msg.data[4] + 1) & 0xFF) == 0)
			{
				temp3 = 1;
			}
			else
				temp3 = 0;	
		}
		else
		{
			temp2 = 0;
			temp3 = 0;
		}
	}
	else
	{
		temp1 = 0;
		temp2 = 0;
		temp3 = 0;
	}	
	imu_l3gd20_data.acc[1] = -((((~can_bus_msg.data[2] & 0x07) + 1) & 0x07) + (((~can_bus_msg.data[3] + temp1) & 0xFF) << 3) + (((~can_bus_msg.data[4] + temp2) & 0xFF) << 11) + ((((~can_bus_msg.data[5] >> 7) + temp3) & 0x01) << 19));
}
else
	imu_l3gd20_data.acc[1] = ((can_bus_msg.data[2] & 0x07) + (can_bus_msg.data[3] << 3) + (can_bus_msg.data[4] << 11) + ((can_bus_msg.data[5] & 0x80) << 12));
if (can_bus_msg.data[7] & 0x02){
	if (((~can_bus_msg.data[5] + 1) & 0x3F) == 0)
	{
		temp1 = 1;
		if (((~can_bus_msg.data[6] + 1) & 0xFF) == 0)
		{
			temp2 = 1;
		}
		else{
			temp2 = 0;
		}
	}
	else{
		temp1 = 0;
		temp2 = 0;
	}
	imu_l3gd20_data.acc[2] = -(((~can_bus_msg.data[5] + 1) & 0x3F) + ((~can_bus_msg.data[6] + temp1) << 6) + ((((~can_bus_msg.data[7] >> 2) + 1) & 0x3F) << 14));
}
else
	imu_l3gd20_data.acc[2] = ((can_bus_msg.data[5] & 0x3F) + (can_bus_msg.data[6] << 6) + (((can_bus_msg.data[7] >> 2) & 0x3F) << 14));
}


void IMU_L3GD20::PublishIMUl3gd20DataPackage() {
	agrobit_msg::imu_l3gd20 first_imu_l3gd20_data_for_pub;
	sensor_msgs::Imu imu_l3gd20_data_for_pub;
	geometry_msgs::Vector3 gyro;
	geometry_msgs::Vector3 acc;
	std_msgs::Header header;
	first_imu_l3gd20_data_for_pub.gyro_x = imu_l3gd20_data.gyro[0] / 10000.0;
	first_imu_l3gd20_data_for_pub.gyro_y = imu_l3gd20_data.gyro[1] / 10000.0;
	first_imu_l3gd20_data_for_pub.gyro_z = imu_l3gd20_data.gyro[2] / 10000.0;
	first_imu_l3gd20_data_for_pub.acc_x = imu_l3gd20_data.acc[0] / 10000.0;
	first_imu_l3gd20_data_for_pub.acc_y = imu_l3gd20_data.acc[1] / 10000.0;
	first_imu_l3gd20_data_for_pub.acc_z = imu_l3gd20_data.acc[2] / 10000.0;

	gyro.x = first_imu_l3gd20_data_for_pub.gyro_x;
	gyro.y = first_imu_l3gd20_data_for_pub.gyro_y;
	gyro.z = first_imu_l3gd20_data_for_pub.gyro_z;

	acc.x = first_imu_l3gd20_data_for_pub.acc_x;
	acc.y = first_imu_l3gd20_data_for_pub.acc_y;
	acc.z = first_imu_l3gd20_data_for_pub.acc_z;

	header.frame_id = "imu";
	header.stamp = ros::Time::now();

	imu_l3gd20_data_for_pub.header = header;
	imu_l3gd20_data_for_pub.angular_velocity = gyro;
	imu_l3gd20_data_for_pub.angular_velocity_covariance = {0.02, 0 , 0,
	0 , 0.02, 0,
	0 , 0 , 0.02};
	imu_l3gd20_data_for_pub.linear_acceleration = acc;
	imu_l3gd20_data_for_pub.linear_acceleration_covariance = {0.04 , 0 , 0,
	0 , 0.04, 0,
	0 , 0 , 0.04};

	imu_l3gd20_data_for_pub.orientation_covariance = {-1 , 0 , 0,
	0 , 0.04, 0,
	0 , 0 , 0.04};
	
	imu_format_publisher_.publish(imu_l3gd20_data_for_pub);
	imu_l3gd20_data_publisher_.publish(first_imu_l3gd20_data_for_pub);
}


void IMU_L3GD20::Run() {
	ros::Rate rate(SPIN_RATE_HZ);
	while(ros::ok()) {
		PublishIMUl3gd20DataPackage();
		rate.sleep();
		ros::spinOnce();
	}
}