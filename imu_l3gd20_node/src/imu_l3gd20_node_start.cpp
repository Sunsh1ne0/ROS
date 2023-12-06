#include "imu_l3gd20_node/imu_l3gd20_node.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_l3gd20_node");
	IMU_L3GD20 node;
    node.Run();
}

