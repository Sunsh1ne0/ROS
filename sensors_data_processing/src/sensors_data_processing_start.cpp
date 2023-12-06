#include "sensors_data_processing/sensors_data_processing.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sensors_data_processing");
	SensorsData node;
    node.Run();
}
