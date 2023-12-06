#include "wheels_controller/wheels_controller.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheels_controller");
	WheelsController node;
    node.Run();
}
