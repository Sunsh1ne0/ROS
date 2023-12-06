#include "can_transceiver/can_transceiver.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "can_transceiver");
	CanTransceiver node;
    node.Run();
}
