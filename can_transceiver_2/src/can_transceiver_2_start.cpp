#include "can_transceiver_2/can_transceiver_2.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "can_transceiver_2");
	CanTransceiver node;
    node.Run();
}
