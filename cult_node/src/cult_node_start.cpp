#include "cult_node/cult_node.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cult_node");
	CultivatorController node;
    node.Run();
}
