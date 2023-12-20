/*
 * waypoint_driver_node.cpp
 *
 *  Created on: Jan 1, 2022
 *      Author: yakirhuri
 */


#include <waypoints_navigation/WaypointDriverPath.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_driver_node");

	WaypointDriverBase *waypointDriverBase = nullptr;

	if (true)
	{
		waypointDriverBase = new WaypointDriverPath();
	}

	ros::spin();

	return 0;
}