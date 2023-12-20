/**
 * Filename: lynx_navigation_manager_node.cpp
 *   Author: Daria Syvash
 *     Date: Aug 28, 2019
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <lynx_navigation/manager/NavigationManager.h>

using namespace lynx;

void testMission(NavigationManager& manager) {
	const int WAYPOINT_COUNT = 3;

	lynx_msgs::Mission mission;

	mission.name = "test_mission";
	
	for (size_t i = 0; i < WAYPOINT_COUNT; i++) {
		lynx_msgs::Waypoint w;
		w.latitude = 34.0 + i / 1e6;
		w.longitude = 32.0 + i / 1e6;
		w.name = "waypoint_" + std::to_string(i);

		mission.waypoints.push_back(w);
	}

	manager.startMission(mission);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lynx_navigation_manager_node");
	ros::NodeHandle node;
    NavigationManager manager;

	if (argc > 1) {
		if (string(argv[1]) == "-t") {
			testMission(manager);
		}
	}

    ros::spin();
	return 0;
}
