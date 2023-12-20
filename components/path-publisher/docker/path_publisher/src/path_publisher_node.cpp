

#include <path_publisher/PathPublisher.h>

#include <ros/ros.h>

using namespace std;



bool readWaypoints(vector<geometry_msgs::PoseStamped>& waypoints) {
	
	ros::NodeHandle nodePrivate("~");

	vector<string> waypointsList;


	if (nodePrivate.getParam("waypoints", waypointsList)) {

		geometry_msgs::PoseStamped pose;

		int line = 1;

		for(auto waypointString : waypointsList) {
			double heading = 0;

			auto parsedValues = sscanf(waypointString.c_str(), "%lf,%lf,%lf",
					&pose.pose.position.x,
					&pose.pose.position.y,
					&heading);

			pose.header.frame_id = "map";
			pose.header.stamp = ros::Time(0);

			pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

			waypoints.push_back(pose);

			if (parsedValues < 3) {
				ROS_ERROR("Failed to parse a waypoint (line %i)", line);
				return false;
			}

			line++;
		}

	} else {


		ROS_ERROR("Error: waypoints parameter does not exists or empty");
		
		return false;
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_publisher_node");

	vector<geometry_msgs::PoseStamped> waypoints;

	if (readWaypoints(waypoints) == false) {
		
		ROS_WARN("Waypoints list is empty, exiting");
		cerr<<" Waypoints list is empty, exiting "<<endl;
		
		return -1;

	} else {


		PathPublisher PathPublisher(waypoints);

		ros::spin();
	}




	return 0;
}