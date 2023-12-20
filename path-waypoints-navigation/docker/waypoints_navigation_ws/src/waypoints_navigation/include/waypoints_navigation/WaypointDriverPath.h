/*
 * WaypointDriverPath.h
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_waypoints_navigation_WaypointDriverPath_H_
#define INCLUDE_waypoints_navigation_WaypointDriverPath_H_


#include "WaypointDriverBase.h"


using namespace std;


class WaypointDriverPath : public WaypointDriverBase {



public:

	WaypointDriverPath() : WaypointDriverBase() {

		ROS_INFO("Waiting for /move_base action server...");
		moveBaseController_.waitForServer();
		
		ros::Duration(1).sleep();
		
		ROS_INFO("Connected to /move_base!\n");


		systemTimer_ = node_.createTimer(ros::Rate(20), 
                &WaypointDriverPath::updateTimerCallback, this);
			

	}

	virtual ~WaypointDriverPath(){}


private:

	

	void publishNavigationDone( ) {

		std_msgs::Bool msg;
		msg.data = true;
		navigationDonePub_.publish(msg);
	}

	void updateTimerCallback(const ros::TimerEvent&) {

		if ( getPathTopic_ ) {

			getPathTopic_ = false;
			
			MISSION_STATE state = missionExectue();

			if( state == FINISHED ){			

				ROS_INFO("================================================");
				string s = "Navigation done";
				ROS_INFO("Navigation done");
				publishLog(s);		

				waypoints_.clear();
				visitedWaypoints_.clear();

				getPathTopic_ = false;

				std_msgs::Bool msg;
				msg.data = true;
				navigationDonePub_.publish(msg);

				return;


			} else if (state == ABORT ){

				string s = "Navigation aborted";
				ROS_INFO("================================================");
				ROS_INFO("Navigation aborted");
				publishLog(s);

				waypoints_.clear();
				visitedWaypoints_.clear();

				abortMission_ = false;

				getPathTopic_ = false;
				
				//waith for new path, stand in place
				while (ros::ok()) {
					
					ROS_INFO("================================================");
					string s = "Waiting fo new path";
					ROS_INFO("Waiting fo new path");
					publishLog(s);

					if ( getPathTopic_){

						ROS_INFO("================================================");
						string s = "new path recieved!!!";
						ROS_INFO("new path recieved!!!");
						publishLog(s);
									
						break;	
					}
					ros::spinOnce();
				}	
			}

		} else {
			
			waypoints_.clear();
			visitedWaypoints_.clear();

			ROS_INFO("================================================");
			string s = "Waiting fo new path";
			ROS_INFO("Waiting fo new path");
			publishLog(s);			
			
		}

		ros::spinOnce();
			
		
	}

	
	MISSION_STATE missionExectue() {
	
		ROS_INFO("WaypointDriverPath: Starting navigation (%lu waypoints):", waypoints_.size());
		
		if(min_distance_ != -1) {
			orderWaypointsByDistance(min_distance_); //changes the waypoints_ variable according to the min_distance_ parameter
		}

		for (int i = 0; i < waypoints_.size(); i++) {

			auto waypoint = waypoints_[i];

			cerr<<"WaypointDriverPath: goal: x, "<<waypoint.pose.position.x<<" y "<<waypoint.pose.position.y<<endl;

			std_msgs::Bool msg_false;
			msg_false.data = false;
			current_goal_reached_publisher_.publish(msg_false);
				
			//navigate to the point			
			moveBaseController_.navigate(waypoint);

			bool res = false;

			while(ros::ok()) {

				moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
				if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED){
					
					if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
						res = true;
					} else {
						res = false;
					}
					break;
				} else {

					//cerr<<" move-base state "<<getMoveBaseState(moveBaseController_.moveBaseClient_.getState())<<endl;
				}

					// If we want the robot to move to the next point when it is at a certain distance from a current point
				if( dontStopOnGoal_){

					if(updateRobotLocation()){

						double distFromCurrentGoal = 
							distanceCalculate(waypoint, robotPose_);

						if( distFromCurrentGoal < maxDistBeforeSkipCurrentGoal_){
							
							std_msgs::Bool msg_true;
							msg_true.data = false;
							current_goal_reached_publisher_.publish(msg_true);
							break;

						}		
					}
				}

				/// recieved ABORT while nav to goal
				if( abortMission_ ){

					cerr<<"WaypointDriverPath: ABORT mission "<<endl;

					moveBaseController_.cancelNavigation();
							
					return ABORT;
				}

				/// recieved PAUSE while nav to goal

				if( pauseMission_ ) {

					ROS_INFO("================================================");
					string s = "pause mission";
					ROS_INFO("pause mission");
					publishLog(s);	

					pauseMission_ = false;

					// pause, wait for continue mission flag
					while (ros::ok()) {

						/// recieved CONTINUE while nav to goal
						if (continueMission_){

							ROS_INFO("================================================");
							string s = "continue mission";
							ROS_INFO("continue mission");
							publishLog(s);
	
							continueMission_ = false;
							break;	
						}

						ros::spinOnce();
					}	
				}

				ros::spinOnce();	

			
			}		

			

			ROS_INFO("================================================");
			string s = "Waypoint reached!";
			ROS_INFO("\t #%i: %s", i + 1, res ? "Waypoint reached!" : "Failed to reach waypoint");
			publishLog(s);

			std_msgs::Bool msg_true;
			msg_true.data = true;
			current_goal_reached_publisher_.publish(msg_true);
			
			visitedWaypoints_[i] = true;

			if (wait_for_pause_cmd_enable){

				cerr<<" wait for pause cmd "<<endl;	
				while (ros::ok()) {

						/// recieved CONTINUE while nav to goal
					if (pauseMission_){

						pauseMission_ = false;
						break;
					}

					ros::spinOnce();
				}

				cerr<<" wait for continue cmd "<<endl;	
				while (ros::ok()) {

					if (continueMission_){

						continueMission_ = false;
						break;
					}

					ros::spinOnce();
				}	
			}
		}		


		ROS_INFO("================================================");
		string s = "mission completed!";
		ROS_INFO("mission completed!");
		publishLog(s);
 
		return FINISHED;
		
	}

	bool updateRobotLocation() {

        tf::StampedTransform transform;

        try
        {
            //get current robot pose
            tfListener_.lookupTransform("map", "base_link",
                                        ros::Time(0), transform);

            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();



            return true;
        }

        catch (...)
        {
            cerr << " error between " << "map" << " to " << "base_link" << endl;
            return false;
        }
    }

	double distanceCalculate(const geometry_msgs::PoseStamped& p1, 
		const geometry_msgs::PoseStamped& p2)
    {
        double x = p1.pose.position.x - p2.pose.position.x; //calculating number to square in next step
        double y = p1.pose.position.y - p2.pose.position.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

	void orderWaypointsByDistance(double minimumDistance) {
		bool flag;
		int i = 0;
		int j = 1;
    	vector<geometry_msgs::PoseStamped> localWaypoints;
    	for(int i = 0; i < waypoints_.size(); i++) {
        	localWaypoints.push_back(waypoints_.at(i));
   		}
    	vector<geometry_msgs::PoseStamped> result;
		result.push_back(localWaypoints.at(0));
		while (i < localWaypoints.size()) {
			flag = false;
			while(!flag) {
				if(j >= localWaypoints.size()) {
					break;
				}
				if(distanceCalculate(waypoints_.at(i),waypoints_.at(j)) > minimumDistance) {
					result.push_back(waypoints_.at(j));
					flag = true;
				}
				else
					j++;
			}
			i=j;
			j++;
		}
		if (std::find(result.begin(), result.end(), waypoints_.at(waypoints_.size()-1)) != result.end()) {
			waypoints_ = result;
		}
		else {
			result.push_back(waypoints_.at(waypoints_.size()-1));
			waypoints_ = result;
		}
	}
	

private:

	ros::Timer systemTimer_;	

	ros::NodeHandle node_;


};

#endif /* INCLUDE_waypoints_navigation_WaypointDriverPath_H_ */

