/*
 * WaypointDriverFile.h
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_waypoints_navigation_WaypointDriverFile_H_
#define INCLUDE_waypoints_navigation_WaypointDriverFile_H_


#include "WaypointDriverBase.h"


using namespace std;


class WaypointDriverFile : public WaypointDriverBase {



public:

	WaypointDriverFile() : WaypointDriverBase() {

		ROS_INFO("Waiting for /move_base action server...");
		moveBaseController_.waitForServer();
		
		ros::Duration(1).sleep();
		
		ROS_INFO("Connected to /move_base!\n");


		systemTimer_ = node_.createTimer(ros::Rate(20), 
                &WaypointDriverFile::updateTimerCallback, this);
			

	}

	virtual ~WaypointDriverFile(){}


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
				ROS_INFO("Navigation done");

				waypoints_.clear();
				

				std_msgs::Bool msg;
				msg.data = true;
				navigationDonePub_.publish(msg);

				return;


			} else if (state == ABORT ){

				ROS_INFO("================================================");
				ROS_INFO("Navigation aborted");

				waypoints_.clear();

				abortMission_ = false;

				getPathTopic_ = false;
				
				//waith for new path, stand in place
				while (ros::ok()) {
					
					ROS_INFO("================================================");
					ROS_INFO("Waiting fo new path");
					if ( getPathTopic_){
						ROS_INFO("================================================");
						ROS_INFO("new path recieved!!! ");							
						break;	
					}
					ros::spinOnce();
				}	
			}

		}

		ros::spinOnce();
			
		
	}
	string getMoveBaseState( const actionlib::SimpleClientGoalState& state ){

		if( state == actionlib::SimpleClientGoalState::PENDING){

			return "PENDING";
		}
		if( state == actionlib::SimpleClientGoalState::ACTIVE){
			
			return "ACTIVE";
		}
		if( state == actionlib::SimpleClientGoalState::RECALLED){
			
			return "RECALLED";
		}
		if( state == actionlib::SimpleClientGoalState::REJECTED){
			
			return "REJECTED";
		}
		if( state == actionlib::SimpleClientGoalState::PREEMPTED){
			
			return "PREEMPTED";
		}
		if( state == actionlib::SimpleClientGoalState::ABORTED){
			
			return "ABORTED";
		}
		if( state == actionlib::SimpleClientGoalState::SUCCEEDED){
			
			return "SUCCEEDED";
		}
		if( state == actionlib::SimpleClientGoalState::LOST){
			
			return "LOST";
		}

		return "";
	}
	MISSION_STATE missionExectue() {
	

		ROS_INFO("WaypointDriverFile: Starting navigation (%lu waypoints):", waypoints_.size());	

		for (int i = 0; i < waypoints_.size(); ++i) {
			auto waypoint = waypoints_[i];

			cerr<<"WaypointDriverFile: goal: x, "<<waypoint.pose.position.x<<" y "<<waypoint.pose.position.y<<endl;
				
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

					cerr<<" move-base state "<<getMoveBaseState(moveBaseController_.moveBaseClient_.getState())<<endl;
				}

				/// recieved ABORT while nav to goal
				if( abortMission_ ){

					cerr<<"WaypointDriverFile: ABORT mission "<<endl;

					moveBaseController_.cancelNavigation();
							
					return ABORT;
				}

				/// recieved PAUSE while nav to goal

				if( pauseMission_ ){

					cerr<<"WaypointDriverFile: pause mission, cancel Goal  "<<endl;

					pauseMission_ = false;

					moveBaseController_.cancelNavigation();

					// pause, wait for continue mission flag
					while (ros::ok()) {

						/// recieved CONTINUE while nav to goal
						if (continueMission_){

							cerr<<"WaypointDriverFile: continue mission "<<endl;
	
							continueMission_ = false;
							break;	
						}

						ros::spinOnce();
					}	
				}

				ros::spinOnce();	

			
			}		

			//bool result = moveBaseController_.wait(); //block
			ROS_INFO("\t #%i: %s", i + 1, res ? "Waypoint reached!" : "Failed to reach waypoint");
			
		}		

		cerr<<"WaypointDriverFile: FINISHED mission "<<endl;
 
		return FINISHED;
		
	}

private:

	ros::Timer systemTimer_;	

	ros::NodeHandle node_;


};

#endif /* INCLUDE_waypoints_navigation_WaypointDriverFile_H_ */
