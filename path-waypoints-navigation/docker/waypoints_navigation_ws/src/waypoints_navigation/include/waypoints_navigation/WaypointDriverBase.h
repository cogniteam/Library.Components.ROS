/*
 * WaypointDriverBase.h
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 */

#ifndef INCLUDE_waypoints_navigation_WaypointDriverBase_H_
#define INCLUDE_waypoints_navigation_WaypointDriverBase_H_


#include <vector>
#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <waypoints_navigation/MoveBaseController.h>
#include <tf/transform_listener.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;


enum MISSION_STATE {
    ABORT,
    FINISHED   

};


class WaypointDriverBase {



public:

	WaypointDriverBase(){

		ros::NodeHandle node;

		///subs
		moveToNextPointSub_  = node.subscribe("/move_to_next_point", 1,
         	&WaypointDriverBase::waitForAckNextPointCallback, this); 

		waitForStartSub_  = node.subscribe("/start_mission", 1,
         	&WaypointDriverBase::waitForStartPathCallback, this); 

		commandSub_  = node.subscribe("/command_string", 1,
         	&WaypointDriverBase::commandCallback, this); 		

		pathSub_ = node.subscribe("/waypoints_route", 1,
         	&WaypointDriverBase::pathCallback, this);		 

		//pubs
		navigationDonePub_ = node.advertise<std_msgs::Bool>(
			"/navigation_done", 1, false);	


		current_goal_reached_publisher_= node.advertise<std_msgs::Bool>(
			"/current_goal_reached", 1, false);	

		goals_marker_array_publisher_ =
        	node.advertise<visualization_msgs::MarkerArray>("/waypoints_markers", 10);			

		log_publisher_ =
        	node.advertise<std_msgs::String>("/log", 10);	
		//timers
		pathMarkersTimer_ = node.createTimer(ros::Rate(1), 
                &WaypointDriverBase::markersPathCallback, this);	


		//rosparams			
		ros::NodeHandle node_p("~");	
		node_p.param("minimal_distance", min_distance_, -1.0);
		node_p.param("wait_for_pause_cmd_enable", wait_for_pause_cmd_enable, false);	
		node_p.param("go_to_next_point_before_reach_current",dontStopOnGoal_, false);
		node_p.param("max_distance_before_skip_current_goal", maxDistBeforeSkipCurrentGoal_, 0.3);

		canMoveNextPoint_ = false;

		currentPointReached_ = true;
	}

	virtual ~WaypointDriverBase(){}

	void setWayPoints(vector<geometry_msgs::PoseStamped> waypoints){

		waypoints_ = waypoints;
	}

	vector<geometry_msgs::PoseStamped> getWayPoints(vector<geometry_msgs::PoseStamped> waypoints){

		return waypoints_;
	}


protected:

	void markersPathCallback(const ros::TimerEvent&) {

		std::vector<geometry_msgs::PoseStamped> wayPointsForMarkers;

		if ( waypoints_.size() > 0 ) {

			
			visualization_msgs::MarkerArray markers;

			for (int i = 0; i < waypoints_.size(); i++) {

				visualization_msgs::Marker marker;
				marker.lifetime = ros::Duration(1.0);
				marker.action = visualization_msgs::Marker::ADD;
				marker.type = visualization_msgs::Marker::SPHERE;// ARROW;
				// marker.text = to_string(i);
				marker.header.frame_id = "map";
				marker.header.stamp  = ros::Time::now(); 
				marker.id = rand();
			
				marker.pose.position = waypoints_[i].pose.position;
				marker.pose.orientation = waypoints_[i].pose.orientation;
				marker.scale.x = 0.2;
				marker.scale.y = 0.2;                
				marker.scale.z = 0.2;
				if( visitedWaypoints_[i] == true){
					marker.color.r = 0.0f;
					marker.color.g = 1.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
			
				} else {
					marker.color.r = 1.0f;
					marker.color.g = 0.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
				}
				
				
			
				markers.markers.push_back(marker);              
			}

			goals_marker_array_publisher_.publish(markers);


		}



	}		


	void commandCallback(const std_msgs::StringConstPtr &msg){


		if( strcmp(msg->data.c_str(), "ABORT") == 0 ){

			abortMission_ = true;

			cerr<<"ABORT command has been sent "<<endl;

			return;
		}

		if(  strcmp(msg->data.c_str(), "PAUSE") == 0 ){

			pauseMission_ = true;

			cerr<<"PAUSE command has been sent "<<endl;

			return;

		}

		if( strcmp(msg->data.c_str(),  "CONTINUE") == 0  ){

			continueMission_ = true;

			cerr<<"CONTINUE command has been sent "<<endl;

			return;

		}

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

	
	void waitForAckNextPointCallback(const std_msgs::BoolConstPtr &msg){

		//can move to next point
		if( msg->data == true ){

			if(currentPointReached_ ){

				canMoveNextPoint_ = true;
				return;
			} else {

			}
		} else {			
			canMoveNextPoint_ = false;
		}
	}

	void waitForStartPathCallback(const std_msgs::BoolConstPtr &msg) {

		if( msg->data == true ){

			canStartMission_ = true;
		}
	}

	
	void pathCallback(const nav_msgs::Path::ConstPtr &msg)
	{
		getPathTopic_ = true;

		setWayPoints(msg->poses);

		visitedWaypoints_.resize(msg->poses.size(), false);


		cerr<<" got PATH!!" <<endl;
	}

	void publishLog(string s){

		std_msgs::String msg;
		msg.data = s;

		log_publisher_.publish(msg);
	}
	
protected:

	//members
	vector<geometry_msgs::PoseStamped> waypoints_;
	vector<bool> visitedWaypoints_;
	double min_distance_ = -1.0;

	//tf
	tf::TransformListener tfListener_;
	geometry_msgs::PoseStamped robotPose_;
	
	bool dontStopOnGoal_ = false;
	double maxDistBeforeSkipCurrentGoal_;

	bool currentPointReached_ = true;
	bool canMoveNextPoint_ = false;;
	bool navigationDone_ = false;
	bool canStartMission_ = false;
	bool abortMission_ = false;
	bool pauseMission_ = false;
	bool continueMission_ = false;
	bool getPath_ = true;
	bool wait_for_pause_cmd_enable = false;

	double waypointWait_ = 0.0;

	bool getPathTopic_ = false;

	//move-base
	MoveBaseController moveBaseController_;


	//subs
	ros::Subscriber moveToNextPointSub_;
	ros::Subscriber waitForStartSub_;
	ros::Subscriber commandSub_;
	ros::Subscriber pathSub_;

	ros::Timer systemTimer_;
	ros::Timer pathMarkersTimer_;	



	//pub
	ros::Publisher navigationDonePub_;
	ros::Publisher goals_marker_array_publisher_ ;
	ros::Publisher current_goal_reached_publisher_ ;
	ros::Publisher log_publisher_;

};

#endif /* INCLUDE_waypoints_navigation_WaypointDriverBase_H_ */
