/*
 * PathPublisher.h
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 */

#ifndef INCLUDE_path_publisher_H_
#define INCLUDE_path_publisher_H_


#include <vector>
#include<string.h>
#include <tf/tf.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;




class PathPublisher {



public:

	PathPublisher(vector<geometry_msgs::PoseStamped> waypoints){

		waypoints_ = waypoints;

		ros::NodeHandle node;

		ros::NodeHandle node_p("~");

		/// subs
		sendPathSub_  = node.subscribe("/send_path", 1,
         	&PathPublisher::sendPathCallback, this);	

		//pubs
			pathPub_ = node.advertise<nav_msgs::Path>(
			"/polygon_path", 1, false);				

		//timers
		missionTimer_ = node.createTimer(ros::Rate(30), 
                &PathPublisher::missionTimerCallback, this);	
	}

	virtual ~PathPublisher(){}

	void sendPathCallback(const std_msgs::BoolConstPtr &msg){

		if( msg->data == true){
	
			cerr<<" user ask to send Path !!!!!!!!!!!!! "<<endl;

			canSendPath_ = true;
	
		}
	}	

	void setWayPoints(vector<geometry_msgs::PoseStamped> waypoints){

		waypoints_ = waypoints;
	}

	

private:
	
	void sendPath(){

		nav_msgs::Path msgMsg;
		msgMsg.header.frame_id ="map";
		msgMsg.header.stamp = ros::Time::now();
		msgMsg.poses = waypoints_;

		pathPub_.publish(msgMsg);

		ros::Duration(1).sleep();

	}

	

	void missionTimerCallback(const ros::TimerEvent&) {

		cerr<<"Beginning of timer callback"<<endl;

		if( canSendPath_ ) {
			
			canSendPath_ = false;
			
			sendPath();

		}
	}
	
	

private:

	//timers
	ros::Timer missionTimer_;

	//subs
	ros::Subscriber sendPathSub_;

	ros::Publisher  pathPub_;
	
	// system params
	
	vector<geometry_msgs::PoseStamped> waypoints_;

	bool canSendPath_ = false;

};

#endif /* INCLUDE_path_publisher_H_ */
