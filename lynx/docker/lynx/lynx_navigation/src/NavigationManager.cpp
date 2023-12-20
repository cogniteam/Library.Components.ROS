/**
 * Filename: NavigationManager.cpp
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


namespace lynx{


NavigationManager::NavigationManager() {
    ros::NodeHandle node;
    
    missionSubscriber_ = node.subscribe("commands/mission/start",
            1,&NavigationManager::missionCallback, this);
    
    finishEventSubscriber_ = node.subscribe("events/navigation/finished",
            1,&NavigationManager::finishEventCallback, this);
    
    stopStateSubscriber_ = node.subscribe("commands/mission/stop",
            1, &NavigationManager::stopStateCallback, this);
    
    goalPublisher_ = node.advertise<sensor_msgs::NavSatFix>(
            "commands/navigation/goal_gps", 1, false);
    
    stateIndicatorPublisher_ = node.advertise<std_msgs::Bool>(
            "events/mission/state", 1, true);
    
    stopCommandPublisher_ = node.advertise<std_msgs::Empty>(
            "commands/navigation/stop",1 ,false);

}

NavigationManager::~NavigationManager() {


}

void NavigationManager::sendNextWaypoint() {
    currentWaypoint_++;

    if (mission_.waypoints.size() > currentWaypoint_) {
        
        auto currentWaypoint = mission_.waypoints[currentWaypoint_];

        ROS_INFO(" - Next waypoint %s: lat %f, lon %f [%i/%i]", currentWaypoint.name.c_str(),
                currentWaypoint.latitude, currentWaypoint.longitude,
                currentWaypoint_ + 1, (int)mission_.waypoints.size());

        // Publish goal
        sensor_msgs::NavSatFix gps;
        gps.latitude = currentWaypoint.latitude;
        gps.longitude = currentWaypoint.longitude;
        
        goalPublisher_.publish(gps);

        setState(true);
    } else {
        // Index out of range - finished
        setState(false);
    }
}

void NavigationManager::startMission(const lynx_msgs::Mission& mission) {
    
    ROS_INFO("Starting mission '%s'", mission.name.c_str());

    currentWaypoint_ = -1;
    mission_ = mission;

    if (mission.waypoints.size() == 0) {
        // TODO Stop navigation
        setState(false);
        return;
    }

    sendNextWaypoint();
}

void NavigationManager::missionCallback(const lynx_msgs::Mission::Ptr& mission) {
    startMission(*mission);
}

void NavigationManager::waypointSleepTimerCallback(const ros::TimerEvent&) {
    sendNextWaypoint();
}

void NavigationManager::finishEventCallback(const std_msgs::BoolConstPtr& msg) {
    if (!running_) {
        return;
    }
    
    if (msg->data) {
        ROS_INFO("Goal is reached");
        
        if (currentWaypoint_ == (mission_.waypoints.size() - 1)) {
            setState(false);
            ROS_INFO("Mission finished");
            return;
        }
        auto sleepTime = mission_.waypoints[currentWaypoint_].wait_duration;

        //
        // Sleep if needed
        //
        if (sleepTime > 0) {
            ROS_INFO("Sleeping for %f seconds", sleepTime);
            ros::NodeHandle node;
            waypointSleepTimer_ = node.createTimer(ros::Duration(sleepTime),
                    &NavigationManager::waypointSleepTimerCallback, this, true, true);
            return;
        }
        
        sendNextWaypoint();

    } else {
        ROS_INFO("Failed to reach goal");
        setState(false);
    }
}

void NavigationManager::stopStateCallback(const std_msgs::EmptyConstPtr& msg) {
    ROS_INFO("Mission stop request received");
    ROS_INFO("Stopping navigation...");
    stopCommandPublisher_.publish(msg);
    setState(false);
    ROS_INFO("Mission stopped");
}

void NavigationManager::setState(bool state) {
    running_ = state;
    std_msgs::Bool s;
    s.data = state;
    stateIndicatorPublisher_.publish(s);
}

} // lynx namespace
