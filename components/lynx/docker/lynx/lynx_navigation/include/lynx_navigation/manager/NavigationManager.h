/**
 * Filename: NavigationManager.h
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

#include <ros/ros.h>
#include <fstream>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>

#include <lynx_msgs/Mission.h>


namespace lynx{


using namespace std;


class NavigationManager {

public:

    NavigationManager();

    virtual ~NavigationManager();

public: 

    void startMission(const lynx_msgs::Mission& mission);

private:

    void missionCallback(const lynx_msgs::Mission::Ptr&);

    void finishEventCallback(const std_msgs::BoolConstPtr&);

    void stopStateCallback(const std_msgs::EmptyConstPtr&);

    void setState(bool state);

    void sendNextWaypoint();

    void waypointSleepTimerCallback(const ros::TimerEvent&);

private:

    ros::Subscriber missionSubscriber_;

    ros::Subscriber finishEventSubscriber_;
    
    ros::Subscriber stopStateSubscriber_;
    
    ros::Publisher goalPublisher_;

    ros::Publisher stateIndicatorPublisher_;

    ros::Publisher stopCommandPublisher_;

    lynx_msgs::Mission mission_;

    int currentWaypoint_ = -1;

    bool running_ = false;

    ros::Timer waypointSleepTimer_;

};

} // namespace lynx