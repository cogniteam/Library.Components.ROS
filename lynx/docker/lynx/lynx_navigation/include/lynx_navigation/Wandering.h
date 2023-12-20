/**
 * Filename: Wandering.h
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
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

#ifndef INCLUDE_WANDERER_WANDERING_H_
#define INCLUDE_WANDERER_WANDERING_H_


#include <time.h>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <angles/angles.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>

#include <object_msgs/ObjectsInBoxes.h>

#include <lynx_navigation/NavigationConfig.h>
#include <lynx_navigation/trajectory/simulator/TrajectorySimulator.h>
#include <lynx_navigation/trajectory/matcher/SimpleTrajectoryMatcher.h>
#include <lynx_navigation/trajectory/simulator/models/AckermannModel.h>


using namespace std;


#define foreach BOOST_FOREACH


/*
 * Simple wandering algorithm
 */
class Wandering {

public:

	Wandering(const string& baseFrameId);

	virtual ~Wandering();

public:

	void spin();

private:

	TrajectoryMatch::Ptr chooseBestTrajectory(
            const nav_msgs::OccupancyGrid& costMap);

	void createTrajectories(
			double simulationTime, double granularity);

	void stopCallback(const std_msgs::Empty::Ptr&);

    void goalCallback(const geometry_msgs::PoseStamped::Ptr& pose) {
        ROS_INFO("Goal received");
        trajectoryMatcher_->setGoal(*pose);
        enabled_ = true;

        publishState();
    }

    void goalGpsCallback(const sensor_msgs::NavSatFix::Ptr& gps) {

        geodesy::UTMPoint utmPoint;

        geographic_msgs::GeoPoint geoPoint;
        geoPoint.latitude = gps->latitude;
        geoPoint.longitude = gps->longitude;
        geoPoint.altitude = 0;

        geodesy::fromMsg(geoPoint, utmPoint);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "utm";
        goal.header.stamp = ros::Time(0);
        goal.pose.orientation.w = 1.0;
        goal.pose.position.x = utmPoint.easting;
        goal.pose.position.y = utmPoint.northing;
        goal.pose.position.z = 0;

        trajectoryMatcher_->setGoal(goal);
        enabled_ = true;

        ROS_INFO("GPS Goal received LatLon(%f, %f), XY(%f, %f)", gps->latitude, gps->longitude,
                 goal.pose.position.x, goal.pose.position.y);

        publishState();
    }

    void costmapCallback(const nav_msgs::OccupancyGrid::Ptr& costmap) {
        costmap_ = costmap;
    }

    void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::Ptr& map);

    void publishState() {
        std_msgs::Bool state;
        state.data = enabled_;
        statePublisher_.publish(state);
    }

    void configCallback(lynx_navigation::NavigationConfig& config, 
            uint32_t level) {
        config_ = config;
        rate_ = ros::Rate(config_.rate);

        createTrajectories(config_.simulation_time, config_.simulation_step);

        ROS_INFO("Config received");
    }

    void stop(bool goalReached = false);

    void publishTrajectories(const TrajectoryMatch::Ptr& bestMatch);

    void objectCallback(const object_msgs::ObjectsInBoxes::Ptr& objects) {
        for (auto&& object : objects->objects_vector) {
            if (object.object.object_name == "person") {
                if (object.roi.width > config_.person_min_width && 
                        object.roi.height > config_.person_min_height) {
                    personDetected_ = true;
                    personDetectionTime_ = ros::Time::now();
                }
            }
        }
    }

private:

    /**
     * Is front trajectory is blocked
     */
    bool frontObstacle_ = false;

    bool personDetected_ = false;

    ros::Time personDetectionTime_;

    ros::Rate rate_ = ros::Rate(30);

    ros::Publisher finishedPublisher_;

    ros::Publisher pathPublisher_;

    ros::Subscriber objectSubscriber_;

	bool enabled_ = false;

	bool publishStop_ = false;

	string baseFrameId_;

    ros::Publisher statePublisher_;

	SimpleTrajectoryMatcher* trajectoryMatcher_;

	Trajectory::VectorPtr frontTrajectories_;

    nav_msgs::OccupancyGrid::Ptr costmap_;

    dynamic_reconfigure::Server<lynx_navigation::NavigationConfig> configServer_;
    lynx_navigation::NavigationConfig config_;

    tf::TransformListener tfListener_;

};

#endif /* INCLUDE_WANDERER_WANDERING_H_ */
