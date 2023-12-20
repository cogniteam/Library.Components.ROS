/**
 * Filename: Wandering.cpp
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

#include <lynx_navigation/Wandering.h>


Wandering::Wandering(const string& baseFrameId)
	: baseFrameId_(baseFrameId), publishStop_(false)
{   

    //
    // Will be overridden by configCallback()
    //
	createTrajectories(1.0, 0.1);
    
	trajectoryMatcher_ = new SimpleTrajectoryMatcher();
    configServer_.setCallback(boost::bind(&Wandering::configCallback, this, _1, _2));
    ros::spinOnce();
}


Wandering::~Wandering() {
	delete trajectoryMatcher_;
}

TrajectoryMatch::Ptr Wandering::chooseBestTrajectory(const nav_msgs::OccupancyGrid& costMap) {

	TrajectoryMatch::SetPtr frontMathces =
			trajectoryMatcher_->match(costMap, frontTrajectories_);

	return *frontMathces->begin();
}

void Wandering::spin() {
	ros::NodeHandle node;
	ros::NodeHandle nodePrivate("~");

	srand(time(0));

	/**
	 * Publishers
	 */
	pathPublisher_ = node.advertise<
            visualization_msgs::MarkerArray>("navigation_path", 100, false);

	auto ackermannPublisher = node.advertise<
            ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1, false);

	statePublisher_ = node.advertise<
            std_msgs::Bool>("events/navigation/state", 1, true);

    auto mapPublisher = node.advertise<nav_msgs::OccupancyGrid>(
            "perception/navigation/costmap", 1, false);

    finishedPublisher_ = node.advertise<std_msgs::Bool>(
            "events/navigation/finished", 1, true);

    auto distanceToGoalPublisher = node.advertise<std_msgs::Float64>(
            "perception/navigation/distance_to_goal", 1, false);

	auto stopSubscriber = node.subscribe(
            string("commands/navigation/stop"), 1, 
            &Wandering::stopCallback, this);

	auto goalSubscriber = node.subscribe(
            string("commands/navigation/goal"), 1, 
            &Wandering::goalCallback, this);

	auto gpsGoalSubscriber = node.subscribe(
            string("commands/navigation/goal_gps"), 1, 
            &Wandering::goalGpsCallback, this);

    auto costmapSubscriber = node.subscribe(
            "costmap_node/costmap/costmap", 1, &Wandering::costmapCallback, this);

    auto costmapUpdateSubscriber = node.subscribe(
            "costmap_node/costmap/costmap_updates", 1, 
            &Wandering::costmapUpdateCallback, this);

    objectSubscriber_ = node.subscribe("openvino_toolkit/detected_objects", 1,
            &Wandering::objectCallback, this);

    publishState();

	while (ros::ok()) {
		ros::spinOnce();

        if (!costmap_ || !enabled_) {

            if (publishStop_) {
				ackermannPublisher.publish(ackermann_msgs::AckermannDriveStamped());
				publishStop_ = false;
			}

            rate_.sleep();
            continue;
        }

        //
        // Check if detected person is gone
        //
        if (personDetected_ && 
                ((ros::Time::now() - personDetectionTime_).toSec() > config_.person_timeout)) {
            personDetected_ = false;
        }

        //
        // Check if goal reached
        //
        geometry_msgs::PoseStamped robotPose;
        robotPose.header.frame_id = baseFrameId_;
        robotPose.header.stamp = ros::Time(0);
        auto goal = trajectoryMatcher_->getGoal();
        goal.header.stamp = ros::Time(0);

        double distanceToGoal = trajectoryMatcher_->distance(robotPose, goal);

        std_msgs::Float64 distanceToGoalMessage;
        distanceToGoalMessage.data = distanceToGoal;
        distanceToGoalPublisher.publish(distanceToGoalMessage);

        if (distanceToGoal < config_.goal_tolerance) {
            ROS_INFO("Goal reached");
            stop(true); 
            continue;
        }

		// 
		// Evaluate trajectories
		// 
		TrajectoryMatch::Ptr bestMatch = chooseBestTrajectory(*costmap_);

		//
		// Publish all paths
		//
        publishTrajectories(bestMatch);

		//
		// Publish velocity command
		//
        ROS_INFO_ONCE("Navigation is running...");

        auto command = 
                bestMatch->getTrajectory()->getMotionModelAs<AckermannModel>()->getAckermannMessage();

        //
        // Check front obstacle
        //
        if (fabs(command.drive.steering_angle) > 0.001) {
            // Trajectory is not the front trajectory, this means the front trajectory is blocked
            frontObstacle_ = true;
        } else {
            frontObstacle_ = false;
        }

        
        // Set appropriate speed
        if (personDetected_) {
            command.drive.speed = config_.speed_person;
        } else if (frontObstacle_) {
            command.drive.speed = config_.speed_obstacle;
        } else {
            command.drive.speed = config_.speed;
        }

        // 
        // Best match is blocked -> all trajectories are blocked
        // Stop the vehicle
        // 
        if (bestMatch->isBlocked()) {
            command.drive.speed = 0.0;
        }

        command.drive.steering_angle *= config_.steer_scale;

        ackermannPublisher.publish(command);

        //
        // 
        //
        costmap_->header.stamp = ros::Time::now();
        mapPublisher.publish(costmap_);

		rate_.sleep();
	}

}

void Wandering::publishTrajectories(const TrajectoryMatch::Ptr& bestMatch) {
    visualization_msgs::MarkerArray pathMarker;
    auto now = ros::Time(0);

    auto allTrajectories = *frontTrajectories_;
    allTrajectories.push_back(bestMatch->getTrajectory());

    pathMarker.markers.resize(allTrajectories.size());

    for (int i = 0; i < allTrajectories.size(); ++i) {
        
        auto&& trajectory = allTrajectories[i];

        bool bestTrajectory = false;

        if (i == allTrajectories.size() - 1) {
            bestTrajectory = true;
        }

        pathMarker.markers[i].header.stamp = now;
        pathMarker.markers[i].header.frame_id = baseFrameId_;
        pathMarker.markers[i].id = i;
        pathMarker.markers[i].ns = "trajectories";
        pathMarker.markers[i].action = visualization_msgs::Marker::ADD;
        pathMarker.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        // pathMarker.markers[i].lifetime = ros::Duration(0.05 + 1.0 / 30.0);

        pathMarker.markers[i].color.a = 0.9;

        if (bestTrajectory) {
            pathMarker.markers[i].color.g = 1.0;
            pathMarker.markers[i].color.b = 0.2;
        } else {
            pathMarker.markers[i].color.g = 0.2;
            pathMarker.markers[i].color.b = 1.0;
        }

        if (bestTrajectory) {
            pathMarker.markers[i].scale.x = 0.2;    
        } else {
            pathMarker.markers[i].scale.x = 0.1;    
        }

        auto pointsCount = trajectory->getPath()->poses.size();
        pathMarker.markers[i].points.resize(pointsCount);

        for (int j = 0; j < pointsCount; j++) {
            pathMarker.markers[i].points[j] = 
                    trajectory->getPath()->poses[j].pose.position;
        }

    }

    pathPublisher_.publish(pathMarker);
}

void Wandering::createTrajectories(double simulationTime, double granularity) {
	TrajectorySimulator trajectorySimulator(simulationTime, granularity);

	Trajectory::Ptr trajectory;

	double MIN_ANGLE = -angles::from_degrees(config_.steer_range);
	double MAX_ANGLE = angles::from_degrees(config_.steer_range);
	int    TRAJECTORIES = fmax(config_.steer_trajectories, 1);

	double ANGLE_RANGE = -MIN_ANGLE + MAX_ANGLE;
	double ANGLE_STEP = ANGLE_RANGE / ((TRAJECTORIES == 1 ? 2 : TRAJECTORIES) - 1);

	frontTrajectories_ = Trajectory::VectorPtr(new Trajectory::Vector());

	for (double i = 0; i < TRAJECTORIES; ++i) {

		const double currentAngle = MIN_ANGLE + ANGLE_STEP * i;

		trajectory = trajectorySimulator.simulate(new AckermannModel(0.88, config_.speed, currentAngle));
		trajectory->setWeight(1.0);
		// trajectory->setWeight(1.0 - (0.05 * fabs(currentAngle) / (ANGLE_RANGE)));
        trajectory->setFrame(baseFrameId_);
		frontTrajectories_->push_back(trajectory);
	}

}

void Wandering::stop(bool goalReached) {
    enabled_ = false;
    publishStop_ = true;

    publishState();

    std_msgs::Bool finishMsg;
    finishMsg.data = goalReached;
    finishedPublisher_.publish(finishMsg);

    //
    // Clear visualization markers
    //
    visualization_msgs::MarkerArray pathMarker;
    pathMarker.markers.resize(config_.steer_trajectories + 1);

    for (int i = 0; i < 1; ++i) {
        pathMarker.markers[i].id = 1;
        pathMarker.markers[i].ns = "trajectories";
        pathMarker.markers[i].header.frame_id = baseFrameId_;
        pathMarker.markers[i].header.stamp = ros::Time::now();
        pathMarker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }

    pathPublisher_.publish(pathMarker);
}

void Wandering::stopCallback(const std_msgs::Empty::Ptr&) {
	stop(false);
}

 void Wandering::costmapUpdateCallback(
        const map_msgs::OccupancyGridUpdate::Ptr& map) {

    if (!costmap_) {
        return;
    }

    size_t x0 = static_cast<size_t>(map->x);
    size_t y0 = static_cast<size_t>(map->y);
    size_t xn = map->width + x0;
    size_t yn = map->height + y0;

    // lock as we are accessing raw underlying map

    size_t costmap_xn = costmap_->info.width;
    size_t costmap_yn = costmap_->info.height;

    // update map with data
    vector<int8_t>& costmap_data = costmap_->data;

    size_t i = 0;

    for (size_t y = y0; y < yn && y < costmap_yn; ++y)
    {
        for (size_t x = x0; x < xn && x < costmap_xn; ++x)
        {
            size_t idx =  y * costmap_->info.width + x;
            unsigned char cell_cost = static_cast<unsigned char>(map->data[i]);
            costmap_data[idx] = cell_cost;
            ++i;
        }
    }
}
