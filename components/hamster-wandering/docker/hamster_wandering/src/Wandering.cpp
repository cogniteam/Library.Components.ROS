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

#include <hamster_wandering/Wandering.h>

Wandering::Wandering(const string& robotId,
		const string& baseFrameId, double maxVelocity, bool enabled)
	: robotId_(robotId), baseFrameId_(baseFrameId), maxVelocity_(maxVelocity),
	  enabled_(enabled), publishStop_(false), preferRight_(true)
{
	trajectoryMatcher_ = new SimpleTrajectoryMatcher();
	preferSideChangeTime_ = ros::Time::now();
}


Wandering::~Wandering() {
	delete trajectoryMatcher_;
}

TrajectoryMatch::Ptr Wandering::chooseBestTrajectory(CostMap& costMap) {

	Trajectory::Ptr rotateInPlaceTrajectory(new Trajectory(new AckermannModel(0.1, 0, 1.0)));


	TrajectoryMatch::SetPtr frontMathces =
			trajectoryMatcher_->match(costMap, frontTrajectories_);

	if ((*frontMathces->begin())->isBlocked()) {
		// All front blocked, rotate till front is clean
		return TrajectoryMatch::Ptr(new TrajectoryMatch(rotateInPlaceTrajectory, 1.0));
	}

	return *frontMathces->begin();
}

void Wandering::spin() {
	ros::NodeHandle nodePrivate("~");

	srand(time(0));

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/scan");
	CostMap costMap(laserScanDataSource, new RosParametersProvider());

	/**
	 * Publishers
	 */
	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("costmap", 1, false);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("path", 1, false);
	ros::Publisher bestPathPublisher = nodePrivate.advertise<nav_msgs::Path>("path_best", 1, false);
	ros::Publisher ackermannPublisher = nodePrivate.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, false);
	ros::Subscriber stateSubscriber = nodePrivate.subscribe(string("/decision_making/" + robotId_ + "/events"), 1, &Wandering::stateCallback, this);

	createTrajectories(2.0, 0.05);

	ros::Rate rate(9);
	enabled_ = true;
	while (ros::ok()) {
		ros::spinOnce();
		if (!enabled_) {

			if (publishStop_) {
				ackermannPublisher.publish(ackermann_msgs::AckermannDriveStamped());
				publishStop_ = 0;
			}

			rate.sleep();
			continue;
		}

		/**
		 * Evaluate trajectories
		 */
		TrajectoryMatch::Ptr bestMatch = chooseBestTrajectory(costMap);

		/**
		 * Publish all paths
		 */
//		for (int i = 0; i < frontTrajectories_->size(); ++i) {
//			pathPublisher.publish((*frontTrajectories_)[i]->getPath(true, baseFrameId_));
//			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
//		}

		/**
		 * Publish best matched trajectory
		 */
		bestPathPublisher.publish(bestMatch->getTrajectory()->getPath(true, baseFrameId_));

		/**
		 * Publish velocity command
		 */
		ackermannPublisher.publish(bestMatch->getTrajectory()->getMotionModelAs<AckermannModel>()->getAckermannMessage());

		/**
		 * Publish local cost map
		 */
		mapPublisher.publish(costMap.getOccupancyGrid());

		rate.sleep();
	}

}

void Wandering::createTrajectories(double simulationTime, double granularity) {
	TrajectorySimulator trajectorySimulator(simulationTime, granularity);

	Trajectory::Ptr trajectory;

	static const double MIN_ANGLE = -0.25;
	static const double MAX_ANGLE = 0.25;
	static const int    TRAJECTORIES = 9;

	static const double ANGLE_RANGE = -MIN_ANGLE + MAX_ANGLE;
	static const double ANGLE_STEP = ANGLE_RANGE / (TRAJECTORIES - 1);

	frontTrajectories_ = Trajectory::VectorPtr(new Trajectory::Vector());

	for (double i = 0; i < TRAJECTORIES; ++i) {

		const double currentAngle = MIN_ANGLE + ANGLE_STEP * i;

		trajectory = trajectorySimulator.simulate(new AckermannModel(0.155, maxVelocity_, currentAngle));
		trajectory->setWeight(1.0 - (fabs(currentAngle) / (ANGLE_RANGE)));
		frontTrajectories_->push_back(trajectory);
	}

	frontTrajectory_ = (*frontTrajectories_)[TRAJECTORIES / 2];
}

void Wandering::stateCallback(const std_msgs::String::Ptr& message) {
	if (message->data == "RESUME") {
		enabled_ = true;
		publishStop_ = false;
		ROS_INFO("Started!");
	} else if (message->data == "PAUSE") {

		if (enabled_)
			publishStop_ = true;

		enabled_ = false;
		ROS_INFO("Stoped!");
	}
}
