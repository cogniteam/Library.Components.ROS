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

#include <std_msgs/String.h>

#include <hamster_wandering/costmap/CostMap.h>
#include <hamster_wandering/costmap/datasource/LaserScanDataSource.h>
#include <hamster_wandering/costmap/parameters/RosParametersProvider.h>
#include <hamster_wandering/trajectory/simulator/TrajectorySimulator.h>
#include <hamster_wandering/trajectory/matcher/SimpleTrajectoryMatcher.h>
#include <hamster_wandering/trajectory/simulator/models/SkidSteerModel.h>
#include <hamster_wandering/trajectory/simulator/models/AckermannModel.h>


using namespace std;
using namespace cv;


#define foreach BOOST_FOREACH


/*
 * Simple wandering algorithm
 */
class Wandering {

public:

	Wandering(const string& robotId, const string& baseFrameId, double maxVelocity, bool enabled);
	virtual ~Wandering();

public:

	void spin();

private:

	string robotId_;
	double maxVelocity_;
	bool enabled_;
	bool publishStop_;
	string baseFrameId_;

	ros::Time preferSideChangeTime_;
	bool preferRight_;

	ITrajectoryMatcher* trajectoryMatcher_;

	Trajectory::VectorPtr frontTrajectories_;
	Trajectory::Ptr frontTrajectory_;

private:

	TrajectoryMatch::Ptr chooseBestTrajectory(CostMap& costMap);

	void createTrajectories(
			double simulationTime, double granularity);

	void stateCallback(const std_msgs::String::Ptr& message);
};

#endif /* INCLUDE_WANDERER_WANDERING_H_ */
