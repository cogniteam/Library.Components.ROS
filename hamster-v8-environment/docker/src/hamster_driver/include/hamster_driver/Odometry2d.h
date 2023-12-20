/*
 * Odometry.h
 *
 *  Created on: Dec 11, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_HAMSTER_DRIVER_ODOMETRY2D_H_
#define INCLUDE_HAMSTER_DRIVER_ODOMETRY2D_H_


#include <ros/ros.h>


class Odometry2d {

public:

	Odometry2d() {
		firstReading_ = true;

		heading_ = 0;
		x_ = 0;
		y_ = 0;

		angularVelocity_ = 0;
		speed_ = 0;
	}

	virtual ~Odometry2d() {

	}

	void updateHeading(double angularVelocity) {
		if (firstReading_) {
			return;
		}

		double timeDelta = (ros::Time::now() - headingUpdateTime_).toSec();

		heading_ += angularVelocity * timeDelta;
		angularVelocity_ = angularVelocity;

		headingUpdateTime_ = ros::Time::now();


	}

	void update(double speed) {

		speed_ = speed;

		if (firstReading_) {
			firstReading_ = false;
			poseUpdateTime_ = ros::Time::now();
			headingUpdateTime_ = poseUpdateTime_;

			return;
		}

		double timeDelta = (ros::Time::now() - poseUpdateTime_).toSec();

		double distance = speed * timeDelta;

		double heading;

		// Rotate offset vector by initial heading
		float ca = cos(heading_);
		float sa = sin(heading_);
		float rx = ca * distance;
		float ry = sa * distance;

		// Add offset to pose
		x_ 			+= rx;
		y_ 			+= ry;

		poseUpdateTime_ = ros::Time::now();
	}

	inline double getX() const {
		return x_;
	}

	inline double getY() const {
		return y_;
	}

	inline double getHeading() const {
		return heading_;
	}

	inline double getAngularVelocity() const {
		return angularVelocity_;
	}

	inline double getSpeed() const {
		return speed_;
	}

private:

	bool firstReading_;

	ros::Time poseUpdateTime_;
	ros::Time headingUpdateTime_;

	double heading_;
	double x_;
	double y_;
	double angularVelocity_;
	double speed_;

};

#endif /* INCLUDE_HAMSTER_DRIVER_ODOMETRY2D_H_ */
