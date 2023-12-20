/*
 * RobotStateEstimator.h
 *
 *  Created on: Dec 17, 2017
 *      Author: blackpc
 */

#ifndef SRC_IMU_ROBOTSTATEESTIMATOR_H_
#define SRC_IMU_ROBOTSTATEESTIMATOR_H_


#include <string>
#include <map>
#include <deque>
#include <numeric>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/moment.hpp>

#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

#include <hamster_driver/calibration/MotionDetector.h>


using namespace std;


/**
 * Estimates whether the robot is still or moving
 */
class RobotStateEstimator {

public:

	RobotStateEstimator();

	virtual ~RobotStateEstimator();

public:

	inline bool isStill() const {
		return !motionDetected_;
	}

private:

	using StatsAccumulator = boost::accumulators::accumulator_set<
			double, boost::accumulators::stats<
				boost::accumulators::tag::variance(boost::accumulators::lazy)> >;

private:

	void motorsCallback(
			const nav_msgs::Odometry::Ptr&);

	void motorsCommandCallback(
			const ackermann_msgs::AckermannDriveStamped::Ptr&);

	void imuCallback(const sensor_msgs::Imu::Ptr&);

	void motionDetected(const MotionDetectorType detectorType);

	void updateTimerCallback(const ros::TimerEvent&);

	/**
	 * Returns true if motion detected using gyro readings
	 * @param gyroReadings
	 * @return
	 */
	bool checkGyroMotion(const geometry_msgs::Vector3& gyroReadings);

	bool checkAccelMotion(const geometry_msgs::Vector3& accelReadings);

	/**
	 * Element-wise minimum
	 * @param a
	 * @param b
	 * @return
	 */
	inline tf::Vector3 min(const tf::Vector3& a, const tf::Vector3& b) {
		return tf::Vector3(
				(a.x(), b.x()),
				(a.y(), b.y()),
				(a.z(), b.z()));
	}

	/**
	 * Element-wise minimum
	 * @param a
	 * @param b
	 * @return
	 */
	inline tf::Vector3 updateStddev(const tf::Vector3& stddevOld, const tf::Vector3& stddevNew) {

		// If new deviation is less than current, then use it,
		// If it's bigger, increase the old deviation a bit

		double fraction = 0.001;
		double fractionDown = 0.05;

		// Very high value, real rest values are around 0.001
		double hardStddevLimit = 0.1;

		auto stddev = tf::Vector3(
						(stddevOld.x() > stddevNew.x() ? (stddevOld.x() * (1.0 - fractionDown) + stddevNew.x() * fractionDown) :
								(stddevOld.x() * (1.0 - fraction) + stddevNew.x() * fraction)),
						(stddevOld.y() > stddevNew.y() ? (stddevOld.y() * (1.0 - fractionDown) + stddevNew.y() * fractionDown) :
								(stddevOld.y() * (1.0 - fraction) + stddevNew.y() * fraction)),
						(stddevOld.z() > stddevNew.z() ? (stddevOld.z() * (1.0 - fractionDown) + stddevNew.z() * fractionDown) :
								(stddevOld.z() * (1.0 - fraction) + stddevNew.z() * fraction)));

		// Limit deivation
		if (stddev.x() > hardStddevLimit)
			stddev.setX(hardStddevLimit);

		if (stddev.y() > hardStddevLimit)
			stddev.setY(hardStddevLimit);

		if (stddev.z() > hardStddevLimit)
			stddev.setZ(hardStddevLimit);

		return stddev;
	}

	/**
	 * Set large value to minimal distribution
	 */
	void resetMinStddev();

private:

	/**
	 * List of all available motion detector,
	 * the robot is considered still, when all sources
	 * indicate no motion
	 */
	map<MotionDetectorType, MotionDetector> motionDetectors_;

	deque<tf::Vector3> accelReadingsBuffer_;

	tf::Vector3 accelMinStddev_;

	deque<tf::Vector3> gyroReadingsBuffer_;

	tf::Vector3 gyroMinStddev_;

	int motionBufferLength_ = 30;

	double motionSigmaThreshold_ = 6.0;

	/**
	 * Periodically reset minimal stddev
	 */
	double stddevResetInterval_ = 1.0;

	ros::Timer updateTimer_;

	ros::Subscriber motorsSubscriber_;

	ros::Subscriber motorsCommandSubscriber_;

	ros::Subscriber imuSubscriber_;

	ros::Publisher debugStddevThresholdPublisher_;

	ros::Publisher debugStddevCurrentPublisher_;

	ros::Publisher debugStatePublisher_;

	ros::Publisher debugMotionEventPublisher_;

	/**
	 * Robot's state
	 */
	bool motionDetected_ = true;

};


#endif /* SRC_IMU_ROBOTSTATEESTIMATOR_H_ */
