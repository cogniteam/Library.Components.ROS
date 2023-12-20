/*
 * AutoImuCalibrator.h
 *
 *  Created on: Dec 20, 2017
 *      Author: blackpc
 */

#ifndef SRC_IMU_AUTOIMUCALIBRATOR_H_
#define SRC_IMU_AUTOIMUCALIBRATOR_H_


#include <vector>

#include <geometry_msgs/Vector3.h>

#include <hamster_driver/calibration/RobotStateEstimator.h>


using namespace boost::accumulators;


class AutoImuCalibrator {

public:

	AutoImuCalibrator();

	virtual ~AutoImuCalibrator();

public:

private:

	using GyroAccumulator = accumulator_set<double, stats<tag::variance(lazy)> >;

private:

	void imuCallback(const sensor_msgs::Imu::Ptr&);

	void recalibrate(vector<tf::Vector3> gyroReadings);

private:

	ros::Subscriber imuSubscriber_;

	ros::Publisher calibratedEventPublisher_;

	RobotStateEstimator robotState_;

	vector<tf::Vector3> gyroBuffer_;

	int gyroBufferLength_ = 50;

	GyroAccumulator gyroAccumulatorX_;

	GyroAccumulator gyroAccumulatorY_;
    
	GyroAccumulator gyroAccumulatorZ_;

};

#endif /* SRC_IMU_AUTOIMUCALIBRATOR_H_ */
