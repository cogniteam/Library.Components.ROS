/*
 * AutoImuCalibrator.cpp
 *
 *  Created on: Dec 20, 2017
 *      Author: blackpc
 */

#include <hamster_driver/calibration/AutoImuCalibrator.h>


AutoImuCalibrator::AutoImuCalibrator() {

	ros::NodeHandle node;

	imuSubscriber_ = node.subscribe("imu/biased", 1,
			&AutoImuCalibrator::imuCallback, this);

	calibratedEventPublisher_ = node.advertise<geometry_msgs::Vector3>(
			"events/auto_imu_calibrator/calibrated", 1, true);

}

AutoImuCalibrator::~AutoImuCalibrator() {

}

void AutoImuCalibrator::imuCallback(
		const sensor_msgs::Imu::Ptr& imu) {

	try {

		if (robotState_.isStill()) {

			if (std::isnan(imu->angular_velocity.x)) {
				imu->angular_velocity.x = 0;
				ROS_WARN("nan in angular_velocity.x");
			}

			if (std::isnan(imu->angular_velocity.y)) {
				imu->angular_velocity.y = 0;
				ROS_WARN("nan in angular_velocity.y");
			}

			if (std::isnan(imu->angular_velocity.z)) {
				imu->angular_velocity.z = 0;
				ROS_WARN("nan in angular_velocity.z");
			}

			gyroBuffer_.push_back(
					tf::Vector3(
							imu->angular_velocity.x,
							imu->angular_velocity.y,
							imu->angular_velocity.z));


			// ROS_INFO("Gyro reading added [%i/%i]",
			// 		(int)gyroBuffer_.size(), gyroBufferLength_);

			if (gyroBuffer_.size() == gyroBufferLength_) {
				recalibrate(gyroBuffer_);
				gyroBuffer_.clear();

				// Reset accumulators
				gyroAccumulatorX_ = GyroAccumulator();
				gyroAccumulatorY_ = GyroAccumulator();
				gyroAccumulatorZ_ = GyroAccumulator();

				ROS_INFO("Gyro recalibrated");
			}
		}

	} catch (...) {
		ROS_ERROR("Unexpected error in AutoImuCalibrator::imuCallback()");
	}

}

void AutoImuCalibrator::recalibrate(
		vector<tf::Vector3> gyroReadings) {

	bool calibrationEnabled = true;

	ros::param::get("hamster_driver/imu_calibration/enabled", calibrationEnabled);

	if (calibrationEnabled == false) {
		ROS_INFO("Auto IMU calibration disabled by user "
				"[via ROS parameter hamster_driver/imu_calibration/enabled]");
		return;
	}

	for (const auto& g : gyroReadings) {
		gyroAccumulatorX_(g.x());
		gyroAccumulatorY_(g.y());
		gyroAccumulatorZ_(g.z());
	}

	double stddevX = sqrt((double)boost::accumulators::variance(gyroAccumulatorX_));
	double stddevY = sqrt((double)boost::accumulators::variance(gyroAccumulatorY_));
	double stddevZ = sqrt((double)boost::accumulators::variance(gyroAccumulatorZ_));

	double meanX = (double)boost::accumulators::mean(gyroAccumulatorX_);
	double meanY = (double)boost::accumulators::mean(gyroAccumulatorY_);
	double meanZ = (double)boost::accumulators::mean(gyroAccumulatorZ_);

	ros::param::set("gyro_bias_x", meanX);
	ros::param::set("gyro_bias_y", meanY);
	ros::param::set("gyro_bias_z", meanZ);

	ros::param::set("gyro_stddev_x", stddevX);
	ros::param::set("gyro_stddev_y", stddevY);
	ros::param::set("gyro_stddev_z", stddevZ);

	geometry_msgs::Vector3 msg;
	msg.x = meanX;
	msg.y = meanY;
	msg.z = meanZ;

	// ROS_INFO("Biases (gyroscopes):");
	// ROS_INFO(" - x = %f (stddev %f)", meanX, stddevX);
	// ROS_INFO(" - y = %f (stddev %f)", meanY, stddevY);
	// ROS_INFO(" - z = %f (stddev %f)", meanZ, stddevZ);

	calibratedEventPublisher_.publish(msg);
}
