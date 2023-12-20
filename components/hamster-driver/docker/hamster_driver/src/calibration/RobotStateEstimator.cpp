/*
 * RobotStateEstimator.cpp
 *
 *  Created on: Dec 17, 2017
 *      Author: blackpc
 */

#include <hamster_driver/calibration/RobotStateEstimator.h>


RobotStateEstimator::RobotStateEstimator() {
	ros::NodeHandle node;

	imuSubscriber_ = node.subscribe("imu/biased", 1,
			&RobotStateEstimator::imuCallback, this);

	motorsSubscriber_ = node.subscribe("odom",
			1, &RobotStateEstimator::motorsCallback, this);

	motorsCommandSubscriber_ = node.subscribe("ackermann_cmd",
			1, &RobotStateEstimator::motorsCommandCallback, this);

	debugStddevThresholdPublisher_ = node.advertise<geometry_msgs::Vector3>(
			"debug/imu_calibrator/gyro_stddev_threshold", 1, true);

	debugStddevCurrentPublisher_ = node.advertise<geometry_msgs::Vector3>(
			"debug/imu_calibrator/gyro_stddev_current", 1, true);

	debugStatePublisher_ = node.advertise<std_msgs::String>(
			"debug/imu_calibrator/state", 1, true);
	
	debugMotionEventPublisher_ = node.advertise<std_msgs::String>(
			"debug/imu_calibrator/events", 1, true);

	// Set all sources to indicate motion as we don't know
	// anything right now
	motionDetectors_[MotionDetectorType::Accelerometers]
					 = MotionDetector(MotionDetectorType::Accelerometers);

	motionDetectors_[MotionDetectorType::Gyroscopes]
					 = MotionDetector(MotionDetectorType::Gyroscopes);

	motionDetectors_[MotionDetectorType::Motors]
					 = MotionDetector(MotionDetectorType::Motors);

	motionDetectors_[MotionDetectorType::MotorsCommand]
					 = MotionDetector(MotionDetectorType::MotorsCommand);

	updateTimer_ = node.createTimer(ros::Rate(10),
			&RobotStateEstimator::updateTimerCallback, this);

	resetMinStddev();
}

RobotStateEstimator::~RobotStateEstimator() {

}

void RobotStateEstimator::resetMinStddev() {
	accelMinStddev_ = tf::Vector3(1.0, 1.0, 1.0);
	gyroMinStddev_ = tf::Vector3(0.001, 0.001, 0.001);

	ROS_DEBUG("Min Stddev reset");
}

void RobotStateEstimator::motorsCallback(
		const nav_msgs::Odometry::Ptr& motors) {
	if (fabs(motors->twist.twist.linear.x) > 0.001) {
		motionDetected(MotionDetectorType::Motors);
	}
}

void RobotStateEstimator::motorsCommandCallback(
		const ackermann_msgs::AckermannDriveStamped::Ptr& command) {
	if (fabs(command->drive.speed) > 0.001 || fabs(command->drive.steering_angle) > 0.001) {
		motionDetected(MotionDetectorType::MotorsCommand);
	}
}

void RobotStateEstimator::imuCallback(const sensor_msgs::Imu::Ptr& imu) {
	
	if (isnan(imu->linear_acceleration.x))
		imu->linear_acceleration.x = 0.0;
	
	if (isnan(imu->linear_acceleration.y))
		imu->linear_acceleration.y = 0.0;
	
	if (isnan(imu->linear_acceleration.z))
		imu->linear_acceleration.z = 0.0;
		
	if (isnan(imu->angular_velocity.x))
		imu->angular_velocity.x = 0.0;
	
	if (isnan(imu->angular_velocity.y))
		imu->angular_velocity.y = 0.0;
	
	if (isnan(imu->angular_velocity.z))
		imu->angular_velocity.z = 0.0;

	try {

		bool gyroMotionDetected = checkGyroMotion(imu->angular_velocity);
		bool accelMotionDetected = checkAccelMotion(imu->linear_acceleration);

		if (gyroMotionDetected) {
			motionDetected(MotionDetectorType::Gyroscopes);
		}

		if (accelMotionDetected) {
			motionDetected(MotionDetectorType::Accelerometers);
		}

	} catch (...) {
		ROS_ERROR("Unexpected error in RobotStateEstimator::imuCallback()");
		return;
	}
}

void RobotStateEstimator::motionDetected(const MotionDetectorType detectorType) {
	auto& detector = motionDetectors_[detectorType];
	detector.setMotionDetected(true);

	ROS_DEBUG("Motion detected by '%s'", MotionDetector::toString(detectorType));

	std_msgs::String eventMessage;
	eventMessage.data = "Motion detected by '" + 
			string(MotionDetector::toString(detectorType)) + "'";
	debugMotionEventPublisher_.publish(eventMessage);
}

void RobotStateEstimator::updateTimerCallback(const ros::TimerEvent&) {

	auto now = ros::Time::now();
	bool motionDetected = false;

	stringstream output;

	ROS_DEBUG("Update timer called, checking all detectors...");
	output << "Update timer called, checking all detectors..." << endl;

	for(auto& detector : motionDetectors_) {

		auto secsSinceLastDetection =
				(now - detector.second.getMotionDetectionTime()).toSec();

		ROS_DEBUG(" - Last motion detection of '%s': %fsec ago",
				MotionDetector::toString(detector.first),
				secsSinceLastDetection);

		output << " - Last motion detection of '" 
				<< MotionDetector::toString(detector.first) 
				<< "': " << secsSinceLastDetection 
				<< "sec ago" << endl;

		if (secsSinceLastDetection < 1.0) {
			motionDetected = true;
		}

	}

	if (motionDetected_ != motionDetected && (motionDetected == false)) {
		std_msgs::String eventMessage;
		eventMessage.data = "Robot is still again";
		debugMotionEventPublisher_.publish(eventMessage);
	}

	motionDetected_ = motionDetected;

	ROS_DEBUG("Robot's state: %s", (motionDetected_ ? "Moving" : "Still"));
	output << "Robot's state: %s" << (motionDetected_ ? "Moving" : "Still") << endl;

	std_msgs::String stateString;
	stateString.data = output.str();
	debugStatePublisher_.publish(stateString);
}

bool RobotStateEstimator::checkGyroMotion(
		const geometry_msgs::Vector3& gyroReadings) {

	tf::Vector3 gyroVector(gyroReadings.x,
			gyroReadings.y, gyroReadings.z);

	if (gyroReadingsBuffer_.size() < motionBufferLength_) {
		// Wait for buffer to fill
		gyroReadingsBuffer_.push_back(gyroVector);
		return true;
	}


	boost::array<StatsAccumulator, 3> accumulators;
 
	for(auto a : gyroReadingsBuffer_) {
		accumulators[0](a.x());
		accumulators[1](a.y());
		accumulators[2](a.z());
	}

	tf::Vector3 mean(boost::accumulators::mean(accumulators[0]),
			boost::accumulators::mean(accumulators[1]),
			boost::accumulators::mean(accumulators[2]));

	tf::Vector3 stddev(sqrt(boost::accumulators::variance(accumulators[0])),
			sqrt(boost::accumulators::variance(accumulators[1])),
			sqrt(boost::accumulators::variance(accumulators[2])));

	if (isnan(stddev.x())) {
		stddev.setX(0.0);
	}

	if (isnan(stddev.y())) {
		stddev.setY(0.0);
	}

	if (isnan(stddev.z())) {
		stddev.setZ(0.0);
	}

//	gyroMinStddev_ = min(gyroMinStddev_, stddev);
	gyroMinStddev_ = updateStddev(gyroMinStddev_, stddev);


	geometry_msgs::Vector3 gyroStddevMsg;
	gyroStddevMsg.x = gyroMinStddev_.x() * motionSigmaThreshold_;
	gyroStddevMsg.y = gyroMinStddev_.y() * motionSigmaThreshold_;
	gyroStddevMsg.z = gyroMinStddev_.z() * motionSigmaThreshold_;
	debugStddevThresholdPublisher_.publish(gyroStddevMsg);


	gyroStddevMsg.x = stddev.x();
	gyroStddevMsg.y = stddev.y();
	gyroStddevMsg.z = stddev.z();
	debugStddevCurrentPublisher_.publish(gyroStddevMsg);


	auto gyroZeroBias = gyroVector - mean;

	gyroReadingsBuffer_.push_back(gyroVector);
	gyroReadingsBuffer_.pop_front();

	if (fabs(gyroZeroBias.x()) > (gyroMinStddev_.x() * motionSigmaThreshold_) ||
			fabs(gyroZeroBias.y()) > (gyroMinStddev_.y() * motionSigmaThreshold_) ||
			fabs(gyroZeroBias.z()) > (gyroMinStddev_.z() * motionSigmaThreshold_)) {
		return true;
	}

	return false;
}

bool RobotStateEstimator::checkAccelMotion(
		const geometry_msgs::Vector3& accelReadings) {

	tf::Vector3 accelVector(accelReadings.x,
			accelReadings.y, accelReadings.z);


	if (accelReadingsBuffer_.size() < motionBufferLength_) {
		// Wait for buffer to fill
		accelReadingsBuffer_.push_back(accelVector);
		return true;
	}


	boost::array<StatsAccumulator, 3> accumulators;

	for(auto a : accelReadingsBuffer_) {
		accumulators[0](a.x());
		accumulators[1](a.y());
		accumulators[2](a.z());
	}

	tf::Vector3 mean(boost::accumulators::mean(accumulators[0]),
			boost::accumulators::mean(accumulators[1]),
			boost::accumulators::mean(accumulators[2]));

	tf::Vector3 stddev(sqrt(boost::accumulators::variance(accumulators[0])),
			sqrt(boost::accumulators::variance(accumulators[1])),
			sqrt(boost::accumulators::variance(accumulators[2])));

	if (isnan(stddev.x())) {
		stddev.setX(0.0);
	}

	if (isnan(stddev.y())) {
		stddev.setY(0.0);
	}

	if (isnan(stddev.z())) {
		stddev.setZ(0.0);
	}

	accelMinStddev_ = min(accelMinStddev_, stddev);

	ROS_DEBUG("Standard deviation (accelerometers):");
	ROS_DEBUG(" - x = %f (%f)", accelMinStddev_.x(), mean.x());
	ROS_DEBUG(" - y = %f (%f)", accelMinStddev_.y(), mean.y());
	ROS_DEBUG(" - z = %f (%f)", accelMinStddev_.z(), mean.z());

	auto accelZeroMean = accelVector - mean;

	accelReadingsBuffer_.push_back(accelVector);
	accelReadingsBuffer_.pop_front();

	if (fabs(accelZeroMean.x()) > accelMinStddev_.x() * motionSigmaThreshold_ ||
			fabs(accelZeroMean.y()) > accelMinStddev_.y() * motionSigmaThreshold_ ||
			fabs(accelZeroMean.z()) > accelMinStddev_.z() * motionSigmaThreshold_) {
		return true;
	}

	return false;
}
