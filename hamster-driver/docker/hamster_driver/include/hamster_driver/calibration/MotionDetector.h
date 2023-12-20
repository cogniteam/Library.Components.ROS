/*
 * MotionDetector.h
 *
 *  Created on: Dec 19, 2017
 *      Author: blackpc
 */

#ifndef SRC_IMU_MOTIONDETECTOR_H_
#define SRC_IMU_MOTIONDETECTOR_H_


#include <ros/ros.h>


/**
 * Source of motion detector
 */
enum MotionDetectorType {
	Accelerometers,//!< Accelerometers
	Gyroscopes,    //!< Gyroscopes
	Motors,        //!< Motors
	MotorsCommand, //!< MotorsCommand
};


/**
 * Represents single source of motion detection
 */
class MotionDetector {

public:

	MotionDetector(MotionDetectorType type = MotionDetectorType::Accelerometers) {
		motionDetectorType_ = type;
		motionDetected_ = true; // We don't know the current state
		motionDetectionTime_ = ros::Time::now();
		idleDetectionTime_ = ros::Time::now();
	}

public:

	static const char* toString(MotionDetectorType type) {
		switch (type) {
			case MotionDetectorType::Accelerometers:
				return "Accelerometers";
			case MotionDetectorType::Gyroscopes:
				return "Gyroscopes";
			case MotionDetectorType::Motors:
				return "Motors";
			case MotionDetectorType::MotorsCommand:
				return "MotorsCommand";
			default:
				throw new std::invalid_argument("Invalid motion detector type provided");
		}
	}

public:

	inline MotionDetectorType getMotionDetectorType() const {
		return motionDetectorType_;
	}

	inline ros::Time getMotionDetectionTime() const {
		return motionDetectionTime_;
	}

	inline bool isMotionDetected() const {
		return motionDetected_;
	}

	inline void setMotionDetected(bool motionDetected) {
		motionDetected_ = motionDetected;

		if (motionDetected) {
			motionDetectionTime_ = ros::Time::now();
		} else {
			idleDetectionTime_ = ros::Time::now();
		}
	}


private:

	/**
	 * Type of detector
	 */
	MotionDetectorType motionDetectorType_;

	/**
	 * Last motion detection time
	 */
	ros::Time motionDetectionTime_;

	/**
	 * Last idle state detection time
	 */
	ros::Time idleDetectionTime_;

	/**
	 * Current motion detection state
	 */
	bool motionDetected_;

};


#endif /* SRC_IMU_MOTIONDETECTOR_H_ */
