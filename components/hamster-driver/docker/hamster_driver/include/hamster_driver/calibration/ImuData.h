/*
 * ImuData.h
 *
 *  Created on: Dec 11, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_HAMSTER_DRIVER_CALIBRATION_IMUDATA_H_
#define INCLUDE_HAMSTER_DRIVER_CALIBRATION_IMUDATA_H_


#include "MagnetometerData.h"


struct AngularVelocities {
	double x;
	double y;
	double z;

	AngularVelocities(double x_ = 0, double y_ = 0, double z_ = 0)
		: x(x_), y(y_), z(z_) {

	}

	void operator+=(const AngularVelocities& other) {
		x += other.x;
		y += other.y;
		z += other.z;
	}

	void operator/=(double divisor) {
		x /= divisor;
		y /= divisor;
		z /= divisor;
	}
};


class ImuData {

public:

	MagnetometerData mag;
	AngularVelocities gyro;

	ImuData(const AngularVelocities& velocities)
		: gyro(velocities) {

	}

	ImuData(const MagnetometerData& mag)
		: mag(mag) {

	}

	ImuData(const MagnetometerData& mag, const AngularVelocities& velocities)
		: mag(mag), gyro(velocities) {

	}

};


#endif /* INCLUDE_HAMSTER_DRIVER_CALIBRATION_IMUDATA_H_ */
