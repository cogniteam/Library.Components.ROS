/*
 * MagnetometerData.h
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_HAMSTER_DRIVER_CALIBRATION_MAGNETOMETERDATA_H_
#define INCLUDE_HAMSTER_DRIVER_CALIBRATION_MAGNETOMETERDATA_H_


#include <limits>
#include <cmath>

#include <tf/tf.h>


using namespace std;


class MagnetometerData {

public:

	MagnetometerData()
		: x(0), y(0), z(0) { }

	MagnetometerData(double x, double y, double z)
		: x(x), y(y), z(z) { }

	MagnetometerData(const tf::Vector3& v)
		: x(v.x()), y(v.y()), z(v.z()) { }

	static MagnetometerData null() {
		return MagnetometerData(numeric_limits<double>::quiet_NaN(),
				numeric_limits<double>::quiet_NaN(),
				numeric_limits<double>::quiet_NaN());
	}

	bool isNull() const {
		return isnan(x) && isnan(y) && isnan(z);
	}

	bool operator== (const MagnetometerData& mag) const {
		return x == mag.x && y == mag.y && z == mag.z;
	}

	operator tf::Vector3() const {
		return tf::Vector3(x, y, z);
	}

public:

	double x;
	double y;
	double z;
};


/**
 * Min-max values of magnetometer readings for each axis
 */
class MagnetometerLimits {

public:

	double minX;
	double maxX;
	double minY;
	double maxY;
	double minZ;
	double maxZ;


};


#endif /* INCLUDE_HAMSTER_DRIVER_CALIBRATION_MAGNETOMETERDATA_H_ */
