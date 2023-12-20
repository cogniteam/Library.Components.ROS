/*
 * SoftIronCalibration.h
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_HAMSTER_DRIVER_CALIBRATION_SOFTIRONCALIBRATION_H_
#define INCLUDE_HAMSTER_DRIVER_CALIBRATION_SOFTIRONCALIBRATION_H_


#include <limits>

#include <tf/tf.h>

#include <hamster_driver/calibration/MagnetometerData.h>


/**
 * Compares the magnitude of magnetometers measurement vectors
 */
struct MagnetometerDataComparator
{
    inline bool operator() (const MagnetometerData& m1, const MagnetometerData& m2)
    {
        return tf::Vector3(m1.x, m1.y, m1.z).length()
        		< tf::Vector3(m2.x, m2.y, m2.z).length();
    }
};


struct SoftIronCalibrationData {
	double roll;
	double pitch;
	double yaw;
	double scaleX;
	double scaleY;
	double scaleZ;
};


class SoftIronCalibration {

public:

	SoftIronCalibration();

	virtual ~SoftIronCalibration();

public:

	/**
	 * Adds magnetometer measurement data, measurements origin must be (0, 0, 0)
	 * @param mag
	 */
	void addMagnetometerData(const MagnetometerData& mag);

	inline tf::Vector3 computeMajorXYAxis() const {
		return computeMajorSemiAxis(xyPlane);
	}

	inline tf::Vector3 computeMajorYZAxis() const {
		return computeMajorSemiAxis(yzPlane);
	}

	inline tf::Vector3 computeMajorZXAxis() const {
		return computeMajorSemiAxis(zxPlane);
	}

	void computeCalibrationParameters();

	inline SoftIronCalibrationData getCalibrationParameters() const {
		return calibrationData_;
	}

	MagnetometerData correctMeasurement(const MagnetometerData& mag) const;

private:

	/**
	 * Finds longest vector assumed to be the major semi-axis of ellipse
	 * @param planePoints
	 * @return
	 */
	tf::Vector3 computeMajorSemiAxis(const vector<MagnetometerData> planePoints) const;

	/**
	 * Computes safe threshold value about half average radius of ellipse
	 * @param planePoints
	 * @return
	 */
	double computeSafeThreshold(const vector<MagnetometerData> planePoints) const;

private:

	vector<MagnetometerData> xyPlane; // yaw
	vector<MagnetometerData> yzPlane; // roll
	vector<MagnetometerData> zxPlane; // pitch

	bool calibrated_;

	tf::Transform calibrationTransform_;

	SoftIronCalibrationData calibrationData_;
};

#endif /* INCLUDE_HAMSTER_DRIVER_CALIBRATION_SOFTIRONCALIBRATION_H_ */
