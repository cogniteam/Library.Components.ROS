/*
 * SoftIronCalibration.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */

#include <hamster_driver/calibration/SoftIronCalibration.h>

SoftIronCalibration::SoftIronCalibration() {
	calibrated_ = false;
}

SoftIronCalibration::~SoftIronCalibration() {
}

void SoftIronCalibration::addMagnetometerData(const MagnetometerData& mag) {
	xyPlane.push_back(MagnetometerData(mag.x, mag.y, 0.0));
	yzPlane.push_back(MagnetometerData(0.0, mag.y, mag.z));
	zxPlane.push_back(MagnetometerData(mag.x, 0.0, mag.z));
}

tf::Vector3 SoftIronCalibration::computeMajorSemiAxis(
		const vector<MagnetometerData> planePoints) const {

	vector<MagnetometerData> measurements = planePoints;

	std::sort(measurements.rbegin(), measurements.rend(),
			MagnetometerDataComparator());

	const int fivePercentPoints = 1; // planePoints.size() * 1.0 / 100.0;

	tf::Vector3 longestVector;
	tf::Vector3 positiveMajorAxis;
	tf::Vector3 negativeMajorAxis;

	longestVector = measurements[0];

	double safeThreshold = computeSafeThreshold(planePoints);

	for (int i = 0; i < fivePercentPoints; ++i) {
		tf::Vector3 vector = measurements[i];

		if (vector.distance(longestVector) < safeThreshold) {
			positiveMajorAxis += vector;
		} else {
			negativeMajorAxis += vector;
		}
	}

	positiveMajorAxis /= (double)fivePercentPoints;
	negativeMajorAxis /= (double)fivePercentPoints;

	return positiveMajorAxis;
}

double SoftIronCalibration::computeSafeThreshold(
		const vector<MagnetometerData> planePoints) const {
	double minX = numeric_limits<double>::max();
	double maxX = numeric_limits<double>::min();
	double minY = numeric_limits<double>::max();
	double maxY = numeric_limits<double>::min();
	double minZ = numeric_limits<double>::max();
	double maxZ = numeric_limits<double>::min();

	for (int i = 0; i < planePoints.size(); ++i) {
		MagnetometerData m = planePoints[i];

		if (m.x > maxX)
			maxX = m.x;

		if (m.x < minX)
			minX = m.x;

		if (m.y > maxY)
			maxY = m.y;

		if (m.y < minY)
			minY = m.y;

		if (m.z > maxZ)
			maxZ = m.z;

		if (m.z < minZ)
			minZ = m.z;
	}

	double radiusX = (fabs(minX) + fabs(maxX)) / 2.0;
	double radiusY = (fabs(minY) + fabs(maxY)) / 2.0;
	double radiusZ = (fabs(minZ) + fabs(maxZ)) / 2.0;

	const double epsilon = 0.00001;

	if (fabs(radiusX) <epsilon)
		return min(radiusY, radiusZ);

	if (fabs(radiusY) <epsilon)
		return min(radiusX, radiusZ);

	if (fabs(radiusZ) <epsilon)
		return min(radiusY, radiusX);


	return min(radiusX, min(radiusY, radiusZ));
}

void SoftIronCalibration::computeCalibrationParameters() {
	tf::Vector3 xyMajorAxis = computeMajorXYAxis();
	tf::Vector3 yzMajorAxis = computeMajorYZAxis();
	tf::Vector3 zxMajorAxis = computeMajorZXAxis();

	calibrationData_.yaw = tf::Vector3(1, 0, 0).angle(xyMajorAxis);


	calibrationData_.roll = tf::Vector3(0, 1, 0).angle(yzMajorAxis);
	calibrationData_.pitch = tf::Vector3(0, 0, 1).angle(zxMajorAxis);

	if (xyMajorAxis.y() < 0)
		calibrationData_.yaw *= -1;

	if (yzMajorAxis.y() > 0)
		calibrationData_.roll *= -1;

	if (zxMajorAxis.z() < 0)
		calibrationData_.pitch *= -1;

	calibrationData_.scaleX = 5000.0 / xyMajorAxis.length();
	calibrationData_.scaleY = 5000.0 / yzMajorAxis.length();
	calibrationData_.scaleZ = 5000.0 / zxMajorAxis.length();

	calibrationTransform_ .setOrigin(tf::Vector3(0, 0, 0));
	calibrationTransform_.setRotation(
			tf::createQuaternionFromRPY(
					-calibrationData_.roll, -calibrationData_.pitch, -calibrationData_.yaw));

	calibrated_ = true;
}

MagnetometerData SoftIronCalibration::correctMeasurement(
		const MagnetometerData& mag) const {
	tf::Vector3 rotatedMeasurement = calibrationTransform_ * (tf::Vector3)mag;

	rotatedMeasurement.setValue(
			rotatedMeasurement.x() * calibrationData_.scaleX,
			rotatedMeasurement.y() * calibrationData_.scaleY,
			rotatedMeasurement.z() * calibrationData_.scaleZ);

	return rotatedMeasurement;
}

