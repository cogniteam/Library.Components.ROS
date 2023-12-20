/*
 * HardIronCalibration.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */


#include <hamster_driver/calibration/HardIronCalibration.h>


HardIronCalibration::HardIronCalibration() {
}

HardIronCalibration::~HardIronCalibration() {
}

void HardIronCalibration::addMagnetometerData(const MagnetometerData& mag) {
	xValues_.push_back(mag.x);
	yValues_.push_back(mag.y);
	zValues_.push_back(mag.z);
}

MagnetometerData HardIronCalibration::computeOffsets() const {

	MagnetometerData offsets;
	MagnetometerLimits limits = calculateLimits();

	offsets.x = (limits.minX + limits.maxX) / 2.0;
	offsets.y = (limits.minY + limits.maxY) / 2.0;
	offsets.z = (limits.minZ + limits.maxZ) / 2.0;

	return offsets;

}

vector<MagnetometerData> HardIronCalibration::getMeasurements() const {
	vector<MagnetometerData> v;

	for (int i = 0; i < xValues_.size(); ++i)
		v.push_back(MagnetometerData(xValues_[i], yValues_[i], zValues_[i]));

	return v;
}

MagnetometerLimits HardIronCalibration::calculateLimits() const {
	vector<double> xValues = xValues_;
	vector<double> yValues = yValues_;
	vector<double> zValues = zValues_;

	const int averageCount = 10;

	MagnetometerLimits limits;

	if (xValues.size() < averageCount) {
		std::cerr << "At least " << averageCount << " measurements required for computing offsets" << std::endl;
		return limits;
	}

	std::sort(xValues.begin(), xValues.end());
	std::sort(yValues.begin(), yValues.end());
	std::sort(zValues.begin(), zValues.end());

	limits.minX = std::accumulate(xValues.begin(), xValues.begin() + averageCount, 0) / averageCount;
	limits.maxX = std::accumulate(xValues.end() - averageCount, xValues.end(), 0) / averageCount;

	limits.minY = std::accumulate(yValues.begin(), yValues.begin() + averageCount, 0) / averageCount;
	limits.maxY = std::accumulate(yValues.end() - averageCount, yValues.end(), 0) / averageCount;

	limits.minZ = std::accumulate(zValues.begin(), zValues.begin() + averageCount, 0) / averageCount;
	limits.maxZ = std::accumulate(zValues.end() - averageCount, zValues.end(), 0) / averageCount;

	return limits;
}
