/*
 * HardIronCalibration.h
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_HAMSTER_DRIVER_HARDIRONCALIBRATION_H_
#define INCLUDE_HAMSTER_DRIVER_HARDIRONCALIBRATION_H_


#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#include <hamster_driver/calibration/ImuData.h>


using namespace std;


class HardIronCalibration {

public:

	HardIronCalibration();

	virtual ~HardIronCalibration();

public:

	void addMagnetometerData(const MagnetometerData& mag);

	MagnetometerData computeOffsets() const;

	vector<MagnetometerData> getMeasurements() const;

	MagnetometerLimits calculateLimits() const;

private:

	vector<double> xValues_;
	vector<double> yValues_;
	vector<double> zValues_;

	MagnetometerLimits limits_;

};

#endif /* INCLUDE_HAMSTER_DRIVER_HARDIRONCALIBRATION_H_ */
