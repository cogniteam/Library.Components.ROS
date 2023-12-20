/*
 * calibration_tests.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: blackpc
 */

#include <gtest/gtest.h>

#define private public

#include <hamster_driver/calibration/HardIronCalibration.h>
#include <hamster_driver/calibration/SoftIronCalibration.h>

// Declare a test
TEST(HardIronCalibration, computeOffsets)
{
	HardIronCalibration calibration;

	MagnetometerData mag;

	for (int i = 1; i < 11; ++i) {
		mag.x = i;
		mag.y = i;
		mag.z = i;

		calibration.addMagnetometerData(mag);
	}

	for (int i = 0; i < 100; ++i) {
		mag.x = 500;
		mag.y = 500;
		mag.z = 500;

		calibration.addMagnetometerData(mag);
	}

	for (int i = 1; i < 11; ++i) {
		mag.x = 1000 + i;
		mag.y = 1000 + i;
		mag.z = 1000 + i;

		calibration.addMagnetometerData(mag);
	}

	mag = calibration.computeOffsets();

	const double exptectedMin = ( 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10) / 10;
	const double exptectedMax = 1000 + ( 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10) / 10;
	const double exptectedOffset = (exptectedMin + exptectedMax) / 2.0;

	ASSERT_EQ(exptectedOffset, mag.x);
	ASSERT_EQ(exptectedOffset, mag.y);
	ASSERT_EQ(exptectedOffset, mag.z);
}

TEST(SoftIronCalibration, computeSafeThreshold)
{
	SoftIronCalibration calibration;

	MagnetometerData mag;

	const double expectedThreshold = 100;

	for (int i = -expectedThreshold; i < expectedThreshold + 1; ++i) {
		mag.x = i;
		mag.y = i;
		mag.z = i;

		calibration.addMagnetometerData(mag);
	}

	double xyThreshold = calibration.computeSafeThreshold(calibration.xyPlane);
	double yzThreshold = calibration.computeSafeThreshold(calibration.yzPlane);
	double zxThreshold = calibration.computeSafeThreshold(calibration.zxPlane);

	ASSERT_EQ(expectedThreshold, xyThreshold);
	ASSERT_EQ(expectedThreshold, yzThreshold);
	ASSERT_EQ(expectedThreshold, zxThreshold);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
