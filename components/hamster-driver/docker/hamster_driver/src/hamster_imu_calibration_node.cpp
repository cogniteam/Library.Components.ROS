/*
 * imu_calibration_node.cpp
 *
 *  Created on: May 26, 2015
 *      Author: blackpc
 */


#include <iostream>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <hamster_driver/calibration/HardIronCalibration.h>


using namespace std;


const string FIXED_FRAME_ID = "map";
const double MEASUREMENT_SCALE_FACTOR = 0.001;


ros::Publisher rawPointPublisher_;
ros::Publisher hardIronCorrectedPointsPublisher_;
ros::Publisher rawCentroidPublisher_;
HardIronCalibration hardIronCalibration_;
int measurementsCounter_ = 1;
bool calibrationDone_ = false;

double gyroCalibrationDuration_; // seconds
int magCalibrationSamples_;


float toRadians(float degrees) {
	return degrees * M_PI / 180.0;
}

MagnetometerData correctHardIron(const MagnetometerData& mag,
		const MagnetometerData& magOffsets) {
	return MagnetometerData(
			mag.x - magOffsets.x,
			mag.y - magOffsets.y,
			mag.z - magOffsets.z);
}


ImuData parseImu(string line) {

	if (!boost::starts_with(line, "#IMU:"))
		return ImuData(MagnetometerData::null());

	boost::replace_all(line, "#IMU:", "");

	vector<string> imuValues;
	boost::split(imuValues, line, boost::is_any_of(","));

	double magX = boost::lexical_cast<double>(imuValues[9]);
	double magY = boost::lexical_cast<double>(imuValues[10]);
	double magZ = boost::lexical_cast<double>(imuValues[11]);

	double gyroX = toRadians(boost::lexical_cast<double>(imuValues[3]) / 1000.0);
	double gyroY = toRadians(boost::lexical_cast<double>(imuValues[4]) / 1000.0);
	double gyroZ = toRadians(boost::lexical_cast<double>(imuValues[5]) / 1000.0);

	return ImuData(
			MagnetometerData(magX, magY, magZ),
			AngularVelocities(gyroX, gyroY, gyroZ));
}

void publishMagnetometerData(const MagnetometerData& mag, ros::Publisher& publisher,
		double red = 1.0, double green = 0.0, double blue = 0.0) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = FIXED_FRAME_ID;
	marker.header.stamp = ros::Time::now();

	marker.id = rand();

	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(1000);
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.pose.position.x = mag.x * MEASUREMENT_SCALE_FACTOR;
	marker.pose.position.y = mag.y * MEASUREMENT_SCALE_FACTOR;
	marker.pose.position.z = mag.z * MEASUREMENT_SCALE_FACTOR;

	marker.pose.orientation.w = 1;

	publisher.publish(marker);
}

void publishRawMagnetometerData(const MagnetometerData& mag) {
	publishMagnetometerData(mag, rawPointPublisher_);
}

void publishHardIronCorrectedMagnetometerData(const MagnetometerData& mag) {
	publishMagnetometerData(mag, hardIronCorrectedPointsPublisher_, 0.0, 0.0, 1.0);
}

void publishRawCentroid(const MagnetometerData& centroid) {
	geometry_msgs::PointStamped point;
	point.header.stamp = ros::Time::now();
	point.header.frame_id = FIXED_FRAME_ID;
	point.point.x = centroid.x * MEASUREMENT_SCALE_FACTOR;
	point.point.y = centroid.y * MEASUREMENT_SCALE_FACTOR;
	point.point.z = centroid.z * MEASUREMENT_SCALE_FACTOR;
	rawCentroidPublisher_.publish(point);
}

void clearMarkers() {
	visualization_msgs::Marker marker;

	marker.header.frame_id = FIXED_FRAME_ID;
	marker.header.stamp = ros::Time::now();

	marker.action = 3u; // DELETEALL

	rawPointPublisher_.publish(marker);
	hardIronCorrectedPointsPublisher_.publish(marker);
}

void publishMajorAxis(tf::Vector3 axis, tf::Vector3 offset, double red = 1.0, double green = 0.0, double blue = 0.0) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = FIXED_FRAME_ID;
	marker.header.stamp = ros::Time::now();

	marker.id = rand();

	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(1000);
	marker.scale.x = 0.2;

	geometry_msgs::Point start;
	geometry_msgs::Point end;

	tf::Vector3 majorAxisOrigin = offset * MEASUREMENT_SCALE_FACTOR;
	tf::Vector3 majorAxis = (axis + offset) * MEASUREMENT_SCALE_FACTOR; // Translate to raw measurements frame

	start.x = majorAxisOrigin.x();
	start.y = majorAxisOrigin.y();
	start.z = majorAxisOrigin.z();

	end.x = majorAxis.x();
	end.y = majorAxis.y();
	end.z = majorAxis.z();

	marker.points.push_back(start);
	marker.points.push_back(end);

	marker.pose.orientation.w = 1;

	rawPointPublisher_.publish(marker);
}

void publishCorrectedMajorAxis(tf::Vector3 axis, tf::Vector3 offset) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = FIXED_FRAME_ID;
	marker.header.stamp = ros::Time::now();

	marker.id = measurementsCounter_;

	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(1000);
	marker.scale.x = 0.2;

	geometry_msgs::Point start;
	geometry_msgs::Point end;

	tf::Vector3 majorAxisOrigin = offset;
	tf::Vector3 majorAxis = (axis + offset); // Translate to raw measurements frame


//	axis = softIronCalibration_.correctMeasurement(axis);
	axis *= MEASUREMENT_SCALE_FACTOR;

//	start.x = majorAxisOrigin.x();
//	start.y = majorAxisOrigin.y();
//	start.z = majorAxisOrigin.z();
//
//	end.x = majorAxis.x();
//	end.y = majorAxis.y();
//	end.z = majorAxis.z();

	start.x = 0;
	start.y = 0;
	start.z = 0;

	end.x = axis.x();
	end.y = axis.y();
	end.z = axis.z();

	marker.points.push_back(start);
	marker.points.push_back(end);

	marker.pose.orientation.w = 1;

	rawPointPublisher_.publish(marker);
}

void processMagnetometerData(const MagnetometerData& rawMagnetometerData,
		bool publishRawMagnetometerPoints = true) {

	// Raw measurements
	if (publishRawMagnetometerPoints)
		publishRawMagnetometerData(rawMagnetometerData);

	// Hard & Soft iron correction
	const int requiredMeasurements = magCalibrationSamples_;

	if (measurementsCounter_ == requiredMeasurements) {
		MagnetometerData offsets = hardIronCalibration_.computeOffsets();
		MagnetometerData hardIronCorrectedMagnetometerData =
				correctHardIron(rawMagnetometerData, offsets);

		ROS_INFO_ONCE("%i points collected, publishing hard iron calibrated points", magCalibrationSamples_);
		ROS_INFO_ONCE("Offsets: x = %f, y = %f, z = %f", offsets.x, offsets.y, offsets.z);

		publishRawCentroid(offsets);

		// Soft iron calibration

		vector<MagnetometerData> rawMeasurements = hardIronCalibration_.getMeasurements();

		for (int i = 0; i < rawMeasurements.size(); ++i) {
			MagnetometerData correctedHardIronMeasurement = correctHardIron(rawMeasurements[i], offsets);

			publishHardIronCorrectedMagnetometerData(correctedHardIronMeasurement);

			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		calibrationDone_ = true;


	} else if (measurementsCounter_ < requiredMeasurements) {
		ROS_INFO_THROTTLE(1.0, "Need additional %i points for Hard Iron Calibration",
				requiredMeasurements - measurementsCounter_);
		hardIronCalibration_.addMagnetometerData(rawMagnetometerData);
	}


	// Misc
	measurementsCounter_++;
}

void magnetometerCallback(const visualization_msgs::Marker::Ptr& mag) {

	ROS_INFO_ONCE("Magnetometer message stream started");

	MagnetometerData magData;

	magData.x = mag->pose.position.x / MEASUREMENT_SCALE_FACTOR;
	magData.y = mag->pose.position.y / MEASUREMENT_SCALE_FACTOR;
	magData.z = mag->pose.position.z / MEASUREMENT_SCALE_FACTOR;

	processMagnetometerData(magData, false);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hamster_imu_calibration_node");
	ros::NodeHandle nodePrivate("~");

	string port;
	int baudRate;
	bool useTopic; // Use magnetometer data from topic rather then real device

	nodePrivate.param("port", port, string("/dev/ttyAMA0"));
	nodePrivate.param("baud_rate", baudRate, 57600);

	nodePrivate.param("gyro_calibration_duration", gyroCalibrationDuration_, 10.0);
	nodePrivate.param("mag_calibration_samples", magCalibrationSamples_, 500);

	nodePrivate.param("use_topic", useTopic, false);

	ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	ROS_WARN("DO NOT MOVE THE ROBOT WHILE GYRO CALIBRATION PROCESS IS ACTIVE");
	ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

	serial::Serial serial(port, baudRate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
	serial.setTimeout(timeout);

	rawPointPublisher_ = nodePrivate.advertise<visualization_msgs::Marker>(
			"/imu_calibration/raw_magnetometer", 10, false);

	hardIronCorrectedPointsPublisher_ = nodePrivate.advertise<visualization_msgs::Marker>(
			"/imu_calibration/hard_iron_corrected", 1000, false);

	rawCentroidPublisher_ = nodePrivate.advertise<geometry_msgs::PointStamped>(
			"/imu_calibration/raw_magnetometer_centroid", 10, true);

	clearMarkers();

	MagnetometerData lastMagnetometerData;

	bool gyroCalibration = true;
	bool gyroReceived = false;
	long gyroReadingsCount = 0;
	ros::Duration gyroCalibrationDuration(gyroCalibrationDuration_); // seconds
	ros::Time gyroCalibrationStart;
	ros::Time gyroCalibrationFinish;
	AngularVelocities gyroReadings;

	if (useTopic) {
		ros::Subscriber magnetometerSubscriber =
				nodePrivate.subscribe(rawPointPublisher_.getTopic(), 100, magnetometerCallback);

		ros::spin();
	} else {

		while (ros::ok() && !calibrationDone_) {
			string line = serial.readline();
			boost::trim(line);

			ImuData imuData = parseImu(line);

			if (gyroCalibration) {

				MagnetometerData rawMagnetometerData = imuData.mag;

				if (rawMagnetometerData.isNull() ||
						isnan(rawMagnetometerData.x) ||
						isnan(rawMagnetometerData.y) ||
						isnan(rawMagnetometerData.z))
					continue;

				// Gyro bias calibration
				if (!gyroReceived) {
					gyroReceived = true; // First gyro reading
					gyroCalibrationStart = ros::Time::now();
					gyroCalibrationFinish = gyroCalibrationStart + gyroCalibrationDuration;
					ROS_INFO("Gyro calibration started");
				}

				ROS_INFO_THROTTLE(1.0, "Gathering gyro readings, DO NOT MOVE THE ROBOT (%fs remaining)", (gyroCalibrationFinish - ros::Time::now()).toSec());

				if ((ros::Time::now() - gyroCalibrationStart) > gyroCalibrationDuration) {
					// Calibration done
					gyroCalibration = false;
					ROS_INFO("Gyro calibration done");
				}

				gyroReadingsCount++;

				gyroReadings += imuData.gyro;


			} else {

				// Magnetometer hard iron calibration

				MagnetometerData rawMagnetometerData = imuData.mag;

				if (rawMagnetometerData.isNull() ||
						isnan(rawMagnetometerData.x) ||
						isnan(rawMagnetometerData.y) ||
						isnan(rawMagnetometerData.z))
					continue;

				if (lastMagnetometerData == rawMagnetometerData)
					continue;
				else
					lastMagnetometerData = rawMagnetometerData;

				processMagnetometerData(rawMagnetometerData);

			}
		}

	}

	// Calibration proccess done
	MagnetometerData magOffsets = hardIronCalibration_.computeOffsets();
	gyroReadings /= gyroReadingsCount;

	cout << "export HAMSTER_MAG_OFFSET_X=" << magOffsets.x << endl;
	cout << "export HAMSTER_MAG_OFFSET_Y=" << magOffsets.y << endl;
	cout << "export HAMSTER_MAG_OFFSET_Z=" << magOffsets.z << endl;

	cout << "export HAMSTER_GYRO_OFFSET_X=" << gyroReadings.x << endl;
	cout << "export HAMSTER_GYRO_OFFSET_Y=" << gyroReadings.y << endl;
	cout << "export HAMSTER_GYRO_OFFSET_Z=" << gyroReadings.z << endl;

	return 0;
}

