/*
 * hamster_imu_covariance_calibration_node.cpp
 *
 *  Created on: Jun 4, 2015
 *      Author: blackpc
 */

#include <iostream>
#include <algorithm>
#include <numeric>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <boost/foreach.hpp>
#include <boost/thread.hpp>


#define foreach BOOST_FOREACH


using namespace std;


class VarianceCalculator {

public:

	VarianceCalculator()
		: currentMean_(0), currentVariance_(0), measuring_(false) {

	}

	void startMeasurement() {
		measuring_ = true;
		measurements_.clear();
	}

	void stopMeasurements() {
		measuring_ = false;
		calculateVariance();
	}

	inline double getVariance() const {
		return currentVariance_;
	}

	inline double getMean() const {
		return currentMean_;
	}

	void addMeasurement(double value) {
		if (!measuring_)
			return;

		measurements_.push_back(value);
	}

private:

	vector<double> measurements_;

	double currentMean_;
	double currentVariance_;

	bool measuring_;

private:

	void calculateVariance() {

		double sum = accumulate(
				measurements_.begin(), measurements_.end(), 0.0);

		currentMean_ = sum / measurements_.size();

		sum = 0.0;

		for (int i = 0; i < measurements_.size(); ++i)
			sum += pow(measurements_[i] - currentMean_, 2);

		currentVariance_ = sum / measurements_.size();
	}

};


VarianceCalculator varianceCalculator_;


void imuCallback(const sensor_msgs::Imu::Ptr& imu) {
	varianceCalculator_.addMeasurement(tf::getYaw(imu->orientation));
}

void publishVelocity(ros::Publisher& publisher, double speed) {
	ackermann_msgs::AckermannDriveStamped msg;

	msg.drive.speed = speed;

	publisher.publish(msg);
}

boost::posix_time::ptime now() {
	return boost::posix_time::microsec_clock::local_time();
}

int main(int argc, char **argv) {

	if (argc < 5) {
		cout << "Usage: " << argv[0] << " start_speed stop_speed step measurement_duration_sec" << endl;
		return 1;
	}

	double minSpeed = atof(argv[1]);
	double maxSpeed = atof(argv[2]);
	double step = atof(argv[3]);
	double measurementDuration = atof(argv[4]);


	ros::init(argc, argv, "hamster_imu_covariance_calibration_node");
	ros::NodeHandle node;
	ros::NodeHandle nodePrivate("~");
	ros::Subscriber imuSubscriber = node.subscribe("imu", 1, imuCallback);

	ros::Publisher ackermannPublisher =
			node.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1, true);

	vector<string> summary;

	for (double speed = minSpeed; speed <= maxSpeed; speed += step) {

		ROS_INFO("Setting speed = %f", speed);
		publishVelocity(ackermannPublisher, speed);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		ROS_INFO("Measuring for %f seconds...", measurementDuration);
		varianceCalculator_.startMeasurement();

		boost::posix_time::ptime startTime = now();
		ros::Rate rate(100);

		while (ros::ok() && (now() - startTime).total_seconds() < measurementDuration) {
			ros::spinOnce();
			rate.sleep();
		}

		ROS_INFO("Done");

		varianceCalculator_.stopMeasurements();
		boost::this_thread::sleep(boost::posix_time::milliseconds(200));

		publishVelocity(ackermannPublisher, 0.0);

		ROS_INFO("Speed = %f, Variance = %f, Mean = %f",
				speed,
				varianceCalculator_.getVariance(),
				varianceCalculator_.getMean());

		string summaryLine = boost::lexical_cast<string>(speed)
				+ "," +
				boost::lexical_cast<string>(varianceCalculator_.getVariance());

		summary.push_back(summaryLine);
	}

	ROS_INFO("Summary");
	ROS_INFO("=====================================");

	for (int i = 0; i < summary.size(); ++i) {
		cout << summary[i] << endl;
	}

}

