/*
 * hamster_driver_node.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: blackpc
 */

#include <iostream>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <tf/tf.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <hamster_driver/calibration/ImuData.h>
#include <hamster_driver/Odometry2d.h>
#include <hamster_driver/calibration/AutoImuCalibrator.h>

#include "WirelessInfo.h"

using namespace std;


/*************************************************************************************************
 *** Hasmter serial driver
 **************************************************************************************************/


template <typename T>
struct MagnetometerCalibration
{
	T magOffsetX;
	T magOffsetY;
	T magOffsetZ;
};


class HamsterSerialDriver {

public:

	HamsterSerialDriver(const string& port, uint32_t baudRate, int steerOffset,
			const string& odomFrame, const string& baseFrame, bool publishOdomTf,
			double headingOffset, const MagnetometerCalibration<double>& magnetometer,
			const AngularVelocities gyroBias, double maxSpeed, bool debug, bool smoothSteering,
			bool realsense, std::string agentNumber)
		: serial_(port, baudRate), targetSpeed_(0),
		  targetSteer_(0), lastSpeed_(0), lastSteer_(0), steerOffset_(steerOffset),
		  odomFrame_(odomFrame), baseFrame_(baseFrame), publishOdomTf_(publishOdomTf),
		  headingOffset_(headingOffset), maxSpeed_(fabs(maxSpeed)), debug_(debug),
		  firstRead_(true), magnetometer_(magnetometer), gyroBias_(gyroBias),
			smoothSteering_(smoothSteering), realsense_(realsense), agentNumber_(agentNumber) {

		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(timeout);

		ros::NodeHandle node;

		ackermannCmdSub_ = node.subscribe("ackermann_cmd", 10,
				&HamsterSerialDriver::ackermannCallback, this);

		batteryPublisher_ = node.advertise<std_msgs::Float32>(
				"battery", 1, false);

		pidPublisher_ = node.advertise<geometry_msgs::Vector3>(
				"pid", 10, false);

        // 
        // Actually bias-corrected reading
        // 
		imuPublisher_ = node.advertise<sensor_msgs::Imu>("imu/raw", 10, false);

        // 
        // Real raw data without biases
        // 
		imuBiasedPublisher_ = node.advertise<sensor_msgs::Imu>("imu/biased", 10, false);

		magneticFieldPublisher_ = node.advertise<sensor_msgs::MagneticField>("mag", 10, false);

		odomPublisher_ = node.advertise<nav_msgs::Odometry>("odom", 10, false);

		velocityThread_ = boost::thread(&HamsterSerialDriver::velocityPublishThread, this);

        // imuBiasUpdateTimer_ = node.createTimer(ros::Duration(1.0), 
        //         &HamsterSerialDriver::imuBiasUpdateTimerCallback, this);

		//
		// Realsense IMU data
		//
		realsenseGyroSubscriber_ = node.subscribe("camera/gyro/sample", 10,  
				&HamsterSerialDriver::imuRealsenseGyroCallback, this);

		realsenseAccelSubscriber_ = node.subscribe("camera/accel/sample", 10,
				&HamsterSerialDriver::imuRealsenseAccelCallback, this);

		autoImuCalibratorSubscriber_ = node.subscribe("events/auto_imu_calibrator/calibrated", 10,
				&HamsterSerialDriver::imuAutoCalibratorSubscriber, this);
		
	}

public:

	void spin() {
		string line;

		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(timeout);

		ros::Time startTime = ros::Time::now();
		bool dataReceived = false;

		while (ros::ok()) {
			try {
				line = serial_.readline();
			} catch (exception& ex) {
				reopenSerialPort();
			}

			if (!dataReceived && (ros::Time::now() - startTime).toSec() > 3) {
				//reopenSerialPort();
				startTime = ros::Time::now();
			}

			boost::replace_all(line, "\r\n", "");
			
			try {
				dataReceived = parseData(line);
			} catch (exception& ex) {
				cerr<<"error dataReceived "<<endl;
			}

			if(realsense_)
				imuFrameTransformListener();
		}
	}

private:

	serial::Serial serial_;

	ros::Subscriber ackermannCmdSub_;
	ros::Subscriber realsenseGyroSubscriber_;
	ros::Subscriber realsenseAccelSubscriber_;
	ros::Subscriber autoImuCalibratorSubscriber_;

	ros::Publisher odometryPublisher_;
	ros::Publisher batteryPublisher_;
	ros::Publisher pidPublisher_;
	ros::Publisher imuPublisher_;
	ros::Publisher imuBiasedPublisher_;
	ros::Publisher magneticFieldPublisher_;
	ros::Publisher odomPublisher_;

	tf::TransformBroadcaster azimuthTfBroadcaster_;
	tf::TransformBroadcaster odometryTfBroadcaster_;

	double targetSpeed_;
	double targetSteer_;

	double lastSpeed_;
	double lastSteer_;

	int steerOffset_;

	string odomFrame_;
	string baseFrame_;

	boost::thread velocityThread_;

	boost::mutex setSpeedMutex_;

	bool publishOdomTf_;

	double headingOffset_;

	double maxSpeed_;

	bool debug_;

	bool firstRead_;

	bool smoothSteering_;

	bool realsense_;

	std::string agentNumber_;

	MagnetometerCalibration<double> magnetometer_;

	AngularVelocities gyroBias_;

	ros::Time lastVelocityCommandTime_;

	Odometry2d odometry_;

    ros::Timer imuBiasUpdateTimer_;

	sensor_msgs::Imu imuRealsenseMessageRaw_;

	tf::TransformListener imuTransformListener_;

private:

	static const string VOLTAGE_PREFIX;
	static const string ODOMETRY_PREFIX;
	static const string IMU_PREFIX;
	static const string PID_PREFIX;

private:

	void imuFrameTransformListener() {
		tf::StampedTransform transformAccel;
		tf::StampedTransform transformGyro;
		try {

			imuTransformListener_.waitForTransform(agentNumber_ + "/base_link",
									agentNumber_ + "/camera_accel_optical_frame", ros::Time(0), ros::Duration(0.0));
			imuTransformListener_.lookupTransform(agentNumber_ + "/base_link",
									agentNumber_ + "/camera_accel_optical_frame", ros::Time(0), transformAccel);
			imuTransformListener_.waitForTransform(agentNumber_ + "/base_link",
									agentNumber_ + "/camera_gyro_optical_frame", ros::Time(0), ros::Duration(0.0));
			imuTransformListener_.lookupTransform(agentNumber_ + "/base_link",
									agentNumber_ + "/camera_gyro_optical_frame", ros::Time(0), transformGyro);
		} catch (tf::TransformException exception) {
			ROS_ERROR_THROTTLE(5, "%s", exception.what());
		}
			
		tf::Vector3 gyroTransformed = transformGyro * (tf::Vector3(
						imuRealsenseMessageRaw_.angular_velocity.x,
						imuRealsenseMessageRaw_.angular_velocity.y,
						imuRealsenseMessageRaw_.angular_velocity.z));

		tf::Vector3 accelTransformed = transformAccel * (tf::Vector3(
						imuRealsenseMessageRaw_.linear_acceleration.x,
						imuRealsenseMessageRaw_.linear_acceleration.y,
						imuRealsenseMessageRaw_.linear_acceleration.z));
		
		sensor_msgs::Imu imuRawMsg;

		imuRawMsg.angular_velocity.x = gyroTransformed.getX();
		imuRawMsg.angular_velocity.y = gyroTransformed.getY();
		imuRawMsg.angular_velocity.z = gyroTransformed.getZ();

		imuRawMsg.linear_acceleration.x = accelTransformed.getX();
		imuRawMsg.linear_acceleration.y = accelTransformed.getY();
		imuRawMsg.linear_acceleration.z = accelTransformed.getZ();

		imuRawMsg.header.frame_id = baseFrame_;
		imuRawMsg.header.stamp = ros::Time::now();

		imuBiasedPublisher_.publish(imuRawMsg);

		sensor_msgs::Imu imuMsg;

		imuMsg.angular_velocity.x = gyroTransformed.getX() - gyroBias_.x;
		imuMsg.angular_velocity.y = gyroTransformed.getY() - gyroBias_.y;
		imuMsg.angular_velocity.z = gyroTransformed.getZ() - gyroBias_.z;

		imuMsg.linear_acceleration.x = accelTransformed.getX();
		imuMsg.linear_acceleration.y = accelTransformed.getY();
		imuMsg.linear_acceleration.z = accelTransformed.getZ();

		imuMsg.header.frame_id = baseFrame_;
		imuMsg.header.stamp = ros::Time::now();

		imuPublisher_.publish(imuMsg);

		double angularVelocityZ = imuMsg.angular_velocity.z;

		updateOdometry(angularVelocityZ);
	}

	void imuRealsenseGyroCallback(const sensor_msgs::ImuConstPtr& gyroMessage) {
		imuRealsenseMessageRaw_.angular_velocity.x = gyroMessage->angular_velocity.x;
		imuRealsenseMessageRaw_.angular_velocity.y = gyroMessage->angular_velocity.y;
		imuRealsenseMessageRaw_.angular_velocity.z = gyroMessage->angular_velocity.z;
	}

	void imuRealsenseAccelCallback(const sensor_msgs::ImuConstPtr& accelMessage) {
		imuRealsenseMessageRaw_.linear_acceleration.x = accelMessage->linear_acceleration.x;
		imuRealsenseMessageRaw_.linear_acceleration.y = accelMessage->linear_acceleration.y;
		imuRealsenseMessageRaw_.linear_acceleration.z = accelMessage->linear_acceleration.z;
	}
	
    // void imuBiasUpdateTimerCallback(const ros::TimerEvent&) {
    //     ros::NodeHandle node;

    //     node.getParamCached("gyro_bias_x", gyroBias_.x);
    //     node.getParamCached("gyro_bias_y", gyroBias_.y);
    //     node.getParamCached("gyro_bias_z", gyroBias_.z);
    // }

	void imuAutoCalibratorSubscriber(const geometry_msgs::Vector3ConstPtr& msg) {
		ros::NodeHandle node;
		gyroBias_.x = msg->x;
        gyroBias_.y = msg->y;
        gyroBias_.z = msg->z;

	}

	void limit(double& v, double min, double max) {
		if (v < min)
			v = min;

		if (v > max)
			v = max;
	}

	float toRadians(float degrees) const {
		return degrees * M_PI / 180.0;
	}

	int toDegree(float radians) const {
		return radians * 180.0 / M_PI;
	}

	void reopenSerialPort() {
		ROS_WARN("Reopening serial port...");

		serial_.close();

		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(timeout);

		serial_.open();
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		ROS_INFO("Port reopened!");
	}

	void velocityPublishThread() {
		ackermann_msgs::AckermannDriveStamped message;
		double linearZeroCounter;

		while (ros::ok()) {

			double targetSpeed;
			double targetSteer;

			{
				boost::mutex::scoped_lock lock(setSpeedMutex_);
				targetSpeed = targetSpeed_;
				targetSteer = targetSteer_;
			}

			double linear = targetSpeed;
			double angular = toDegree(targetSteer);

			double linearZeroCounterIncremet;

			if (smoothSteering_)
				linearZeroCounterIncremet = 0.1;
			else
				linearZeroCounterIncremet = 0.2;

			if (fabs(linear) < 0.03 && fabs(angular) > 0.001) {
				linearZeroCounter += linearZeroCounterIncremet;
				linear = 1.0 * sin(linearZeroCounter);
				linear *= 0.2;

				if (linear < 0)
					angular *= -1;
			}else
				linearZeroCounter = 0;

			if(smoothSteering_) {
				angular = angular - lastSteer_;

				if (angular > 4.0)
					angular = 4.0;

				if (angular < -4.0)
					angular = -4.0;

				angular = lastSteer_ + angular;
			}

			limit(angular, -45, 45);

			setVelocity(linear, angular);

			lastSpeed_ = linear;
			lastSteer_ = angular;

			// Watchdog
			if ((ros::Time::now() - lastVelocityCommandTime_).toSec() > 0.5) {
				if (targetSpeed_ != 0 || targetSteer_ != 0) {
					boost::mutex::scoped_lock lock(setSpeedMutex_);
					targetSpeed_ = 0;
					targetSteer_ = 0;

					setVelocity(0, 0);
				}
			}

			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		}
	}

	void setVelocity(float speed, int steerAngleDegree) {

		// Limit speed
		if (speed > maxSpeed_)
			speed = maxSpeed_;

		if (speed < -maxSpeed_)
			speed = -maxSpeed_;

		char stringCommand[256];
		sprintf(stringCommand, "%05.2f:%03i;\n", speed, (steerAngleDegree + steerOffset_));
		serial_.write(stringCommand);
	}

	void ackermannCallback(const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {
		boost::mutex::scoped_lock lock(setSpeedMutex_);

		 double speed = cmd->drive.speed;
		 double angle = cmd->drive.steering_angle;
		 bool isRotationOnly = fabs(speed) < 0.03 && fabs(angle) > 0.001;

		 limit(angle, angles::from_degrees(-45), angles::from_degrees(45));

		 if (isRotationOnly && angle > 0){
			 angle = angles::from_degrees(35);
		 }else if (isRotationOnly && angle < 0){
		     angle = angles::from_degrees(-35);
		 }

		targetSpeed_ = speed;
		targetSteer_ = angle;

		lastVelocityCommandTime_ = ros::Time::now();
	}

	/**
	 * Parses data received from hamster firmware
	 * @param command
	 */
	bool parseData(const string command) {

		// Indicates if legal command received
		bool commandParsed = true;

		if (boost::starts_with(command, VOLTAGE_PREFIX)) {
			parseVoltage(command);
		}
		else if (!realsense_ && boost::starts_with(command, IMU_PREFIX)) {
			parseImu(command);
		}
		else if (boost::starts_with(command, ODOMETRY_PREFIX)) {
			parseOdometry(command);
		}
		else if (boost::starts_with(command, PID_PREFIX)) {
			parsePid(command);
		}
		else {
			ROS_INFO("%s", command.c_str());
			commandParsed = false;
		}

		if (debug_) {
			ROS_INFO("%s", command.c_str());
		}

		return commandParsed;
	}

	void parseVoltage(string line) {
		boost::replace_all(line, VOLTAGE_PREFIX, "");
		try {
			float voltage = boost::lexical_cast<float>(line);

			std_msgs::Float32 msg;
			msg.data = voltage;
			batteryPublisher_.publish(msg);

			ROS_INFO_ONCE("Voltage publishing started");
		} catch (boost::bad_lexical_cast& e) {
			ROS_WARN("Failed to parse voltage line '%s'", line.c_str());
		}
	}

	void parseImu(string line) {
		boost::replace_all(line, IMU_PREFIX, "");

		vector<string> imuValues;
		boost::split(imuValues, line, boost::is_any_of(","));

		sensor_msgs::Imu msg;
		sensor_msgs::Imu rawImuMsg;
		sensor_msgs::MagneticField magneticFieldMsg;

		try {
			
			double roll = 0.0;
			double pitch = 0.0;
			double yaw = 0.0;

			double gyroX = boost::lexical_cast<double>(imuValues[3]);
			double gyroY = boost::lexical_cast<double>(imuValues[4]);
			double gyroZ = boost::lexical_cast<double>(imuValues[5]);

			roll = toRadians(boost::lexical_cast<double>(imuValues[0]));
			pitch = toRadians(boost::lexical_cast<double>(imuValues[1]));
			yaw = toRadians(boost::lexical_cast<double>(imuValues[2]));

			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = baseFrame_;

            rawImuMsg.header = msg.header;

			msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

			// Yaw variance

			msg.angular_velocity.x = toRadians(gyroX / 1000.0) - gyroBias_.x;
			msg.angular_velocity.y = toRadians(gyroY / 1000.0) - gyroBias_.y;
			msg.angular_velocity.z = toRadians(gyroZ / 1000.0) - gyroBias_.z;
            
			rawImuMsg.angular_velocity.x = toRadians(gyroX / 1000.0);
			rawImuMsg.angular_velocity.y = toRadians(gyroY / 1000.0);
			rawImuMsg.angular_velocity.z = toRadians(gyroZ / 1000.0);


			// if (targetSpeed_ == 0 && targetSteer_ == 0) {

			// 	double angularVelocityZ = msg.angular_velocity.z;

			// 	if (fabs(angularVelocityZ) < 0.075) {
			// 		angularVelocityZ = 0; // Prevents gyro drift when robot is still
			// 	}

			// 	odometry_.updateHeading(angularVelocityZ);
			// } else {
			// 	odometry_.updateHeading(msg.angular_velocity.z);
			// }
			double angularVelocityZ = msg.angular_velocity.z;

			updateOdometry(angularVelocityZ);

			msg.linear_acceleration.x = boost::lexical_cast<double>(imuValues[6]) / 1000.0;
			msg.linear_acceleration.y = boost::lexical_cast<double>(imuValues[7]) / 1000.0;
			msg.linear_acceleration.z = boost::lexical_cast<double>(imuValues[8]) / 1000.0;

			magneticFieldMsg.header.stamp = msg.header.stamp;
			magneticFieldMsg.header.frame_id = baseFrame_;

			magneticFieldMsg.magnetic_field.x = 
                    (boost::lexical_cast<double>(imuValues[9]) - magnetometer_.magOffsetX) / 1000;
			magneticFieldMsg.magnetic_field.y = 
                    (boost::lexical_cast<double>(imuValues[10]) - magnetometer_.magOffsetY) / 1000;
			magneticFieldMsg.magnetic_field.z = 
                    (boost::lexical_cast<double>(imuValues[11]) - magnetometer_.magOffsetZ) / 1000;

			magneticFieldPublisher_.publish(magneticFieldMsg);
            imuBiasedPublisher_.publish(rawImuMsg);
			imuPublisher_.publish(msg);

			ROS_INFO_ONCE("IMU publishing started");

		} catch (boost::bad_lexical_cast& e) {
			ROS_WARN("Failed to parse imu line '%s'", line.c_str());
		}
	}

	void updateOdometry(double& angularVelocityZ) {
		if (targetSpeed_ == 0 && targetSteer_ == 0) {

			if (fabs(angularVelocityZ) < 0.075) {
				angularVelocityZ = 0; // Prevents gyro drift when robot is still
			}
		}

		odometry_.updateHeading(angularVelocityZ);
	}

	void parseOdometry(string line) {
		boost::replace_all(line, ODOMETRY_PREFIX, "");

		vector<string> odomValues;
		boost::split(odomValues, line, boost::is_any_of(","));

		try {

			double speed = -boost::lexical_cast<float>(odomValues[0]);

			odometry_.update(speed / 1000.0);

			tf::StampedTransform odometryTf;
			odometryTf.setOrigin(tf::Vector3(odometry_.getX(), odometry_.getY(), 0));
			odometryTf.setRotation(tf::createQuaternionFromYaw(odometry_.getHeading()));

			odometryTf.stamp_ = ros::Time::now();
			odometryTf.frame_id_ = odomFrame_;
			odometryTf.child_frame_id_ = baseFrame_;

//			double linearXVelocity  = boost::lexical_cast<float>(odomValues[3]);
//			double linearYVelocity  = boost::lexical_cast<float>(odomValues[4]);
//			double angularVelocity = boost::lexical_cast<float>(odomValues[5]);

			if (publishOdomTf_)
				odometryTfBroadcaster_.sendTransform(odometryTf);

			nav_msgs::Odometry msg;

			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = odomFrame_;

			msg.child_frame_id = baseFrame_;

			msg.pose.pose.position.x = boost::lexical_cast<float>(odometry_.getX());
			msg.pose.pose.position.y = boost::lexical_cast<float>(odometry_.getY());

			msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_.getHeading());

			msg.twist.twist.linear.x = odometry_.getSpeed();
			msg.twist.twist.angular.z = odometry_.getAngularVelocity();

			odomPublisher_.publish(msg);

			ROS_INFO_ONCE("Odometry publishing started");
		} catch (boost::bad_lexical_cast& e) {
			ROS_WARN("Failed to parse odometry line '%s'", line.c_str());
		}
	}

	void parsePid(string line) {
		boost::replace_all(line, PID_PREFIX, "");

		vector<string> pidValues;
		boost::split(pidValues, line, boost::is_any_of(","));

		geometry_msgs::Vector3 msg;

		try {
			if (pidValues.size() < 2)
				throw boost::bad_lexical_cast();

			msg.x = boost::lexical_cast<float>(pidValues[0]);
			msg.y = boost::lexical_cast<float>(pidValues[1]);

			pidPublisher_.publish(msg);

			ROS_INFO_ONCE("PID publishing started");
		} catch (boost::bad_lexical_cast& e) {
			ROS_WARN("Failed to parse pid line '%s'", line.c_str());
		}
	}

};

const string HamsterSerialDriver::VOLTAGE_PREFIX = "#VOLTAGE:";
const string HamsterSerialDriver::ODOMETRY_PREFIX = "#ODOM:";
const string HamsterSerialDriver::IMU_PREFIX = "#IMU:";
const string HamsterSerialDriver::PID_PREFIX = "#PID:";


void rssiPublishThread() {
	ros::NodeHandle nodePrivate("~");
	string interface;

	nodePrivate.param("wlan", interface, string("wlan0"));
	WirelessInfo wirelessInfo(interface);
	ros::Publisher wirelessPublisher = nodePrivate.advertise<std_msgs::Float32>("rssi", 1, true);

	ros::Rate rate(2);

	while (ros::ok()) {
		std_msgs::Float32 message;
		message.data = wirelessInfo.getRssi();
		wirelessPublisher.publish(message);

		rate.sleep();
	}
}

/*************************************************************************************************
 *** Main
 **************************************************************************************************/

template<class T, size_t N>
void extractArrayFromParameter(ros::NodeHandle& node, const string paramName,
		boost::array<T, N>& array) {
	vector<T> vector;

	if (!node.hasParam(paramName)) {
		ROS_INFO("Parameter '%s' not found", paramName.c_str());
		return;
	}

	bool result = node.getParam(paramName, vector);

	if (!result) {
		ROS_WARN("Failed to get parameter '%s'", paramName.c_str());
		return;
	}

	memcpy(array.data(), vector.data(), sizeof(T) * N);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "hamster_driver_node");
	ros::NodeHandle nodePrivate("~");

	boost::thread rssiThread;

	string port;
	string odomFrame;
	string baseFrame;
	int baudRate;
	bool enableRssi;
	int steerOffset;
    bool publishOdomTf;
    bool debug;
    double headingOffset;
    double maxSpeed;

	bool smoothSteering;

	bool realsense;

	std::string agentNumber;

    MagnetometerCalibration<double> magnetometer;
    AngularVelocities gyroBias;
    AutoImuCalibrator imuCalibrator;

	nodePrivate.param("port", port, string("/dev/ttyAMA0"));
	nodePrivate.param("baud_rate", baudRate, 57600);
	nodePrivate.param("steer_offset", steerOffset, 0);
	nodePrivate.param("enable_rssi", enableRssi, true);

	nodePrivate.param("odom_frame", odomFrame, string("odom"));
	nodePrivate.param("base_frame", baseFrame, string("base_link"));

	nodePrivate.param("heading_offset", headingOffset, 0.0);

	nodePrivate.param("publish_odom_tf", publishOdomTf, true);
	nodePrivate.param("debug", debug, false);

	nodePrivate.param("mag_offset_x", magnetometer.magOffsetX, 0.0);
	nodePrivate.param("mag_offset_y", magnetometer.magOffsetY, 0.0);
	nodePrivate.param("mag_offset_z", magnetometer.magOffsetZ, 0.0);

	// nodePrivate.param("gyro_offset_x", gyroBias.x, 0.0);
	// nodePrivate.param("gyro_offset_y", gyroBias.y, 0.0);
	// nodePrivate.param("gyro_offset_z", gyroBias.z, 0.0);

	gyroBias.x = 0.0;
	gyroBias.y = 0.0;
	gyroBias.z = 0.0;

	nodePrivate.param("enable_realsense", realsense, true);

	nodePrivate.param("max_speed", maxSpeed, 1.2);
	nodePrivate.param("smooth_steering", smoothSteering, false);
	nodePrivate.param("agent_id", agentNumber, std::string(""));

	if (enableRssi)
		rssiThread = boost::thread(rssiPublishThread);

	HamsterSerialDriver serialDriver(
			port, baudRate, steerOffset, odomFrame,
			baseFrame, publishOdomTf, headingOffset,
			magnetometer, gyroBias, maxSpeed, debug,
			smoothSteering, realsense, agentNumber);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	serialDriver.spin();
}
