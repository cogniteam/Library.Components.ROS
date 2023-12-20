#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <mutex>
#include <sstream>
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <iterator>
#include <algorithm>

#include "ZScoreAnomalyDetector.h"
#include "SimpleAnomalyDetector.h"
#include "HybridAnomalyDetector.h"
#include "timeseries.h"
#include "AnomalyReport.h"

#ifndef MSG_CONVERTOR
#define MSG_CONVERTOR

#define SIZE_OF_VECTOR 17 //#384

using namespace std;

/**
 * Node: Algorithms
 * @param csv_path (pathTrain_) - The path in the user's computer of the train.csv file.
 * @param detection_mode (detectionMode_)  - when detectionMode is false, it means that the node is learning correlations of attributes according to the train.csv file and will write the results in the appropriate file (pathRes_).
 *                                            when detectionMode is true, it means that the node is in detection mode. The node will receive data from topic /anomalies (each time one line) and will decide according to the correlations he did in the learning mode, which pair of attributes has an anomaly on this specific line (which represents TimeStep)
 * @param algorithm_type (algorithmType_) -  algorithmType_ = 1 - Simple Anomaly Detector (default)
 *                                           algorithmType_ = 2 - ZScore Anomaly Detector
 *                                           algorithmType_ = 3 - Hybrid Anomaly Detector (Combines between simple linear regression, z score, and welzl algorithm)
 * @param result_file_path (pathRes_) - The path in the user's computer of the results file. THis file will store the results from the learning stage (correlated features, level of correlation and more)
 *
 *
 */

class Algorithms {
public:
    Algorithms();
    ~Algorithms(){}

private:

    void updateTimerCallback(const ros::TimerEvent& timerEvent);
    void subCallBack(const std_msgs::String::ConstPtr& msg);
    void detectionSubCallback(const std_msgs::String::ConstPtr& msg);
    void simpleAnomalyDetectionCallback();
    void zScoreAnomalyDetectionCallback();
    void hybridAnomalyDetectionCallback();

private:

    ros::NodeHandle node_;
    ros::Timer rosTimer_;
    ros::Subscriber subscriber_;
    ros::Subscriber detectionSub_;
    ros::Publisher pub_;

    SimpleAnomalyDetector* simpleAnomalyDetector_;
    ZScoreAnomalyDetector* zScoreDetector_;
    HybridAnomalyDetector* hybridAnomalyDetector_;

    TimeSeries* tsTrain_;
    TimeSeries* tsTest_;

    std::map<int, void (Algorithms::*)()> algorithmToCallback;

    vector<AnomalyReport> outputStream_;

    std::mutex mtx_;

    std::stringstream ss;

    string pathTrain_;
    string pathTest_;
    string pathRes_;
    bool detectionMode_ = false;
    int algorithmType_ = 1;
    float simpleAnomalyTh_ = 0.7;
    float upperBound_ = 0.9;
    float lowerBound_ = 0.5;
    std_msgs::String result_;
    int testCount_ = 1;

};

#endif