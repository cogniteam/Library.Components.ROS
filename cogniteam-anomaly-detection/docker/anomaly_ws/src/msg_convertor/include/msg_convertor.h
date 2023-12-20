#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Duration.h>
#include <time.h>
#include <mutex>
#include <sstream>

#include <string>
#include <iostream>

#include <map>
#include <vector>

#include <iterator>
#include <algorithm>

#ifndef MSG_CONVERTOR
#define MSG_CONVERTOR

#define SIZE_OF_VECTOR 22//#384

using namespace std;

/**
 * Node: MsgConvertor
 * @param csv_path (path_) - The path in the user's computer, where the node will write the csv file.
 * @param detection_mode (detectionMode_) - When detection_mode is false, the node will write to the given file (csv_path) a given time from the user.
 *                                          When detection_mode is true, the node will mot write data to the .csv file, but will publish csv (every time step a line) in the /csv topic.
 * @param time (time_) - The time the user wants to record the data from the subscribed topics, convert them to csv and write them in the appropriate file (csv_path)
 */

class MsgConvertor
{
public:
    MsgConvertor();
    ~MsgConvertor(){}

private: 

    //void streamsCallback(const std_msgs::Float64::ConstPtr& msg, std::string topicName);
    void streamsCallback(std::string datatype);
    void updateTimerCallback(const ros::TimerEvent&);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    bool timePassed( int Second = 1);
    
private:

    ros::NodeHandle node_;
    ros::Timer rosTimer_;

    ros::Subscriber subOdom_;
    ros::Subscriber subImu_;
    ros::Subscriber subScan_;
    ros::Subscriber subPose_;
    ros::Subscriber subTopic_;

    ros::Publisher pub_;
    ros::Publisher pubDetectionMode_;


    std::map<std::string, double > mapStreams_;
    std::vector<double> outputStream_;

    std::mutex mtx_;

    int count_ = 1;

    int numOfStreams_;
    double algoRate_;
    std::string mode_;
    std::stringstream ss;

    string path_;
    bool writeCsv_ = false;

    std::map<std::string, std::string> topics_;
    vector<ros::Subscriber> streamSubscribers_;

    ros::Time time_begin_;
    int time_;
    bool detectionMode_ = false;
    std_msgs::String state_;
};

#endif 