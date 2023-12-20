#include "../include/msg_convertor.h"
#include <fstream>
#include <iostream>
#include <filesystem>


MsgConvertor::MsgConvertor() {

  subOdom_ = node_.subscribe<nav_msgs::Odometry>("/odom", 1000, &MsgConvertor::odomCallback, this);
  subImu_ = node_.subscribe<sensor_msgs::Imu>("/imu", 1000, &MsgConvertor::imuCallback, this);
  subPose_ = node_.subscribe<geometry_msgs::Pose>("/pose", 1000, &MsgConvertor::poseCallback, this);
  subScan_ = node_.subscribe<sensor_msgs::LaserScan>("/laser_scan", 1000, &MsgConvertor::scanCallback, this);
  ros::NodeHandle nodePrivate("~");

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  /*for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      if(info.name != "/rosout_agg" && info.name!= "/rosout") {
          //cerr << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<std::endl;
          this->topics_.emplace(info.name, info.datatype);
      }
  }*/

  nodePrivate.param("csv_path", path_, string("/home/") );
  nodePrivate.param("time", time_, 10);
  nodePrivate.param("detection_mode", detectionMode_, false);

  if(detectionMode_ == false)
      pub_ = node_.advertise<std_msgs::String>("state", 20);
  if(detectionMode_ == true)
      pubDetectionMode_ = node_.advertise<std_msgs::String>("csv", 1);

  outputStream_.resize(SIZE_OF_VECTOR, 0);

  this->time_begin_ = ros::Time::now();

    try {
    rosTimer_ = node_.createTimer(ros::Rate(30),
    &MsgConvertor::updateTimerCallback, this);
  }
  catch(std::runtime_error& ex) {
    ROS_ERROR("My Exception: [%s]", ex.what());
  }
}

bool MsgConvertor::timePassed( int Second) {

    auto end = ros::Time::now();
    auto durationWithoutDetection = (end - time_begin_).toSec();

    if( durationWithoutDetection > Second){
        return true;
    }

    return false;
}

/*If the node is not in detection mode write */
void MsgConvertor::updateTimerCallback(const ros::TimerEvent& timerEvent) {
    //Todo: If the file is already existing, take in count that you dont need to write the titles again. Currently the methodology is that we don't append to the file while writing, (). Consider,  std::ofstream::trunc in line 82 instead of: std::ios_base:app
  if( !timePassed(time_) ) {
      std_msgs::String result;
      std_msgs::String resultForDetection;
      state_.data = "Not Finished";
      if(detectionMode_ == true) {
          resultForDetection.data += "TimeStep,odom_pose_x,odom_pose_y,odom_pose_z,odom_orientation_x,odom_orientation_y,odom_orientation_z,odom_orientation_w,";
          resultForDetection.data += "imu_orientation_x,imu_orientation_y,imu_orientation_z,imu_orientation_w,pose_x,pose_y,pose_z,lazerScan_angle_min,lazerScan_angle_max,";
          resultForDetection.data += "lazerScan_angle_increment,lazerScan_time_increment,lazerScan_scan_time,lazerScan_range_min,lazerScan_range_max,";
      }
      outputStream_[0] =  ros::Time::now().toSec(); // time stamp
      for(auto& element: outputStream_) {
          if(detectionMode_ == false)
              result.data += (to_string(element) + ",");
          else
              resultForDetection.data += (to_string(element) + ",");
      }
      if(detectionMode_ == false) {
          mtx_.lock();
          ofstream of;
          of.open(path_, std::ios_base::app);
          if (count_ == 1) {
              of << "TimeStep,";
              of << "odom_pose_x,";
              of << "odom_pose_y,";
              of << "odom_pose_z,";
              of << "odom_orientation_x,";
              of << "odom_orientation_y,";
              of << "odom_orientation_z,";
              of << "odom_orientation_w,";
              of << "imu_orientation_x,";
              of << "imu_orientation_y,";
              of << "imu_orientation_z,";
              of << "imu_orientation_w,";
              of << "pose_x,";
              of << "pose_y,";
              of << "pose_z,";
              of << "lazerScan_angle_min,";
              of << "lazerScan_angle_max,";
              of << "lazerScan_angle_increment,";
              of << "lazerScan_time_increment,";
              of << "lazerScan_scan_time,";
              of << "lazerScan_range_min,";
              of << "lazerScan_range_max"<<endl;
          }
          of << result.data << endl;
          pub_.publish(state_);
          of.close();
          count_++;
          mtx_.unlock();
      } else if (detectionMode_ == true) {
          pubDetectionMode_.publish(resultForDetection);
      }
  }
  else {
      // publish finish
      state_.data = "Finished";
      pub_.publish(state_);
      return;
  }
}

void MsgConvertor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  mtx_.lock();

  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

  outputStream_[1] = msg->pose.pose.position.x;
  outputStream_[2] = msg->pose.pose.position.y;
  outputStream_[3] = msg->pose.pose.position.z;

  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  outputStream_[4] = msg->pose.pose.orientation.x;
  outputStream_[5] = msg->pose.pose.orientation.y;
  outputStream_[6] = msg->pose.pose.orientation.z;
  outputStream_[7] = msg->pose.pose.orientation.w;
  
  mtx_.unlock();
}

void MsgConvertor::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  mtx_.lock();
  
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  
  outputStream_[8] = msg->orientation.x;
  outputStream_[9] = msg->orientation.y;
  outputStream_[10] = msg->orientation.z;
  outputStream_[11] = msg->orientation.w;
  
  mtx_.unlock();
}

void MsgConvertor::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  mtx_.lock();
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->position.x, msg->position.y, msg->position.z);
  outputStream_[12] = msg->position.x;
  outputStream_[13] = msg->position.y;
  outputStream_[14] = msg->position.z;

  mtx_.unlock();
}

void MsgConvertor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  mtx_.lock();
  
  ROS_INFO("Laser Scan -> angle_min: [%f], angle_max: [%f], angle_increment: [%f], time_increment: [%f], scan_time: [%f], range_min: [%f], range_max: [%f] ", msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);
  
  int numRanges = sizeof(msg->ranges);
  outputStream_[15] = msg->angle_min;
  outputStream_[16] = msg->angle_max;
  outputStream_[17] = msg->angle_increment;
  outputStream_[18] = msg->time_increment;
  outputStream_[19] = msg->scan_time;
  outputStream_[20] = msg->range_min;
  outputStream_[21] = msg->range_max;

 
  /*int index = 22;
  for(int i = 0; i < numRanges; i++)
  {
      outputStream_[index + i] = (msg->ranges[i]);
  }*/
  mtx_.unlock();
}

void MsgConvertor::streamsCallback(std::string datatype) {

}





