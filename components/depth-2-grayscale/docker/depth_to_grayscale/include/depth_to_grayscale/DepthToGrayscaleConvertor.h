

#ifndef INCLUDE_DEPTH_TO_GRAYSCALE_CONVERTOR_H
#define INCLUDE_DEPTH_TO_GRAYSCALE_CONVERTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class DepthToGrayscaleConvertor {

public:
  DepthToGrayscaleConvertor();

  ~DepthToGrayscaleConvertor(){}

private:
  void depthCb(const sensor_msgs::ImageConstPtr &msg_depth);

private:

  ros::NodeHandle node_;

  image_transport::Publisher grayscalePub_;

  ros::Subscriber dpethSub_;

  int min_gray_scale_value_;
  int max_gray_scale_value_ ;
  int max_distance_ ;

};

#endif // INCLUDE_DEPTH_TO_GRAYSCALE_CONVERTOR_H
