// /*
//  * merge_videos_node.cpp
//  *
//  *  Created on: April 18, 2023
//  *      Author: yakir huri
//  *
//  *
//  * Cogniteam LTD CONFIDENTIAL
//  *
//  * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
//  *
//  * NOTICE:  All information contained  herein  is,  and  remains the property
//  * of Cogniteam.   The   intellectual   and   technical   concepts  contained
//  * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
//  * Foreign Patents, patents in process,  and  are  protected  by trade secret
//  * or copyright law. Dissemination of  this  information  or  reproduction of
//  * this material is strictly forbidden unless  prior  written  permission  is
//  * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
//  * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
//  * managers   or   contractors   who   have   executed   Confidentiality  and
//  * Non-disclosure    agreements    explicitly    covering     such     access
//  *
//  * The copyright notice  above  does  not  evidence  any  actual  or intended
//  * publication  or  disclosure    of    this  source  code,   which  includes
//  * information that is confidential  and/or  proprietary,  and  is  a   trade
//  * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
//  * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
//  * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
//  * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
//  * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
//  * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
//  * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
//  * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
//  *
//  */

#include <ros/ros.h>

#include <angles/angles.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <map>
#include <iostream>
#include <random>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

using namespace cv;
using namespace std::chrono;
using namespace std;

#include <dynamic_reconfigure/server.h>
#include <merge_videos/VideoConfig.h>

using namespace sensor_msgs;
using namespace cv;

Mat frontImg_;
Mat backImg_;
Mat leftImg_;
Mat rightImg_;

cv::Rect roi_front_;
cv::Rect roi_left_;
cv::Rect roi_right_;
cv::Rect roi_back_;

int GLOBAL_W = 1920;
int GLOBAL_H = 720;

int FRONT_ROI_X_ = 49;
int FRONT_ROI_Y_ = 0;
int FRONT_ROI_W_ = 997;
int FRONT_ROI_H_ = 720;
int FRONT_STARTING_X_ = 474;
int FRONT_STARTING_Y_ = 0;

double LEFT_SCALE_FACTOR = 1.0;
int LEFT_STARTING_X_ = -788;
int LEFT_STARTING_Y_  = -172;

double RIGHT_SCALE_FACTOR = 1.0;
int RIGHT_STARTING_X_ = 819;
int RIGHT_STARTING_Y_ = 0;

double BACK_SCALE_FACTOR = 0.35;
int    BACK_STARTING_X_ = 400;
int    BACK_STARTING_Y_ = 20;

string front_camera_topic_;
string  back_camera_topic_;
string  right_camera_topic_;
string  left_camera_topic_;
string  merged_image_topic_;


image_transport::Publisher merged_img_pub;

string video_layout = "FRONT_BACK";


void imageCallbackFront(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    frontImg_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallbackBack(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    backImg_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    leftImg_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    rightImg_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void configCallback(
    merge_videos::VideoConfig &config, uint32_t mask)
{

  FRONT_ROI_X_ = config.front_roi_x;
  FRONT_ROI_Y_ = config.front_roi_y;
  FRONT_ROI_W_ = config.front_roi_w;
  FRONT_ROI_H_ = config.front_roi_h;
  FRONT_STARTING_X_ = config.front_starting_x;
  FRONT_STARTING_Y_ = config.front_starting_y;

  LEFT_SCALE_FACTOR = config.left_scale_factor;
  LEFT_STARTING_X_ = config.left_starting_x;
  LEFT_STARTING_Y_  = config.left_starting_y;

  RIGHT_SCALE_FACTOR = config.right_scale_factor;
  RIGHT_STARTING_X_ = config.right_starting_x;
  RIGHT_STARTING_Y_  = config.right_starting_y;

  BACK_SCALE_FACTOR = config.back_scale_factor;
  BACK_STARTING_X_ = config.back_starting_x;
  BACK_STARTING_Y_  = config.back_starting_y;

}

bool setImgOnRef(const Mat &cropFront, Mat &mergedImg, int startPosition_x, int startPosition_y)
{

  Mat input = mergedImg.clone();

  cv::Mat output = input.clone();
  cv::Mat marker = cropFront.clone();

  // subimage dimensions:
  cv::Point startPosition = cv::Point(startPosition_x, startPosition_y);
  cv::Size size = marker.size();

  // ROI:
  cv::Rect subImageRect = cv::Rect(startPosition, size);

  // limit the roi if roi is bigger than the original image:
  cv::Rect fullImageRect = cv::Rect(cv::Point(0, 0), input.size());

  // intersection of both rois
  subImageRect = subImageRect & fullImageRect;
  if (subImageRect.width == 0 || subImageRect.height == 0)
  {
    std::cout << "marker position isn't within the image dimensions" << std::endl;
    return false;
  }

  // subimage = reference to image part of original image:
  cv::Mat outputSubImage = output(subImageRect);
  // marker subimage should be the whole marker, but might be reduced.
  cv::Mat markerSubImage = marker(cv::Rect(0, 0, subImageRect.width, subImageRect.height));

  // now just copy the data:
  markerSubImage.copyTo(outputSubImage);
  // if you don't want to use .copyTo, just use a loop over 0 .. subImage.width/height and copy from same pixel location to same pixel location.
  // cv::imshow("output", output);
  // cv::waitKey(0);

  mergedImg = output.clone();

  return true;
}

void stream4CamerasLayout(){

  try
  {
    if (!(frontImg_.data && backImg_.data && rightImg_.data && leftImg_.data))
    {

      ROS_ERROR("failed to merge images");

      return;
    }

    Mat mergedImg_ = cv::Mat(GLOBAL_H, GLOBAL_W, CV_8UC3, cv::Scalar(0));

    cv::Rect roi_front_ = cv::Rect(FRONT_ROI_X_, FRONT_ROI_Y_, FRONT_ROI_W_, FRONT_ROI_H_);

    cv::Mat cropFront = frontImg_(roi_front_);

    cv::Mat left_resize;
    cv::resize(leftImg_, left_resize, cv::Size(leftImg_.cols *  LEFT_SCALE_FACTOR, leftImg_.rows * LEFT_SCALE_FACTOR));

    cv::Mat right_resize;
    cv::resize(rightImg_, right_resize, cv::Size(rightImg_.cols * RIGHT_SCALE_FACTOR, rightImg_.rows * RIGHT_SCALE_FACTOR));

    setImgOnRef(left_resize, mergedImg_, LEFT_STARTING_X_, LEFT_STARTING_Y_);

    setImgOnRef(right_resize, mergedImg_, RIGHT_STARTING_X_, RIGHT_STARTING_Y_);

    setImgOnRef(cropFront, mergedImg_, FRONT_STARTING_X_, FRONT_STARTING_Y_);

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mergedImg_).toImageMsg();
    merged_img_pub.publish(msg);
    
    cv::waitKey(1);

    // imshow("mergedImg_", mergedImg_);

    // cv::waitKey(1);
  }
  catch (cv::Exception &e)
  {
    const char *err_msg = e.what();
    // std::cout << "exception caught: " << err_msg << std::endl;
  }
}

void streamFrontBackLayout(){

  try
  {
    if (!(frontImg_.data && backImg_.data ))
    {

      ROS_ERROR("failed to merge images");

      return;
    }

    Mat mergedImg_ = frontImg_.clone();   

    cv::Mat back_resize;
    cv::resize(backImg_, back_resize, cv::Size(backImg_.cols * BACK_SCALE_FACTOR, backImg_.rows * BACK_SCALE_FACTOR)); 

    setImgOnRef(back_resize, mergedImg_, BACK_STARTING_X_, BACK_STARTING_Y_);

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mergedImg_).toImageMsg();
    merged_img_pub.publish(msg);
    
    // imshow("mergedImg_", mergedImg_);
    cv::waitKey(1);

   
    
  }
  catch (cv::Exception &e)
  {
    const char *err_msg = e.what();
    std::cout << "exception caught: " << err_msg << std::endl;
  }
}
void timer_callback(const ros::TimerEvent &event)
{

  if ( video_layout == "four_cameras"){

     stream4CamerasLayout();
  } 
  else if ( video_layout == "front_back"){

     streamFrontBackLayout();

  } else {

    ROS_ERROR("invalid layout");
  }

  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "merge_videos_node");

  ros::NodeHandle nh("~");

  //params
  nh.param("front_camera_topic", front_camera_topic_, string("/usb_cam_front/image_front"));
  nh.param("back_camera_topic", back_camera_topic_, string("/usb_cam_back/image_back"));
  nh.param("right_camera_topic", right_camera_topic_, string("/usb_cam_right/image_right"));
  nh.param("left_camera_topic", left_camera_topic_, string("/usb_cam_left/image_left"));

  cerr<<" front_camera_topic_ "<<front_camera_topic_<<endl;

  nh.param("merged_image_topic", merged_image_topic_, string("/merged_image"));

  nh.param("video_layout", video_layout, string("front_back"));


  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_front = it.subscribe(front_camera_topic_, 1, imageCallbackFront);
  image_transport::Subscriber sub_left = it.subscribe(left_camera_topic_, 1, imageCallbackLeft);
  image_transport::Subscriber sub_right = it.subscribe(right_camera_topic_, 1, imageCallbackRight);
  image_transport::Subscriber sub_back = it.subscribe(back_camera_topic_, 1, imageCallbackBack);

  merged_img_pub = it.advertise(merged_image_topic_, 1);
  nh.param(merged_image_topic_+"/compressed/jpeg_quality", 60);

  dynamic_reconfigure::Server<merge_videos::VideoConfig> configServer_;

  configServer_.setCallback(
      boost::bind(configCallback, _1, _2));

  ros::Timer video_timer = nh.createTimer(ros::Duration(1.0 / 5.0), timer_callback);

  ros::spin();

  return 0;
}
