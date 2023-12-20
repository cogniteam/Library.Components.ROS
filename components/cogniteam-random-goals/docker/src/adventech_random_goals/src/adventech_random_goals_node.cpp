// /*
//  * adventech_random_goals_node.cpp
//  *
//  *  Created on: April 26, 2023
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
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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

#include <numeric>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <ctime>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace cv;
using namespace std::chrono;
using namespace std;

#include "../include/MoveBaseController.h"

#define UNKNOWN -1
#define FREE 0
#define BLOCKED 100

#define LARGE_NUM 10000

#define VISITED 255
#define NOT_VISITED 0

const int free_space = 254;

bool exit_ = false;

void addDilationForGlobalMap(Mat& imgMap, float walls_inflation_m, float mapResolution)
{
  try
  {
    int dilationPix = 3;  // (1.0 / mapResolution) * (walls_inflation_m);

    cv::Mat binary = imgMap.clone();
    binary.setTo(0, imgMap != 0);
    binary.setTo(255, imgMap == 0);
    dilate(binary, binary, Mat(), Point(-1, -1), dilationPix, 1, 1);

    imgMap.setTo(0, binary == 255);
  }
  catch (cv::Exception& e)
  {
    const char* err_msg = e.what();
    std::cerr << "exception caught: " << err_msg << std::endl;
  }
}

void addFreeSpaceDilation(Mat& grayscaleImg)
{
  Mat binary = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, CV_8UC1, cv::Scalar(0));

  binary.setTo(255, grayscaleImg >= 254);
  dilate(binary, binary, Mat(), Point(-1, -1), 1, 1, 1);

  Mat newImg = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, CV_8UC1, cv::Scalar(205));

  newImg.setTo(254, binary >= 254);
  newImg.setTo(0, grayscaleImg == 0);

  grayscaleImg = newImg;
}

class AdventechRandomGoals
{
public:
  AdventechRandomGoals(bool debug = false)
  {
    cerr << " wait for move-base server " << endl;
    moveBaseController_.waitForServer(ros::Duration(100.0));
    ros::Duration(1).sleep();
    cerr << " map-coverage is now connecting with move-base !! " << endl;

    // subs
    global_map_sub_ =
        node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &AdventechRandomGoals::globalMapCallback, this);

    global_cost_map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1,
                                                                     &AdventechRandomGoals::globalCostMapCallback, this);
    
    start_sto_navigation_sub_ = node_.subscribe<std_msgs::Bool>("/start_stop_navigation", 1,
                                                                     &AdventechRandomGoals::startStopCallback, this);
     odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom", 1, 
      &AdventechRandomGoals::odomCallback, this);
    
    //pub
    state_pub_ = node_.advertise<std_msgs::String>("/robot_state", 10);

    stateTimer_ = node_.createTimer(ros::Rate(1), 
      &AdventechRandomGoals::state_timer_callback, this);

    vel_cmd_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);


    initSlamMap_ = false;

    /// params
    mapResolution_ = -1;
    map_origin_position_x = -1;
    map_origin_position_y = -1;
  }

  ~AdventechRandomGoals()
  {
    moveBaseController_.cancelNavigation();

    stateTimer_.stop();

    ros::Duration(1).sleep();

    ros::shutdown();
  }

  Mat occupancyGridMatToGrayScale(const Mat& map)
  {
    // CV_8UC1 - UINT image type
    Mat output(map.rows, map.cols, CV_8UC1, Scalar(205));

    for (int j = 0; j < map.rows; j++)
    {
      for (int i = 0; i < map.cols; i++)
      {
        auto value = map.at<int8_t>(cv::Point(i, j));
        uint8_t newValue = 0;

        if (value == UNKNOWN)  // unknown
          newValue = 205;
        else if (value == FREE)  // clear
          newValue = 254;
        else if (value == BLOCKED)  // occupay
          newValue = 0;

        output.at<uchar>(cv::Point(i, j)) = newValue;
      }
    }

    return output;
  }

  void startStopCallback(const std_msgs::Bool::ConstPtr& msg)
  {
      if ( msg->data == true){

        moveBaseController_.moveBaseClient_.cancelAllGoals();  
        
        state_ = "RUNNING";
        enableNav_ = true;
      
      } else if ( msg->data == false){

        moveBaseController_.moveBaseClient_.cancelAllGoals();  

        state_ = "IDLE";
        enableNav_ = false;
      } 
  }

  void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    globalMapOriginPositionX_ = msg->info.origin.position.x;
    globalMapOriginPositionY_ = msg->info.origin.position.y;
    globalMapResolution_ = msg->info.resolution;

    auto endLocalCostMap = high_resolution_clock::now();
    auto durationFromLastCalc = duration_cast<seconds>(endLocalCostMap - startLocalCostMap_).count();

    /// do this every 2 seconds
    if (initSlamMap_ && durationFromLastCalc > 0.5)
    {
      costMapImg_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1, Scalar(0));
      memcpy(costMapImg_.data, msg->data.data(), msg->info.height * msg->info.width);

      for (int j = 0; j < costMapImg_.rows; j++)
      {
        for (int i = 0; i < costMapImg_.cols; i++)
        {
          int value = costMapImg_.at<uchar>(j, i);

          if (value == 100)
          {
            costMapImg_.at<uchar>(j, i) = 255;  ///
          }
          else if (value > 0 && value < 100)
          {
            costMapImg_.at<uchar>(j, i) = 100;  // inflation
          }
        }
      }
      initGlobalCostMap_ = true;

      string global_costmap_frame = msg->header.frame_id;

      // imshow("costMapImg_",costMapImg_);
      // waitKey(1);


      // Mat dbg = costMapImg.clone();
      // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

      startLocalCostMap_ = endLocalCostMap;
    }
  }

  geometry_msgs::PointStamped transformFrames(Point3d objectPoint3d, string target_frame, string source_Frame,
                                              ros::Time t)
  {
    geometry_msgs::PointStamped pointStampedIn;
    geometry_msgs::PointStamped pointStampedOut;

    pointStampedIn.header.frame_id = source_Frame;
    pointStampedIn.header.stamp = t;
    pointStampedIn.point.x = objectPoint3d.x;
    pointStampedIn.point.y = objectPoint3d.y;
    pointStampedIn.point.z = objectPoint3d.z;

    try
    {
      tf::StampedTransform transform;

      tfListener_.waitForTransform(target_frame, source_Frame, ros::Time(0), ros::Duration(0.01));

      tfListener_.lookupTransform(target_frame, source_Frame, ros::Time(0), transform);

      tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

      return pointStampedOut;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());

      return pointStampedOut;
    }
  }

  void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    // Check if the global map size changed
    if (width_ != -1 && height_ != -1)
    {
      if (msg->info.width != width_ || msg->info.height != height_)
      {
        double old_origin_x = map_origin_position_x;
        double old_origin_y = map_origin_position_y;

        double new_origin_x = msg->info.origin.position.x;
        double new_origin_y = msg->info.origin.position.y;

        double deltaW = msg->info.width - width_;
        double deltaH = msg->info.height - height_;
      }
    }

    cv::Mat tmp = cv::Mat(msg->info.height, msg->info.width, CV_8SC1, Scalar(UNKNOWN));
    memcpy(tmp.data, msg->data.data(), msg->info.height * msg->info.width);

    globalMapWidth_ = msg->info.width;

    globalMapHeight_ = msg->info.height;

    mapResolution_ = msg->info.resolution;

    map_origin_position_x = msg->info.origin.position.x;

    map_origin_position_y = msg->info.origin.position.y;

    initSlamMap_ = true;

    currentGlobalMap_ = occupancyGridMatToGrayScale(tmp.clone());

    mappingMap_ = currentGlobalMap_.clone();

    addDilationForGlobalMap(currentGlobalMap_, 0.05, mapResolution_);

    addFreeSpaceDilation(currentGlobalMap_);
  }

  cv::Mat getCurrentMap()
  {
    Mat tmp = currentGlobalMap_.clone();
    return tmp;
  }

  double distanceCalculate(cv::Point2d p1, cv::Point2d p2)
  {
    double x = p1.x - p2.x;  // calculating number to square in next step
    double y = p1.y - p2.y;
    double dist;

    dist = pow(x, 2) + pow(y, 2);  // calculating Euclidean distance
    dist = sqrt(dist);

    return dist;
  }

  bool run()
  { 

    std::vector<geometry_msgs::PoseStamped> validGoals;

    while (ros::ok())
    {
      ros::spinOnce();

      if (!initSlamMap_ || !initGlobalCostMap_)
      {
        // << "map not recieved !!" << endl;

        continue;
      }

      if ( !enableNav_) {       
        continue;
      }

      if (!initRotation_){
        
        rotateInPlace();
        
        initRotation_ = true;
      }

      if (!updateRobotLocation())
      {
        cerr << "can't update robot location !!" << endl;
        continue;
      }

      currentAlgoMap_ = getCurrentMap();

      // // create the exploration map (gmapping + global cost map)
      Mat explorationImgMap = currentAlgoMap_.clone();
      addDilationByGlobalCostMap(costMapImg_, explorationImgMap, convertPoseToPix(robotPose_));
      addFreeSpaceDilation(explorationImgMap);

      // imshow("explorationImgMap",explorationImgMap);
      // waitKey(1);

      Mat binary = explorationImgMap.clone();
      binary.setTo(255, binary >= 254);
      binary.setTo(0, binary != 255);

      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours(binary, contours, hierarchy, RETR_EXTERNAL,
                    CHAIN_APPROX_NONE, Point(0, 0));



      auto robotPix = convertPoseToPix(robotPose_);

      bool foundCont = false;
      int indexCont = -1;
      for( int i =0; i < contours.size(); i++ ){

          if ( pointPolygonTest(contours[i], robotPix, false) >= 0){
              
              indexCont = i;
              foundCont = true;
          } else {
              
            drawContours(binary, contours, i, Scalar(0), -1 );

          }
      }

      if( foundCont) {
        
        cerr<<" found robot location on the map!! "<<endl;
        Rect r = cv::boundingRect(contours[indexCont]);

        int minRangeX = r.x;
        int maxRangeX = r.x + r.width;
        int miRangeY = r.y;
        int maxRangeY = r.y + r.height;

        std::random_device rdX; // obtain a random number from hardware
        std::mt19937 genX(rdX()); // seed the generator
        std::uniform_int_distribution<> distrX(minRangeX, maxRangeX); // define the range
        int xPix = distrX(genX);

        std::random_device rdY; // obtain a random number from hardware
        std::mt19937 genY(rdY()); // seed the generator
        std::uniform_int_distribution<> distrY(miRangeY, maxRangeY); // define the range
        int yPix = distrY(genY);

        cv::Point goalPix(xPix,yPix);
        cerr<<"goalPix "<<goalPix<<endl;
        
        if (explorationImgMap.at<uchar>(goalPix.y, goalPix.x) != 254 ){
          continue;
        }

       

        // circle(binary, robotPix, 2, Scalar(150), -1, 8, 0); 

        // circle(binary, goalPix, 2, Scalar(100), -1, 8, 0); 

        geometry_msgs::Quaternion q;
        q = robotPose_.pose.orientation;
        auto nextGoal = convertPixToPose(goalPix, q);

        float distGoalFromRobot = 
          distanceCalculate(cv::Point2d(nextGoal.pose.position.x,nextGoal.pose.position.y ), 
            cv::Point2d(robotPose_.pose.position.x,robotPose_.pose.position.y));
        
        if( distGoalFromRobot < 0.3){
          continue;
        }

        /// calculate the  path
        nav_msgs::Path wanted_path;
        bool resMakePlan = makePlan(wanted_path, robotPose_, nextGoal);
        if (resMakePlan){
          
          // imshow("binary",binary);
          // waitKey(1);
          bool result = sendGoal(nextGoal);  

          if( result){
            validGoals.push_back(nextGoal);
          }
        } else {
          
          cerr<<" failed to make a plan !! "<<endl;
          
          if( validGoals.size() > 0) {
            sendGoal(validGoals[0]);
          }

          continue;
        }       

      } else {

        cerr<<" failed to locate robot !! "<<endl;
        if( validGoals.size() > 0){
          sendGoal(validGoals[0]);
        }
        
      }

     
    }

    return false;
  }

  bool rotateInPlace(int numOfRounds = 1)
  {
    geometry_msgs::Twist rotationCmd;
    rotationCmd.linear.x = 0;
    rotationCmd.linear.y = 0;
    rotationCmd.linear.z = 0;
    rotationCmd.angular.x = 0;
    rotationCmd.angular.y = 0;
    // clockwise
    rotationCmd.angular.z = -abs(0.4);

    float sumOfAngles = 0.0;

    int iteration = 0;

    float prevAngle = 0.0;

    cerr << " start to rotate " << numOfRounds << " rounds " << endl;

   

    while (ros::ok())
    {
      updateRobotLocation();     

      float currDeg = angles::to_degrees(
          atan2((2.0 * (currOdom_.pose.pose.orientation.w * currOdom_.pose.pose.orientation.z +
                        currOdom_.pose.pose.orientation.x * currOdom_.pose.pose.orientation.y)),
                (1.0 - 2.0 * (currOdom_.pose.pose.orientation.y * currOdom_.pose.pose.orientation.y +
                              currOdom_.pose.pose.orientation.z * currOdom_.pose.pose.orientation.z))));

      // init
      if (iteration == 0)
      {
        prevAngle = currDeg;
        iteration++;

        continue;
      }

      if (!(prevAngle < 0 && currDeg > 0))
      {
        sumOfAngles = sumOfAngles + (prevAngle - currDeg);

        prevAngle = currDeg;
      }
      else
      {
        sumOfAngles = sumOfAngles + (180 + (180 + prevAngle)) - currDeg;

        prevAngle = currDeg;
      }

      // cerr<<"sumOfAngles "<<sumOfAngles<<endl;

      int num_of_rounds = (int(sumOfAngles) / 360);

      if (num_of_rounds >= numOfRounds)
      {
        geometry_msgs::Twist rotationCmd;
        rotationCmd.linear.x = 0;
        rotationCmd.linear.y = 0;
        rotationCmd.linear.z = 0;
        rotationCmd.angular.x = 0;
        rotationCmd.angular.y = 0;
        rotationCmd.angular.z = 0;

        vel_cmd_pub_.publish(rotationCmd);
        ros::Duration(0.1).sleep();

        return true;
      }      

      iteration++;

      vel_cmd_pub_.publish(rotationCmd);

      ros::Duration(0.1).sleep();

      ros::spinOnce();
    }

    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    currOdom_.child_frame_id = msg->child_frame_id;
    currOdom_.header = msg->header;
    currOdom_.pose = msg->pose;
    currOdom_.twist = msg->twist;
  }

  cv::Point convertPoseToPix(const geometry_msgs::PoseStamped& pose)
  {
    float xPix = (pose.pose.position.x - map_origin_position_x) / mapResolution_;
    float yPix = (pose.pose.position.y - map_origin_position_y) / mapResolution_;

    cv::Point p = cv::Point(xPix, yPix);

    return p;
  }

  bool updateRobotLocation()
  {
    tf::StampedTransform transform;

    try
    {
      // get current robot pose
      tfListener_.waitForTransform(globalFrame_, baseFrame_, ros::Time(0), ros::Duration(0.01));

      tfListener_.lookupTransform(globalFrame_, baseFrame_, ros::Time(0), transform);

      robotPose_.header.frame_id = globalFrame_;
      robotPose_.header.stamp = ros::Time::now();
      robotPose_.pose.position.x = transform.getOrigin().x();
      robotPose_.pose.position.y = transform.getOrigin().y();
      robotPose_.pose.position.z = 0;
      robotPose_.pose.orientation.x = transform.getRotation().x();
      robotPose_.pose.orientation.y = transform.getRotation().y();
      robotPose_.pose.orientation.z = transform.getRotation().z();
      robotPose_.pose.orientation.w = transform.getRotation().w();

      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());

      cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;

      return false;
    }
  }

  geometry_msgs::PoseStamped convertPixToPose(const cv::Point& pixel, geometry_msgs::Quaternion q)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = globalFrame_;

    pose.pose.position.x = (pixel.x * mapResolution_) + map_origin_position_x;
    pose.pose.position.y = (pixel.y * mapResolution_) + map_origin_position_y;
    pose.pose.position.z = 0.0;

    if (q.w != 0.0)
    {
      pose.pose.orientation.w = q.w;
      pose.pose.orientation.x = q.x;
      pose.pose.orientation.y = q.y;
      pose.pose.orientation.z = q.z;
    }
    else
    {
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
    }

    return pose;
  }

  bool makePlan(nav_msgs::Path& path, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;

    if (client.call(srv))
    {
      cerr << " plan size " << srv.response.plan.poses.size() << endl;

      path = srv.response.plan;

      if (srv.response.plan.poses.size() == 0)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    else
    {
      return false;
    }

    return true;
  }

  void addDilationByGlobalCostMap(const Mat& globalCostMap, Mat& algoMap, const cv::Point2d& robotBaseFootPrint)
  {
    if (!globalCostMap.data || !algoMap.data || robotBaseFootPrint.x < 0 || robotBaseFootPrint.y < 0 ||
        robotBaseFootPrint.x > algoMap.cols || robotBaseFootPrint.y > algoMap.rows)
    {
      cerr << " failed to addDilationByGlobalCostMap " << endl;
      return;
    }

    algoMap.setTo(0, globalCostMap > 0);

  }

  bool sendGoal(const geometry_msgs::PoseStamped& goalMsg)
  { 
    ros::spinOnce();

    if ( !enableNav_) {        

      return false;
    }

    // navigate to the point
    moveBaseController_.navigate(goalMsg);

    bool result = true;

    while (ros::ok())
    {
      ros::spinOnce();

       if ( !enableNav_) {        

        return false;
      }

      moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
      auto move_base_state = moveBaseController_.moveBaseClient_.getState();

      if (move_base_state == actionlib::SimpleClientGoalState::ACTIVE ||
          move_base_state == actionlib::SimpleClientGoalState::PENDING)
      {
        continue;
      }

      if (move_base_state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        result = true;
        break;
      }
      else
      {
        result = false;
        break;
      }
    }

    return result;
  }

  void state_timer_callback(const ros::TimerEvent &event)
  {
    std_msgs::String msg;
    msg.data = state_;
    state_pub_.publish(msg);
  }

private:
  
  ros::NodeHandle node_;

  string state_ = "IDLE";


  // move-base
  MoveBaseController moveBaseController_;

  //timer
  ros::Timer stateTimer_;


  // subs
  ros::Subscriber global_map_sub_;

  ros::Subscriber global_cost_map_sub_;

  ros::Subscriber start_sto_navigation_sub_;

  ros::Subscriber odom_sub_;

  // pubs

  ros::Publisher state_pub_;

  ros::Publisher vel_cmd_pub_;

  // ALGO-PARAMS

  bool enableNav_ = false;

  bool initRotation_ = false;

  cv::Mat currentAlgoMap_;

  geometry_msgs::PoseStamped robotPose_;

  geometry_msgs::PoseStamped robotStartLocation_;

  vector<cv::Point2d> currentCameraScanMapPointsM_;

  cv::Mat currentGlobalMap_;

    nav_msgs::Odometry currOdom_;


  Mat mappingMap_;

  float map_origin_position_x = -1;

  float map_origin_position_y = -1;

  float globalMapWidth_;

  float globalMapHeight_;

  float mapResolution_ = 0.05;

  // global cost map params
  float globalMapOriginPositionX_ = -1;
  float globalMapOriginPositionY_ = -1;
  float globalMapResolution_ = -1;
  cv::Mat costMapImg_;

  bool initSlamMap_ = false;

  bool initGlobalCostMap_ = false;

  float width_;

  float height_;

  tf::TransformListener tfListener_;

  string globalFrame_ = "map";

  string baseFrame_ = "base_link";

  string odomFrame_ = "odom";

  high_resolution_clock::time_point startLocalCostMap_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adventech_random_goals_exploration_node", ros::init_options::NoSigintHandler);

  AdventechRandomGoals AdventechRandomGoals;

  AdventechRandomGoals.run();

  ros::spin();

  return 0;
}
