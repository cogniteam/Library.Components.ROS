/*
 * person_follower_rgb_node.cpp
 *
 *  Created on: April 26, 2021
 *      Author: yakir huri
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <object_msgs/ObjectInBox.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <nlohmann/json.hpp> //https://github.com/nlohmann/json#cmake
using json = nlohmann::json;




enum GESTURE_FLAGE {
    NONE,
    TRACK,
    STAND
};
using namespace std;
using namespace cv;


struct Person
{

    std::map<std::string, Point2d> jointsWithLocations;
    cv::Rect boundingBox;
};


class PersonFollowerRgb
{

public:
    PersonFollowerRgb()
    {

        ros::NodeHandle node_;
        ros::NodeHandle nodePrivate("~");

       
        rgbSubscriber_ = node_.subscribe("/openvino_toolkit/images", 1,
                                         &PersonFollowerRgb::imageCallback, this);

        objectSubscriber_ = node_.subscribe("openvino_toolkit/detected_objects", 1,
                                            &PersonFollowerRgb::detectedObjectsCallback, this);

        gesturTypeSub_ = node_.subscribe("/gesture_type", 1,
                                            &PersonFollowerRgb::gestureTypeCallback, this);

        skeletonsSubscriber_ = node_.subscribe("/skeletons_text", 1,
            &PersonFollowerRgb::skeletonsCallback, this);
                                                                        

        image_transport::ImageTransport it(node_);
        debugImgPublisher_ = it.advertise("/debug_img", 1);
        nodePrivate.param("/debug_img/compressed/jpeg_quality", 20);

        updateTimer_ = node_.createTimer(ros::Rate(20), 
                &PersonFollowerRgb::updateTimerCallback, this);
                                    

        twistCommandPublisher_ 
            = node_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, false);      

        nodePrivate.param("target", trackingTargetClassName_,  string("person"));

        nodePrivate.param("focal_length", focalLength_, 747.0); // (pixel_w * real_cm_distance) / real_w_cm

        nodePrivate.param("known_target_width_cm", knownWidthCm_,  50.0); 

        nodePrivate.param("min_distance", minDistance_,  0.3); 

        nodePrivate.param("max_distance", maxDistance_,  1.5);

        nodePrivate.param("speed", speed_,  0.1); 


        nodePrivate.param("box_percentage_from_image_to_stop", boxPercentToStop_, 0.8);

        nodePrivate.param("angular_scale_factor",angularScaleFactor_, 1.0);

        nodePrivate.param("enable_gestures", enableGestures_, false);

        flag_ = STAND;

        trackingTargetPosition_ = cv::Point(0,0);

    } 

    ~PersonFollowerRgb() {}


    void gestureTypeCallback(const std_msgs::StringConstPtr &msg){

        if(msg->data == "NONE"){
            
            flag_ = NONE;
            return;
        }

        if(msg->data == "TWO_HNADS"){
            
            flag_ = STAND;
            return;
        }

        if(msg->data == "RIGHT_HAND_UP"){
            
            flag_ = TRACK;
            return;
        }

         flag_ = NONE;
    }    
    

    void skeletonsCallback(const std_msgs::String::Ptr &msg) {
       
        currentPersons_.clear();

        json j = json::parse(msg->data);

        std::vector<std::string> ec_a;
        for (auto &elem : j["skeletons"])
        {
            Person person;
            for (auto &elem2 : elem["joints"])
            {   
                cv::Point2d p = cv::Point2d(elem2["x"], elem2["y"]);

                if( p.x != 0 && p.y != 0){

                    person.jointsWithLocations[elem2["name"]] =
                        cv::Point2d(elem2["x"], elem2["y"]);
                }                
            }

            currentPersons_.push_back(person);
        }

    }

    void drawSkeletonsOnImg(cv::Mat& img) {

        for(int i = 0; i < currentPersons_.size(); i++) 
        {

            Person person = currentPersons_[i];
            if ( person.jointsWithLocations.find("Rwri") != person.jointsWithLocations.end() &&
                person.jointsWithLocations.find("Relb") != person.jointsWithLocations.end() &&
                person.jointsWithLocations.find("Rsho") != person.jointsWithLocations.end() &&
                person.jointsWithLocations.find("Lsho") != person.jointsWithLocations.end() &&  
                person.jointsWithLocations.find("Lelb") != person.jointsWithLocations.end() &&
                person.jointsWithLocations.find("Lwri") != person.jointsWithLocations.end())
            {
                

                // right hand
                cv::Point2d Rwri = person.jointsWithLocations.at("Rwri");
                cv::Point2d Relb = person.jointsWithLocations.at("Relb");
                cv::Point2d Rsho = person.jointsWithLocations.at("Rsho");
                
                if(Rwri.x > 0 && Rwri.y > 0 && Rwri.x < img.cols  && Rwri.y < img.rows 
                    && Relb.x > 0 && Relb.y > 0 && Relb.x < img.cols  && Relb.y < img.rows){
                        cv::line(img, Rwri, Relb, Scalar(255, 255, 0), 2);
                }

                if ( Rsho.x > 0 && Rsho.y > 0 && Rsho.x < img.cols && Rsho.y < img.rows  
                        && Relb.x > 0 && Relb.y > 0 && Relb.x < img.cols  && Relb.y < img.rows ){
                     cv::line(img, Rsho, Relb,  Scalar(255, 255, 0), 2);
                } 

                // left hand
                if ( person.jointsWithLocations.find("Lsho") != person.jointsWithLocations.end() && 
                    person.jointsWithLocations.find("Lelb") != person.jointsWithLocations.end() &&
                    person.jointsWithLocations.find("Lwri") != person.jointsWithLocations.end()) {

                    cv::Point2d Lsho = person.jointsWithLocations.at("Lsho");
                    cv::Point2d Lelb = person.jointsWithLocations.at("Lelb");
                    cv::Point2d Lwri = person.jointsWithLocations.at("Lwri");

                    if(Lsho.x > 0 && Lsho.y > 0 && Lsho.x < img.cols && Lsho.y < img.rows  
                        && Lelb.x > 0 && Lelb.y > 0 && Lelb.x < img.cols && Lelb.y < img.rows ){
                            cv::line(img, Lsho, Lelb, Scalar(0, 255, 255), 2);
                    }

                    if(Lelb.x > 0 && Lelb.y > 0 && Lelb.x < img.cols && Lelb.y < img.rows  
                        && Lwri.x > 0 && Lwri.y > 0 && Lwri.x < img.cols && Lwri.y < img.rows ){
                             cv::line(img, Lwri, Lelb, Scalar(0, 255, 255), 2);
                    }
                }  
            } 
                   
                    
        }



    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            currentImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            imgWidth = currentImg_.cols;
            showDebugImg(distance, degAngle, nearestObjectInBox);
           
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
    
    bool timePassed( int Second = 1) {

        auto end = ros::WallTime::now();
        auto durationWithoutDetection = (end - lastDetection_).toSec();

        if( durationWithoutDetection > Second){ 
            return true;
        }

        return false; 
    }
    void updateTimerCallback(const ros::TimerEvent&) {

        cerr<<" flag_: "<<flag_<<" state_: "<<state_<<endl;
        // gesture for stop tracing
        if ( flag_ == STAND && enableGestures_) {

            if( !initPublishZeroCmd_){
                publishZeroCmdVel();
                initPublishZeroCmd_ = true;
                trackingTargetPosition_ = cv::Point(0,0);

            }
            foundTarget_ = false;
            state_ = " IDLE";     
            return;
        }

        /// check if timePassed since last detection ( no targets)
        if( timePassed(1) ){
            cerr<<" durationWithoutDetection "<<endl;
            if( !initPublishZeroCmd_){
                publishZeroCmdVel();
                initPublishZeroCmd_ = true;
                trackingTargetPosition_ = cv::Point(0,0);

            }
            foundTarget_ = false;
            state_ = "IDLE";
            return;
        }

        /// no gestures and idle mode
        if ( (  (flag_ == STAND || flag_ == NONE) &&  enableGestures_)  && state_ == "IDLE" ) {

            if( !initPublishZeroCmd_){
                publishZeroCmdVel();
                initPublishZeroCmd_ = true;
                trackingTargetPosition_ = cv::Point(0,0);

            }
            foundTarget_ = false;
            state_ = "IDLE";     
            return;
            
        }

        // recognize gestrue for start tracking or still tracking
        if ( (flag_ == TRACK  && enableGestures_ ) 
            || ( (flag_ == NONE && enableGestures_ ) && state_ == " TRACKING")){

            state_ = "TRACKING"; 
        }

        if (foundTarget_ && state_ == "TRACKING")
        {
            distance = calculateDistanceFromObjectInMeter(nearestObjectInBox);
            degAngle = calculateAngleFromObject(nearestObjectInBox);           

            double widthPercent =  double(nearestObjectInBox.roi.width) / double(imgWidth);
            
            cerr<<" foundTarget_ widthPercent: "<<widthPercent<<endl;   

            // if the object too close by b_box size
            if( widthPercent > boxPercentToStop_ ){
                state_ = "TOO CLOSE";
               if( !initPublishZeroCmd_){
                    publishZeroCmdVel();
                    initPublishZeroCmd_ = true;
                }
                return;
            }
            if( distance >= minDistance_ && distance <= maxDistance_ ){

                state_ = "TRACKING";
                sendVelCmd(distance, degAngle); 
                initPublishZeroCmd_ = false;
            } 
            else
            {   
                state_ = "TOO FAR OR TOO CLOSE";
                if( !initPublishZeroCmd_){
                    publishZeroCmdVel();
                    initPublishZeroCmd_ = true;
                }
            }
        }
        else {
            state_ = "IDLE";
            if( !initPublishZeroCmd_){
                publishZeroCmdVel();
                initPublishZeroCmd_ = true;
            }
        }

    }
    

    void detectedObjectsCallback(const object_msgs::ObjectsInBoxes::Ptr &objects)
    {
     
        if (!currentImg_.empty())
        {   
            foundTarget_ = false;
            double maxWidth = 0;
            int index = -1;
            for (int i = 0; i < objects->objects_vector.size(); i++)
            {   
                
                if (objects->objects_vector[i].object.object_name == trackingTargetClassName_)
                {
                    double width = objects->objects_vector[i].roi.width;
                    double height = objects->objects_vector[i].roi.height;
                    //finding the nearest target
                    if (width > maxWidth)
                    {
                        maxWidth = width;
                        index = i;
                    }
                }
            }

            if (index != -1){

                nearestObjectInBox = objects->objects_vector[index];
                foundTarget_ = true;
                lastDetection_ = ros::WallTime::now();
            } else {
                foundTarget_ = false;
            }

        } else {

            foundTarget_ = false;
        }
            
    }

    void publishZeroCmdVel(){

        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;
        twistCommandPublisher_.publish(command);
    }

    void showDebugImg(double distance, double degAngle,
                      object_msgs::ObjectInBox nearestObjectInBox)
    {
       
        if (!currentImg_.empty())
        {   

            
            Mat debugImg = currentImg_.clone();
            //cv::Mat debugImg(currentImg_.rows, currentImg_.cols,CV_8UC3, cv::Scalar(0));
          
            cv::Point topLeft1(50,50);
           
            cv::Point topLeft2(50,100);

            cv::Point topLeft3(50,200);

            circle(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2),
                5, Scalar(255,0,255),FILLED, 8,0);

          
            circle(debugImg, 
                cv::Point(nearestObjectInBox.roi.x_offset + (nearestObjectInBox.roi.width / 2),
                currentImg_.rows / 2),
                //nearestObjectInBox.roi.y_offset + (nearestObjectInBox.roi.height / 2 ) ),
                 5, Scalar(0,255,255),FILLED, 8,0);
            
            if( state_ == "TRACKING"){

                cv::Rect r(nearestObjectInBox.roi.x_offset,nearestObjectInBox.roi.y_offset,
                    nearestObjectInBox.roi.width, nearestObjectInBox.roi.height);

                cv::rectangle(debugImg, r, cv::Scalar(0,255,0), 2);
            }    
           


            // draw skeletons of all persons
            if( enableGestures_ )
                drawSkeletonsOnImg(debugImg);

            Scalar distanceColor(Scalar(0,255,0));
            if(  distance > maxDistance_ ){
               distanceColor = Scalar(0,0,255);
            }
            
            Scalar angleColor(Scalar(0,255,0));
            
            putText(debugImg, "distance(M): " + to_string(distance), topLeft1,
                    0.2, 1, distanceColor, 3);
            putText(debugImg, "deg_angle: " + to_string(degAngle), topLeft2,
                    0.2, 1, angleColor, 3);
            putText(debugImg, state_, topLeft3,
                    0.2, 1, angleColor, 3);
            
            // cv::imshow("debugImg", debugImg);
            // cv::waitKey(1);

            sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugImg).toImageMsg();
    
            debugImgPublisher_.publish(msg);
        }
    }

    double calculateDistanceFromObjectInMeter(object_msgs::ObjectInBox object)
    {
        double distanceCm = (knownWidthCm_ * focalLength_) / object.roi.width; //in cm

        //convert to meters

        return (distanceCm / 100.0);
    }

    double calculateAngleFromObject(object_msgs::ObjectInBox object)
    {

        double image_width_pixels = currentImg_.cols; //CAMERA_HORIZONTAL_RESOLUTION_PIXELS;
        int x = object.roi.x_offset + (object.roi.width / 2);
        double angle_radians = atan((x - 0.5 *  currentImg_.cols) / focalLength_);

        return angles::to_degrees(angle_radians);
    }

    void sendVelCmd(double distance, double degAngle) {
       
        geometry_msgs::Twist command;
        command.linear.x = speed_;
        command.angular.z = -(angles::from_degrees(degAngle)) * angularScaleFactor_;
        twistCommandPublisher_.publish(command);
     
    }
   

private:

    ros::Subscriber rgbSubscriber_;

    ros::Subscriber objectSubscriber_;

    ros::Subscriber skeletonsSubscriber_; // just for drawing the skeletons
    ros::Subscriber gesturTypeSub_;

    ros::Publisher twistCommandPublisher_;

    image_transport::Publisher debugImgPublisher_;

    ros::Timer updateTimer_;

    string trackingTargetClassName_;

    double focalLength_;

    double minDistance_;
    double maxDistance_;

    double speed_;
    double maxSpeed;


    cv::Mat currentImg_;

    double knownWidthCm_;

    double distance;

    double degAngle;

    bool foundTarget_ = false;

    object_msgs::ObjectInBox nearestObjectInBox;

    double imgWidth = -1;

    double boxPercentToStop_;

    double angularScaleFactor_;

    bool enableGestures_ = false;

    ros::WallTime lastDetection_;

    vector<Person> currentPersons_;

    string state_ = "IDLE";

    GESTURE_FLAGE flag_ = NONE;

    bool initPublishZeroCmd_ = false;

    cv::Point trackingTargetPosition_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_follower_rgb_node");
    ros::NodeHandle node;
    PersonFollowerRgb follower;
    ros::spin();
    return 0;
}
