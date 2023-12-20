/*
 * person_skeleton_follower_node.cpp
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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <map>
#include <iterator>
#include <math.h>

#include <nlohmann/json.hpp> //https://github.com/nlohmann/json#cmake
using json = nlohmann::json;
//git clone https://github.com/nlohmann/json.git
// mkdir build
//cd build
// cmake ..
//make
//make install
//ldconfig

using namespace std;
using namespace cv;

enum FOLLOWER_STATE
{
    IDLE,
    TRACKING,
    STOPPING
};

struct Person
{

    std::map<std::string, Point2d> jointsWithLocations;
    cv::Rect boundingBox;
};

enum GESTURE
{
    RAISING_TWO_HANDS,
    RIGHT_HAND_RAISED,
    NONE
};

class PersonSkeletonFollower
{

public:
    PersonSkeletonFollower()
    {

        ros::NodeHandle node_;
        ros::NodeHandle nodePrivate("~");

        rgbSubscriber_ = node_.subscribe("/webcam/image_raw", 1,
                                         &PersonSkeletonFollower::imageCallback, this);

        skeletonsSubscriber_ = node_.subscribe("/skeletons_text", 1,
                                               &PersonSkeletonFollower::detectedSkeletonsCallback, this);

        /*image_transport::ImageTransport it(node_);
        debugImgPublisher_ = it.advertise("/skeletons_gesture_img", 1);

        node_.param("/debug_img/compressed/jpeg_quality", 20);*/

        updateTimer_ = node_.createTimer(ros::Rate(15),
                                         &PersonSkeletonFollower::updateTimerCallback, this);

        gestureTypePub_ =
            node_.advertise<std_msgs::String>("/gesture_type", 1, false);

        init_ = false;
    }

    ~PersonSkeletonFollower() {}

    void detectedSkeletonsCallback(const std_msgs::String::Ptr &msg)
    {   
        currentPersons_.clear();

        if (init_)
        {

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
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!init_)
        {

            try
            {
                currentImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                if (currentImg_.cols > 0 && currentImg_.rows > 0)
                {
                    imgWidth_ = currentImg_.cols;
                    imgHeight_ = currentImg_.rows;
                    init_ = true;
                }
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
        }
    }


    string getStringGesture(GESTURE gesture) {

        if( gesture ==  RAISING_TWO_HANDS){

            return "TWO_HNADS";
        }
        if( gesture ==  RIGHT_HAND_RAISED){

            return "RIGHT_HAND_UP";
        }
        if( gesture ==  NONE){

            return "NONE";
        }

        return "NONE";
    }

    void updateTimerCallback(const ros::TimerEvent &) {

        auto gesture = detectGesture();
        string sGesture = getStringGesture(gesture); 
        std_msgs::String msg;
        msg.data = sGesture;
        gestureTypePub_.publish(msg);

        currentPersons_.clear();

    }
    

    GESTURE detectGesture()
    {
        for(int i =0; i < currentPersons_.size(); i++ ){
            Person person = currentPersons_[i];
            if (detectTwoHandsSpread(person))
            {

                return RAISING_TWO_HANDS;
            }
            else if (detectRightHandUp(person))
            {

                return RIGHT_HAND_RAISED;
            }

        }
        

        return NONE;
    }

    bool detectRightHandUp(Person person)
    {

        if (person.jointsWithLocations.find("Rwri") != person.jointsWithLocations.end() && 
            person.jointsWithLocations.find("Relb") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Rsho") != person.jointsWithLocations.end()

            && person.jointsWithLocations.find("Lsho") != person.jointsWithLocations.end()
            && person.jointsWithLocations.find("Lelb") != person.jointsWithLocations.end() 
            && person.jointsWithLocations.find("Lwri") != person.jointsWithLocations.end()
            && person.jointsWithLocations.find("Reye") != person.jointsWithLocations.end())
        {

            cv::Point2d Rwri = person.jointsWithLocations.at("Rwri");

            cv::Point2d Relb = person.jointsWithLocations.at("Relb");

            cv::Point2d Lwri = person.jointsWithLocations.at("Lwri");

            cv::Point2d Reye = person.jointsWithLocations.at("Reye");

            cv::Point2d Lsho = person.jointsWithLocations.at("Lsho");


            /// doesnt see right hand 

            if( (Rwri.y == 0 && Rwri.x == 0) || (Relb.y == 0 && Relb.x == 0) 
                || (Lwri.y == 0 && Lwri.x == 0) || (Reye.y == 0 && Reye.x == 0) 
                || (Lsho.y == 0 && Lsho.x == 0)){

                return false;
            }    
            if (Rwri.y <= Reye.y &&
                Lwri.y > Lsho.y)
            {

                return true;
            }
        }

        return false;
    }

    bool detectTwoHandsSpread(Person person)
    {

        if (person.jointsWithLocations.find("Rwri") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Relb") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Rsho") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Lsho") != person.jointsWithLocations.end() && 
            person.jointsWithLocations.find("Lelb") != person.jointsWithLocations.end() &&
             person.jointsWithLocations.find("Lwri") != person.jointsWithLocations.end())
        {

            cv::Point2d Rwri = person.jointsWithLocations.at("Rwri");
            cv::Point2d Relb = person.jointsWithLocations.at("Relb");
            cv::Point2d Rsho = person.jointsWithLocations.at("Rsho");

            cv::Point2d Lsho = person.jointsWithLocations.at("Lsho");
            cv::Point2d Lelb = person.jointsWithLocations.at("Lelb");
            cv::Point2d Lwri = person.jointsWithLocations.at("Lwri");

            double minSqrErrorRightHand =
                (0.3333) * (double)(pow(Rwri.y - Relb.y, 2) + pow(Rwri.y - Rsho.y, 2) +
                                    pow(Relb.y - Rsho.y, 2));

            double maxSqrError = 170;
            double minSqrErrorLeftHand =
                (0.3333) * (double)(pow(Lsho.y - Lelb.y, 2) + pow(Lsho.y - Lwri.y, 2) +
                                    pow(Lelb.y - Lwri.y, 2));

            if (minSqrErrorRightHand < maxSqrError && minSqrErrorLeftHand < maxSqrError)
            {
                cerr << " minSqrErrorRightHand " << minSqrErrorRightHand << " minSqrErrorLeftHand " << minSqrErrorLeftHand << endl;

                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    string convertStateToString(FOLLOWER_STATE state)
    {

        switch (state)
        {
        case IDLE:
            return "IDLE";
        case TRACKING:
            return "TRACKING";
        case STOPPING:
            return "STOPPING";
        }
        return "IDLE";
    }

    string convertGestureString(GESTURE gesture)
    {

        switch (gesture_)
        {
        case NONE:
            return "NONE";
        case RAISING_TWO_HANDS:
            return "RAISING_TWO_HANDS";
        case RIGHT_HAND_RAISED:
            return "RIGHT_HAND_RAISED";
        }
        return "NONE";
    }

    void drawJoinstOnImg(cv::Mat &img, const Person &person)
    {

        for (auto it = person.jointsWithLocations.begin(); it != person.jointsWithLocations.end(); ++it)
        {
            cv::Point p(it->second.x, it->second.y);
            circle(img, p, 1, Scalar(255, 255, 255), FILLED, 8, 0);
        }

        // right hand
        if (person.jointsWithLocations.find("Rwri") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Relb") != person.jointsWithLocations.end() &&
            person.jointsWithLocations.find("Rsho") != person.jointsWithLocations.end())          {

            cv::Point2d Rwri = person.jointsWithLocations.at("Rwri");
            cv::Point2d Relb = person.jointsWithLocations.at("Relb");
            cv::Point2d Rsho = person.jointsWithLocations.at("Rsho");
            
            if(Rwri.x > 0 && Rwri.y > 0 && Rwri.x < imgWidth_ && Rwri.y < imgHeight_ 
                && Relb.x > 0 && Relb.y > 0 && Relb.x < imgWidth_ && Relb.y < imgHeight_){
                    cv::arrowedLine(img, Rwri, Relb, Scalar(255, 255, 0), 1);
            }

            if ( Rsho.x > 0 && Rsho.y > 0 && Rsho.x < imgWidth_ && Rsho.y < imgHeight_ 
                    && Relb.x > 0 && Relb.y > 0 && Relb.x < imgWidth_ && Relb.y < imgHeight_){
                cv::arrowedLine(img, Rsho, Relb,  Scalar(255, 255, 0), 1);
            } 
        }
       
        // left hand
        if ( person.jointsWithLocations.find("Lsho") != person.jointsWithLocations.end() && 
             person.jointsWithLocations.find("Lelb") != person.jointsWithLocations.end() &&
             person.jointsWithLocations.find("Lwri") != person.jointsWithLocations.end()) {

            cv::Point2d Lsho = person.jointsWithLocations.at("Lsho");
            cv::Point2d Lelb = person.jointsWithLocations.at("Lelb");
            cv::Point2d Lwri = person.jointsWithLocations.at("Lwri");

            if(Lsho.x > 0 && Lsho.y > 0 && Lsho.x < imgWidth_ && Lsho.y < imgHeight_ 
                && Lelb.x > 0 && Lelb.y > 0 && Lelb.x < imgWidth_ && Lelb.y < imgHeight_){
                    cv::arrowedLine(img, Lsho, Lelb, Scalar(255, 255, 0), 2);
            }

            if(Lelb.x > 0 && Lelb.y > 0 && Lelb.x < imgWidth_ && Lelb.y < imgHeight_ 
                && Lwri.x > 0 && Lwri.y > 0 && Lwri.x < imgWidth_ && Lwri.y < imgHeight_){
                    cv::arrowedLine(img, Lwri, Lelb, Scalar(255, 255, 0), 2);
            }
        }   

       
       
    }

    cv::Rect calculateBoundingBox(const Person &person)
    {

        vector<cv::Point> points;

        for (auto it = person.jointsWithLocations.begin(); it != person.jointsWithLocations.end(); ++it)
        {
            cv::Point p(it->second.x, it->second.y);
            if (p.x > 0 && p.y > 0 && p.x < imgWidth_ && p.y < imgHeight_)
            {
                points.push_back(p);
            }
        }

        cv::Rect r;
        if (points.size() > 4)
        {
            r = cv::boundingRect(points);
        }

        return r;
    }

private:
    FOLLOWER_STATE followerState_;

    ros::Subscriber rgbSubscriber_;

    ros::Subscriber skeletonsSubscriber_;

    ros::Publisher gestureTypePub_;

    image_transport::Publisher debugImgPublisher_;

    ros::Timer updateTimer_;

    cv::Mat currentImg_;

    bool foundTarget = false;

    vector<Person> currentPersons_;

    double imgWidth_ = -1;

    double imgHeight_ = -1;

    GESTURE gesture_ = NONE;

    bool init_;

    ros::WallTime lastDetection_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_skeletion_follower_node");
    ros::NodeHandle node;
    PersonSkeletonFollower follower;
    ros::spin();
    return 0;
}
