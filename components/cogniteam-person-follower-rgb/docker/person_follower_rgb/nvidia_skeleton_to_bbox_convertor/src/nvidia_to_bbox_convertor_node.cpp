/*
 * nvidia_to_bbox_convertor_node.cpp
 *
 *  Created on: May 13, 2019
 *      Author: Igor Makhtes <igor@cogniteam.com>
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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string
#include <map>
#include <iterator>
#include <math.h>


#include <ros/ros.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <std_msgs/String.h>

//for jetson-inference detection
#include <vision_msgs/Detection2DArray.h>

#include <nlohmann/json.hpp> //https://github.com/nlohmann/json#cmake
using json = nlohmann::json;
//git clone https://github.com/nlohmann/json.git
// mkdir build
//cd build
// cmake ..
//make
//make install
//ldconfig

using namespace cv;
using namespace std;

struct Person
{

    std::map<std::string, Point2d> jointsWithLocations;
    cv::Rect boundingBox;
};

class SkeletonToBboxMsgConvertor {

public:

    SkeletonToBboxMsgConvertor(){

         skeletonsSubscriber_ = node_.subscribe("/skeletons_text", 1,
            &SkeletonToBboxMsgConvertor::skeletonsMsgCallback, this);

        openVinoObjectsPub_ = 
            node_.advertise<object_msgs::ObjectsInBoxes>("openvino_toolkit/detected_objects",1);

    }

    ~SkeletonToBboxMsgConvertor(){}

public:

    void skeletonsMsgCallback(const std_msgs::String::Ptr &msg) {
        

        // parse the json
        vector<Person> currentPersons;

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

            if( person.jointsWithLocations.size() >= 3){ 
                person.boundingBox = calculateBoundingBox(person);

                if( person.boundingBox.width > 0 ){

                    currentPersons.push_back(person);

                }
            }
            
        }

        object_msgs::ObjectsInBoxes openVinoObjects;

        openVinoObjects.objects_vector.resize( currentPersons.size());

        for(int i =0; i < currentPersons.size(); i++) {

            object_msgs::ObjectInBox  objectInBox; 
            objectInBox.roi.width = currentPersons[i].boundingBox.width;
            objectInBox.roi.height = currentPersons[i].boundingBox.height;
            objectInBox.object.object_name = "person";

            objectInBox.roi.y_offset = currentPersons[i].boundingBox.tl().y;
              
            objectInBox.roi.x_offset = currentPersons[i].boundingBox.tl().x;  
               

            openVinoObjects.objects_vector[i] = objectInBox;
        }

        openVinoObjectsPub_.publish(openVinoObjects);
    }

    cv::Rect calculateBoundingBox(const Person &person)
    {

        vector<cv::Point> points;

        for (auto it = person.jointsWithLocations.begin(); it != person.jointsWithLocations.end(); ++it)
        {
            cv::Point p(it->second.x, it->second.y);
            points.push_back(p);
        }

        cv::Rect r = cv::boundingRect(points);

        return r;
    }
   
private:

    ros::NodeHandle node_;
    
    ros::Subscriber skeletonsSubscriber_;

    ros::Publisher openVinoObjectsPub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "skeleton_to_bbox_convertor");
  
    SkeletonToBboxMsgConvertor SkeletonToBboxMsgConvertor;

    ros::spin();
    return 0;
}
