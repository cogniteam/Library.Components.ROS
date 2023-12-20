/*
 * ObjectsPoseEstimation.h
 *
 *  Created on: Jun 12, 2020
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

#include <boost/algorithm/string.hpp>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <object_msgs/ObjectsInBoxes.h>

#include <cv_bridge/cv_bridge.h>

#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#ifndef OBJECTS_POSE_ESTIMATION_H_
#define OBJECTS_POSE_ESTIMATION_H_

using namespace std;
using namespace cv;
using namespace visualization_msgs;


class ObjectsPoseEstimation
{

public:

    ObjectsPoseEstimation() {

        ros::NodeHandle nodePrivate("~");

        objectSubscriber_ = node_.subscribe("openvino_toolkit/detected_objects", 1,
                                            &ObjectsPoseEstimation::detectedObjectsCallback, this);

        image_sub.subscribe(node_, "/camera/depth/image_rect", 1);
        image_sub.registerCallback(&ObjectsPoseEstimation::depthCallback, this);

        info_sub.subscribe(node_, "/camera/depth/image_rect/camera_info", 1);
        info_sub.registerCallback(&ObjectsPoseEstimation::cameraInfoCallback, this);      

        nodePrivate.param("map_frame_id", map_frame_id_, string("map"));

        nodePrivate.param("static_objects_life_time_seconds", lifetimeDuration_, 0.2);


        objectsMarkerArr_ = node_.advertise<visualization_msgs::MarkerArray>("/objects_on_map_markers", 10);
    }

    ~ObjectsPoseEstimation(){};

    void detectedObjectsCallback(const object_msgs::ObjectsInBoxes &objects) {
        ObjectsInBoxes_ = objects;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info) {

        if (cameraInfoInited_ == false)
        {
            if (pinholeCameraModel_.fromCameraInfo(*cam_info))
            {
                cameraInfoInited_ = true;
            }
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &image) {

        if (cameraInfoInited_) {

            depth_camera_link_ = image->header.frame_id;

            cv::Mat depthColor;
            cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat depth = cvBridgePtr->image;

            cv::resize(depth, depth, cv::Size(640, 480) );

            double minVal;
            double maxVal;
            minMaxLoc(depth, &minVal, &maxVal);
            depth.convertTo(depthColor, CV_8UC1, (255 / (maxVal - minVal)));

            cvtColor(depthColor, depthColor, CV_GRAY2BGR);

            object_msgs::ObjectsInBoxes obstaclesOnMap;
            obstaclesOnMap.header.frame_id = map_frame_id_;
            obstaclesOnMap.header.stamp = ros::Time::now();

            visualization_msgs::MarkerArray markers;
            for (int i = 0; i < ObjectsInBoxes_.objects_vector.size(); i++) {

                cv::Rect object_rect(ObjectsInBoxes_.objects_vector[i].roi.x_offset,
                    ObjectsInBoxes_.objects_vector[i].roi.y_offset,
                    ObjectsInBoxes_.objects_vector[i].roi.width,
                    ObjectsInBoxes_.objects_vector[i].roi.height);

                cv::rectangle(depthColor, object_rect, cv::Scalar(0, 0, 255), 1);

                geometry_msgs::PointStamped objectPose =
                    extractDepthFromBboxObject(ObjectsInBoxes_.objects_vector[i], depth);

                visualization_msgs::Marker marker;
                marker.lifetime = ros::Duration(0.2);
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ObjectsInBoxes_.header.stamp;
                marker.id = rand();
                marker.pose.orientation.w = 1.0;               
                marker.pose.position.x = objectPose.point.x;
                marker.pose.position.y = objectPose.point.y;
                marker.pose.position.z = objectPose.point.z;                
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.text = ObjectsInBoxes_.objects_vector[i].object.object_name;

                if( marker.text != "person"){
                    marker.lifetime = ros::Duration(lifetimeDuration_);
                } 

                markers.markers.push_back(marker);              
            }

            ObjectsInBoxes_.objects_vector.clear();

            // imshow("depthColor " + robot_id_,depthColor);
            // waitKey(1);

            objectsMarkerArr_.publish(markers);
        }
    }

    geometry_msgs::PointStamped extractDepthFromBboxObject(object_msgs::ObjectInBox bbox_object,
            const Mat &depthImg) {

        float cx = pinholeCameraModel_.cx();
        float cy = pinholeCameraModel_.cy();

        float fx = pinholeCameraModel_.fx();
        float fy = pinholeCameraModel_.fy(); 

        cv::Point2d centerObject( bbox_object.roi.x_offset + (bbox_object.roi.width / 2),
            bbox_object.roi.y_offset + (bbox_object.roi.height / 2 ) );

        float d = depthImg.at<float>(centerObject.y, centerObject.x) / 1000; /// IN METERS
        cv::Point3d p = cv::Point3d(((centerObject.x - cx) * d / fx), ((centerObject.y - cy) * d / fy), d);

        auto objectPose = transformToMapFrame(p);

        return objectPose;
    }
    
    geometry_msgs::PointStamped transformToMapFrame(
        Point3d objectPoint3d) const {

      
        string mapFrame_ =   map_frame_id_;    
        string cameraLink =  depth_camera_link_;


        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = cameraLink;
        pointStampedIn.header.stamp = ros::Time(0);
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;
        
        if (tfListener_.waitForTransform(mapFrame_, cameraLink, 
                ros::Time(0), ros::Duration(0.1))) {

            tfListener_.transformPoint(mapFrame_, pointStampedIn, pointStampedOut);
            return pointStampedOut;

        } else {
            ROS_ERROR("Failed to find transform between  frames");
            return pointStampedOut;
        }

    }

private:

    ros::NodeHandle node_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;

    ros::Subscriber objectSubscriber_;

    ros::Publisher objectsMarkerArr_;

    object_msgs::ObjectsInBoxes ObjectsInBoxes_;

    bool cameraInfoInited_ = false;

    image_geometry::PinholeCameraModel pinholeCameraModel_;

    string map_frame_id_;

    string depth_camera_link_;

    ros::Publisher objectsOnMapPub_;

    tf::TransformListener tfListener_;

    double lifetimeDuration_;
};

#endif /* OBJECTS_POSE_ESTIMATION_H_ */