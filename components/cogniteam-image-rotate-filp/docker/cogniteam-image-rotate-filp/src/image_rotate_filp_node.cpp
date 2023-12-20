/*
 * image_rotate_filp_node.cpp
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
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ImageRotateFlip
{

public:
    ImageRotateFlip()
    {

        ros::NodeHandle node_;
        ros::NodeHandle nodePrivate("~");

       
        rgbSubscriber_ = node_.subscribe("/camera/color/image_raw", 1,
                                         &ImageRotateFlip::imageCallback, this);  

        image_transport::ImageTransport it(node_);  
        imgPublisher_ = it.advertise("/out_img", 1);
            node_.param("/out_img/compressed/jpeg_quality", 20);                                         

        nodePrivate.param("angle_rotation_degrees", angleRotation_, 0);

        nodePrivate.param("flip_both", flipBoth_, false);

        nodePrivate.param("flip_vertical", flipVertical_, false);

        nodePrivate.param("flip_horizontal", flipHorizontal_, false);


    } 

    ~ImageRotateFlip() {}
   
   

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            if(angleRotation_ != 0 ){
                Point2f src_center(img.cols/2.0F, img.rows/2.0F);
                Mat rot_mat = getRotationMatrix2D(src_center, angleRotation_, 1.0);
                warpAffine(img, img, rot_mat, img.size());
            }

            if(flipBoth_){

                cv::flip(img, img, -1);

            } else {

                if (flipVertical_ ){

                    cv::flip(img, img, 0);
                }

                if( flipVertical_ ){

                    cv::flip(img, img, 0);
                }

            } 
            

            sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();

            msg->header.frame_id = msg->header.frame_id;
            msg->header.stamp = msg->header.stamp;
            msg->header.seq = msg->header.seq;

            imgPublisher_.publish(msg);
                        
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    

private:

    ros::Subscriber rgbSubscriber_;   

    bool enableGestures_ = false;

    image_transport::Publisher imgPublisher_;

    int angleRotation_;

    bool flipBoth_;

    bool flipVertical_;

    bool flipHorizontal_;
     
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_rotate_filp_node");
    ros::NodeHandle node;
    ImageRotateFlip follower;
    ros::spin();
    return 0;
}
