

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Vector3.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

class CircleDetection
{

public:
    CircleDetection()
    {

        ros::NodeHandle node_;
        ros::NodeHandle nodePrivate("~");

        rgbSubscriber_ = node_.subscribe("/usb_cam/image_raw", 1,
                                         &CircleDetection::imageCallback, this);

        image_transport::ImageTransport it(node_);

        circleDetectionPublisher_ = it.advertise("/circle_img", 1);
        nodePrivate.param("/circle_img/compressed/jpeg_quality", 20);

        circlePublisher_ = node_.advertise<geometry_msgs::Vector3>("/largest_circle", 1);

        nodePrivate.param("min_dist_between_two_circles", minDistBetweenTwoCircles_,  1.0); 
        nodePrivate.param("canny_high_threshold", cannyHighThreshold_,  100.0); 
        nodePrivate.param("min_number_of_votes", minNumberOfVotes_,  150); 
        nodePrivate.param("min_radius", minRadius_,  0); 
        nodePrivate.param("max_radius", maxRadius_,  100);    

    }

    ~CircleDetection() {}

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {

        try
        {

            Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            Mat colorImg = frame.clone();
            cvtColor(frame, frame, COLOR_BGR2GRAY);
            medianBlur(frame, frame, 5);

            vector<Vec3f> circles;
            HoughCircles(frame, circles, CV_HOUGH_GRADIENT,
                         2,       // accumulator resolution (size of the image / 2)
                          minDistBetweenTwoCircles_,       // minimum distance between two circles
                         cannyHighThreshold_,     // Canny high threshold
                         minNumberOfVotes_,     // minimum number of votes
                         minRadius_, maxRadius_); // min and max radius

            //find the largset circle
            float maxR = 0;
            int index = -1;
            for (size_t i = 0; i < circles.size(); i++)
            {
                Vec3i c = circles[i];
                int radius = c[2];

                if (radius > maxR)
                {
                    maxR = radius;
                    index = i;
                }
            }

            //publish the largest circle

            if (index != -1)
            {

                geometry_msgs::Vector3 circleMsg;
                circleMsg.x = circles[index][0];
                circleMsg.y = circles[index][1];
                circleMsg.z = circles[index][2];

                cv::Point center = Point(circleMsg.x, circleMsg.y);
                circle(colorImg, center, circleMsg.z, Scalar(255, 0, 255), 3, LINE_AA);

                circlePublisher_.publish(circleMsg);
            }

            //publish the img
            sensor_msgs::ImagePtr msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImg).toImageMsg();

            circleDetectionPublisher_.publish(msg);

            // imshow("frame", frame);
            // imshow("colorImg", colorImg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:


    ros::Subscriber rgbSubscriber_;

    image_transport::Publisher circleDetectionPublisher_;

    ros::Publisher circlePublisher_;

    double minDistBetweenTwoCircles_;
    double cannyHighThreshold_;

    int  minNumberOfVotes_;
    int  minRadius_;
    int  maxRadius_;



    
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "circle_detection_node");

    CircleDetection circleDetection;

    ros::spin();
    return 0;

}