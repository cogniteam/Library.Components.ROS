

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
 
#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
 
using namespace std;

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_camera");
   
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    nh.param("/camera/image/compressed/jpeg_quality", 20); // asaf 19/07/21


    int capture_width = 640 ;
    int capture_height = 480 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 30 ;
    int flip_method = 2 ;
 
    std::string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);
    std::cerr << "Using pipeline: \n\t" << pipeline << "\n";
  
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cerr<<"Failed to open camera."<<std::endl;
        return (-1);
    }
 
    cv::Mat img;

    ros::Rate rate(framerate); // ROS Rate at 5Hz
    while (ros::ok()) {
       
       if (!cap.read(img)) {
            std::cerr<<"Capture read error"<<std::endl;
            continue;
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

        pub.publish(msg);

        rate.sleep();
    }
 
    cerr <<" error with camera "<<endl;
    cap.release();
    return 0;
}
