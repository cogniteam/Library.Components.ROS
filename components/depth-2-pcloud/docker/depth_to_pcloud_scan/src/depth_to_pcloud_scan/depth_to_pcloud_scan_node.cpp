



#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>


using namespace std;

class DepthToPCloudScan
{
private:
   
public:
    DepthToPCloudScan(){

        ros::NodeHandle nodePrivate("~");        

        depth_image_sub.subscribe(node_, "/camera/depth/image_rect_raw", 1);
        depth_image_sub.registerCallback(&DepthToPCloudScan::depthCallback, this);

        info_sub.subscribe(node_, "/camera/depth/camera_info", 1);
        info_sub.registerCallback(&DepthToPCloudScan::cameraInfoCallback, this); 

        pubPclLaser_ = node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >
         ("/pointcloud", 1, true); 

    }
     ~DepthToPCloudScan(){}

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

            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.header.frame_id = depth_camera_link_;

            for(int j =0; j<depth.rows ; j++){

                float* pixel = depth.ptr<float>(j);  
                
                for(int i = 0; i< depth.cols ; i++){

                    float distVal = pixel[i];    
                    cv::Point3d threeDPoint = convertPixTo3dPoint(cv::Point2d(j, i), distVal);
                    pcl::PointXYZRGB pRGB;                
                    pRGB.x =  threeDPoint.x;
                    pRGB.y =  threeDPoint.y;
                    pRGB.z = threeDPoint.z;                    
                  
                    uint8_t r = 0;
                    uint8_t g = 255;
                    uint8_t b = 0;

                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    pRGB.rgb = *reinterpret_cast<float*>(&rgb);

                    cloud.points.push_back(pRGB);                    

                }
            }


            pubPclLaser_.publish(cloud);   
        }
            
    }

    cv::Point3d convertPixTo3dPoint(const cv::Point2d p, float distVal) {

        float cx = pinholeCameraModel_.cx();
        float cy = pinholeCameraModel_.cy();

        float fx = pinholeCameraModel_.fx();
        float fy = pinholeCameraModel_.fy();

        float d = distVal / 1000; /// IN METERS
        cv::Point3d p3d = cv::Point3d(((p.x - cx) * d / fx), ((p.y - cy) * d / fy), d);

        return p3d;      
    }
    
   

private:

    ros::NodeHandle node_;

    ros::Publisher pubPclLaser_;

    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;

    bool cameraInfoInited_ = false;

    image_geometry::PinholeCameraModel pinholeCameraModel_;

    string map_frame_id_;

    string depth_camera_link_;

    tf::TransformListener tfListener_;

};







int main(int argc, char **argv){


    ros::init(argc, argv,"depth_to_pcloud_scan");

    DepthToPCloudScan depthToPCloudScan;     

    ros::spin();
    return 0;
}
