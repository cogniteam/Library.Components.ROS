#include <depth_to_grayscale/DepthToGrayscaleConvertor.h>


DepthToGrayscaleConvertor::DepthToGrayscaleConvertor(){   

    dpethSub_ = node_.subscribe("/zed/zed_node/depth/depth_registered/", 1,
         &DepthToGrayscaleConvertor::depthCb, this); 


    image_transport::ImageTransport it(node_);

    grayscalePub_ = it.advertise("/grayscale_depth_img", 1);

    node_.param("/grayscale_depth_img/compressed/jpeg_quality", 20);

    node_.param("/min_gray_scale_value", min_gray_scale_value_, 0);
    
    node_.param("/max_gray_scale_value", max_gray_scale_value_, 255);

    node_.param("/max_distance", max_distance_, 3000);

    

        
}

void DepthToGrayscaleConvertor::depthCb(const sensor_msgs::ImageConstPtr& msg_depth )
{
    cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    
    Mat grayscaleImg;
    Mat tmp = img_ptr_depth->image;   
    double min, max;
    cv::minMaxLoc(tmp, &min, &max); 
    tmp.setTo(max_distance_, tmp > max_distance_);
    tmp.convertTo( grayscaleImg , CV_8UC1, 255/(max_distance_- min));
    normalize(grayscaleImg, grayscaleImg, max_gray_scale_value_, min_gray_scale_value_, NORM_MINMAX);
    // imshow("grayscaleImg",grayscaleImg);
    // waitKey(1);


    sensor_msgs::ImagePtr msg = 
        cv_bridge::CvImage(std_msgs::Header(), "mono8", grayscaleImg).toImageMsg();
    
    grayscalePub_.publish(msg);

}
