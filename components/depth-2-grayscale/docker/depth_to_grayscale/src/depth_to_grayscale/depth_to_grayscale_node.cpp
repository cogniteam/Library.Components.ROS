
#include <depth_to_grayscale/DepthToGrayscaleConvertor.h>





int main(int argc, char **argv){


    ros::init(argc, argv,"depth_to_grayscale_node");

    DepthToGrayscaleConvertor DepthToGrayscaleConvertor;     

    ros::spin();
    return 0;
}
