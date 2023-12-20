#include "../include/algorithms.h"
#include "../../msg_convertor/include/msg_convertor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "algorithms");
    ros::NodeHandle node;
    Algorithms algorithms;
    ros::spin();
    std::cout<<"DONE"<<endl;
    return 0;
}