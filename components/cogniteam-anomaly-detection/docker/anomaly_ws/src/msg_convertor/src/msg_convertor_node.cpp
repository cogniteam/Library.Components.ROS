#include "../include/msg_convertor.h"
using topic_tools::ShapeShifter;


//TODO:: Understand And Describe topic in a generic way.

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_convertor");
    ros::NodeHandle node;
    MsgConvertor msgConvertor;
    ros::spin();
    std::cout<<"DONE"<<endl;
    return 0;
}

