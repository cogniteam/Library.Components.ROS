#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("/add_two_ints");
  beginner_tutorials::AddTwoInts srv;
   ros::Rate loop_rate(0.1);

  while (ros::ok()){
    srv.request.a = rand() % 100; 
    srv.request.b = rand() % 100;

    if (client.call(srv))
    {
     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
    }
  // ros::spin();
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
