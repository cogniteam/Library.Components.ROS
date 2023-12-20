/*
 * theora_publisher_node.cpp
 *
 *  Created on: May 05, 2021
 *      Author: lin azan
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "theora_image_transport/Packet.h"

using namespace std;

class TheoraPublisher
{

public:
  TheoraPublisher(ros::NodeHandle *nodehandle)
  {
    ros::NodeHandle nodePrivate("~");
    nh_ = *nodehandle;

    nodePrivate.param("time_interval", timeInterval_, 5.0);
    subscriber_ = nh_.subscribe("/video/theora", 1000, &TheoraPublisher::theoraCallback, this);
    publisher_ = nh_.advertise<theora_image_transport::Packet>("/video/theora", 1000);
    timer = nh_.createTimer(ros::Duration(timeInterval_), &TheoraPublisher::timerCallback, this);
    packetsMsgs_.resize(3);
  }

  ~TheoraPublisher() {}

private:
  ros::NodeHandle nh_;

  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::Timer timer;
  std::vector<theora_image_transport::Packet> packetsMsgs_;
  std::map<int, theora_image_transport::Packet> packetsMsgsMap_;
  double timeInterval_;

private:
  void theoraCallback(const theora_image_transport::Packet &msg)
  {
    int num = msg.packetno;
    
    if (num == 0 || num == 1 || num == 2)
    {
      packetsMsgsMap_[num] = msg;
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    for (auto const &x : packetsMsgsMap_)
    {
      publisher_.publish(x.second);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "theora_node");

  ros::NodeHandle n;
  TheoraPublisher theoraPublisher_(&n);

  ros::spin();

  return 0;
}