/**
 *  File name: LynxGPsLocalization.h
 *     Author: Daria Syvash <daria@cogniteam.com>
 * Created on: Aug 22, 2019
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


#ifndef INCLUDE_LYNX_GPS_LOCALISATION_H_
#define INCLUDE_LYNX_GPS_LOCALISATION_H_


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

#include <lynx_localization/ComplimentaryFilter.h>
#include <lynx_localization/LynxLocalizationConfig.h>


namespace lynx {


using namespace std;


class GpsLocalization {

public: 

    GpsLocalization();

    virtual ~GpsLocalization();

private:
    /**
     * @brief ROS interface callbacks
     */
    void gpsCallback(const sensor_msgs::NavSatFixConstPtr&);

    void odomCallback(const nav_msgs::OdometryConstPtr&);

    /**
     * @brief Create UTF object from NavSat gps message
     */
    void navSatToUTM(const sensor_msgs::NavSatFixConstPtr&, geodesy::UTMPoint&);

    /**
     * @brief Create a Transform From Utf object
     * 
     * @return tf::Transform  - ROS - frame transformation
     */
    tf::Transform createTransformFromUTM(const geodesy::UTMPoint& );
    /**
     * @brief gps position point publisher
     */
    void publishGpsPose(ros::Time, tf::Transform);

    /**
     * @brief set gps position to tf
     */
    void setNewTransformFromGPS(const sensor_msgs::NavSatFixConstPtr&);

    void publishOdom(const ros::Time&);

    /**
     * @brief set first gps message data to tf
     */
    void setDatumTransform(const sensor_msgs::NavSatFixConstPtr&);

    /**
     * @brief Dtnamic reconfigure callback 
     */
    void dynamicReconfigureServerCallback(
            lynx_localization::LynxLocalizationConfig&, uint32_t);

private:

    /**
     * @brief ROS interface
     */

    ros::Subscriber gpsSubscriber_;
    
    ros::Subscriber odomSubscriber_;

    ros::Publisher odomPublisher_;

    ros::Publisher gpsLocalPositionPublisher_;

    ros::Publisher datumNavsatPublisher_;

    /**
     * @brief odom->base publisher 
     */

    tf::TransformBroadcaster tfPublisher_;

    /**
     * @brief Previous position vector storage
     */

    tf::Vector3 previousVector_;

    tf::Transform lastOdomTranform_ = tf::Transform::getIdentity();

    bool odomMsgReceived_ = false;

    /**
     * @brief GPS messages counter 
     */
    int gpsMsgCounter_;

    /**
     * @brief GPS position transformation
     */
    tf::Transform transform_ = tf::Transform::getIdentity();

    /**
     * @brief datum transformation
     */
    tf::Transform datumTransform_ = tf::Transform::getIdentity();

    ComplimentaryFilter* filter_;

    dynamic_reconfigure::Server<
            lynx_localization::LynxLocalizationConfig> reconfigureServer_;

    lynx_localization::LynxLocalizationConfig config_;

    string tfPrefix_;

};


} /* namespace lynx */


#endif /* INCLUDE_LYNX_GPS_LOCALISATION_H_ */
