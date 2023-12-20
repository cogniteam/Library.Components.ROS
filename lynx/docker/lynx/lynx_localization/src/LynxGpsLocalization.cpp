/**
 *  File name: LynxGpsLocalistaion.cpp
 *     Author: Daria Syvash <daria@cogniteam.com>
 * Created on: Aug 24, 2019
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


#include <lynx_localization/LynxGpsLocalization.h>


namespace lynx {


GpsLocalization::GpsLocalization() 
    : odomMsgReceived_(false), gpsMsgCounter_(0),
      reconfigureServer_(ros::NodeHandle("~/gps_odom")) {
    
    filter_ = new ComplimentaryFilter();

    reconfigureServer_.setCallback(
            boost::bind(&GpsLocalization::dynamicReconfigureServerCallback, this, _1, _2));

    ros::NodeHandle node;
    ros::NodeHandle nodePrivate("~");

    nodePrivate.param<string>("tf_prefix", tfPrefix_, "");

    gpsSubscriber_ = node.subscribe(
            "sensors/gps/fix", 1, &GpsLocalization::gpsCallback, this);

    odomSubscriber_ = node.subscribe(
            "odom", 1, &GpsLocalization::odomCallback, this);

    odomPublisher_ = node.advertise<nav_msgs::Odometry>(
            "odom_filtered", 5);
            
    datumNavsatPublisher_ = node.advertise<sensor_msgs::NavSatFix>(
            "events/localization/datum", 1, true);

    gpsLocalPositionPublisher_ = node.advertise<geometry_msgs::PointStamped>(
            "gps_local", 5);

}

GpsLocalization::~GpsLocalization() {
    delete filter_;
    filter_ = NULL;
}

void GpsLocalization::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gpsMsg) {

    if (!odomMsgReceived_) { 
        return;
    }

    if (gpsMsg->status.status != sensor_msgs::NavSatStatus::STATUS_FIX) {
        return;
    }

    if (gpsMsgCounter_ < config_.skip_gps_msg) {
        gpsMsgCounter_ ++;
        return;
    } else if (gpsMsgCounter_ == config_.skip_gps_msg) {
        setDatumTransform(gpsMsg);
        gpsMsgCounter_ ++;
        return;
    } else {
        ROS_INFO_ONCE("GPS stream started");
        setNewTransformFromGPS(gpsMsg);

    }

    publishOdom(ros::Time::now());
}

void GpsLocalization::odomCallback(const nav_msgs::OdometryConstPtr& odomMsg) {

    if (!odomMsgReceived_) {
        tf::poseMsgToTF(odomMsg->pose.pose, lastOdomTranform_);
        odomMsgReceived_ = true;
        return;
    }
    
    //
    // Update orientation
    //

    tf::Quaternion q;
    tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, q);
    transform_.setRotation(q);

    //
    // Make one relative step
    //
    tf::Transform odomTransform;
    tf::poseMsgToTF(odomMsg->pose.pose, odomTransform);

    auto relativeTransform = lastOdomTranform_.inverse() * odomTransform;
    auto distance = relativeTransform.getOrigin().length();

    if (relativeTransform.getOrigin().x() < 0) {
        distance = -distance;
    }

    tf::Transform odometryStep(
            tf::Quaternion::getIdentity(), 
            tf::Vector3(distance, 0, 0));

    transform_ = transform_ * odometryStep;

    //
    // Save last odom message
    //
    tf::poseMsgToTF(odomMsg->pose.pose, lastOdomTranform_);

    publishOdom(ros::Time::now());
}

void GpsLocalization::navSatToUTM(
        const sensor_msgs::NavSatFixConstPtr& gpsMsg, geodesy::UTMPoint& utfPosition) {
    geographic_msgs::GeoPoint geoPointMsg;
    geoPointMsg = geodesy::toMsg(*gpsMsg);
    fromMsg(geoPointMsg, utfPosition);
}

tf::Transform GpsLocalization::createTransformFromUTM(
        const geodesy::UTMPoint& utmPoint) {

    if (geodesy::isValid(utmPoint)) {
        tf::Vector3 vector(utmPoint.easting, utmPoint.northing, 0);
        auto zeroRotation = tf::Quaternion::getIdentity();
        tf::Transform transform(zeroRotation, vector);
        return transform;
    } else {
        ROS_ERROR("Cannot create transformation.Bad UTM");
        return tf::Transform::getIdentity();
    }
}

void GpsLocalization::publishGpsPose(ros::Time time, tf::Transform localGpsPoint) {
    geometry_msgs::PointStamped gpsPoint;
    tf::pointTFToMsg(localGpsPoint.getOrigin(), gpsPoint.point);
    gpsPoint.header.stamp = time;
    gpsPoint.header.frame_id = config_.fixed_frame;

    if (tfPrefix_ != "") {
        gpsPoint.header.frame_id = tfPrefix_ + "/" + gpsPoint.header.frame_id;
    }

    gpsLocalPositionPublisher_.publish(gpsPoint);
}

void GpsLocalization::setDatumTransform(const sensor_msgs::NavSatFixConstPtr& gpsMsg) {
    
    geodesy::UTMPoint datumGps;
    navSatToUTM(gpsMsg, datumGps);

    tf::Transform odomTf = lastOdomTranform_;
    odomTf.setRotation(tf::Quaternion::getIdentity());
    datumTransform_ = createTransformFromUTM(datumGps) * odomTf.inverse();
    datumTransform_.setRotation(tf::Quaternion::getIdentity());

    
    sensor_msgs::NavSatFix datumNavsat;
    datumGps.easting -= lastOdomTranform_.getOrigin().x();
    datumGps.northing -= lastOdomTranform_.getOrigin().y();
    auto datumGeoPoint = geodesy::toMsg(datumGps);
    
    datumNavsat.altitude = 0;
    datumNavsat.latitude = datumGeoPoint.latitude;
    datumNavsat.longitude = datumGeoPoint.longitude;
    datumNavsat.header.stamp = ros::Time::now();
    datumNavsat.header.frame_id = "base_link";

    if (tfPrefix_ != "") {
        datumNavsat.header.frame_id = tfPrefix_ + "/" + datumNavsat.header.frame_id;
    }

    datumNavsatPublisher_.publish(datumNavsat);

    ROS_INFO("Datum set");
}

void GpsLocalization::publishOdom(const ros::Time& time) {
    
    if (config_.publish_tf) {

        string fixedFrame = config_.fixed_frame;
        string baseFrame = config_.base_frame;


        if (tfPrefix_ != "") {
            fixedFrame = tfPrefix_ + "/" + fixedFrame;
            baseFrame = tfPrefix_ + "/" + baseFrame;
        }

        //
        // Odometry transform
        //
        tfPublisher_.sendTransform(tf::StampedTransform(transform_,
                ros::Time::now(), fixedFrame, baseFrame));

        //
        // Datum transform
        //
        tfPublisher_.sendTransform(tf::StampedTransform(datumTransform_,
                ros::Time::now(), config_.datum_frame, fixedFrame));

        //
        // Odometry message
        //        
        nav_msgs::Odometry odomMsg;

        odomMsg.header.frame_id = fixedFrame;
        odomMsg.header.stamp = time;
        odomMsg.child_frame_id = baseFrame;


        tf::poseTFToMsg(transform_, odomMsg.pose.pose);

        odomPublisher_.publish(odomMsg);
    }
    
}

void GpsLocalization::setNewTransformFromGPS(
        const sensor_msgs::NavSatFixConstPtr& gpsMsg) {

    geodesy::UTMPoint positionGps;
    navSatToUTM(gpsMsg, positionGps);
    
    auto gpsLocalTransform = createTransformFromUTM(positionGps);

    gpsLocalTransform = datumTransform_.inverse() * gpsLocalTransform;

    publishGpsPose(gpsMsg->header.stamp, gpsLocalTransform);
    
    transform_ = filter_->getFilteredPosition(gpsLocalTransform, transform_);
}

void GpsLocalization::dynamicReconfigureServerCallback(
        lynx_localization::LynxLocalizationConfig& config, uint32_t level) {
    config_ = config;
    filter_->setPoseWeight(config_.gps_trust);
}

} /* namespace lynx */

